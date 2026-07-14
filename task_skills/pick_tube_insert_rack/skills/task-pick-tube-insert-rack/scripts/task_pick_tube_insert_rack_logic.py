#!/usr/bin/env python3
from __future__ import annotations

import json
import math
from pathlib import Path
from typing import Any


def _load(value: str | Path | dict[str, Any]) -> dict[str, Any]:
    if isinstance(value, dict):
        return value
    return json.loads(Path(value).expanduser().read_text(encoding="utf-8"))


def _point_xyz(point: Any, frame: str = "base") -> list[float] | None:
    if not isinstance(point, dict):
        return None
    candidate = point if all(axis in point for axis in ("x_m", "y_m", "z_m")) else point.get(frame)
    if not isinstance(candidate, dict):
        return None
    try:
        xyz = [float(candidate[axis]) for axis in ("x_m", "y_m", "z_m")]
    except (KeyError, TypeError, ValueError):
        return None
    if not all(math.isfinite(item) for item in xyz):
        return None
    return xyz


def _unit(vector: list[float]) -> list[float] | None:
    norm = math.sqrt(sum(item * item for item in vector))
    if norm <= 1e-12:
        return None
    return [item / norm for item in vector]


def parse_object_locator_result(path_or_dict: str | Path | dict[str, Any]) -> dict[str, Any]:
    result = _load(path_or_dict)
    detection = result.get("detection") if isinstance(result.get("detection"), dict) else {}
    parsed = {
        "found": bool(result.get("found", detection.get("found", False))),
        "label": detection.get("label") or result.get("target"),
        "confidence": detection.get("confidence", result.get("confidence", 0.0)),
        "position_base": result.get("position_base"),
        "points_base": result.get("points_base"),
        "orientation": result.get("orientation"),
        "detection": detection,
        "raw": result,
        "errors": [],
        "warnings": [],
    }
    if not parsed["found"]:
        parsed["errors"].append("object_locator_result_not_found")
    points_base = parsed.get("points_base")
    position_base = parsed.get("position_base")
    if not (isinstance(points_base, dict) and points_base.get("available")) and not (
        isinstance(position_base, dict) and position_base.get("available")
    ):
        parsed["errors"].append("base_frame_position_unavailable")
    return parsed


def extract_tube_geometry(result: str | Path | dict[str, Any], *, frame: str = "base") -> dict[str, Any]:
    parsed = parse_object_locator_result(result)
    raw = parsed["raw"]
    points = raw.get("points_base") if isinstance(raw.get("points_base"), dict) else {}
    warnings = list(parsed["warnings"])
    head = _point_xyz(points.get("head"), frame)
    tail = _point_xyz(points.get("tail"), frame)
    grasp = _point_xyz(points.get("tail_to_head_1_5"), frame) or _point_xyz(points.get("bbox_center"), frame)
    if head and tail:
        axis = _unit([head[i] - tail[i] for i in range(3)])
        mouth = axis
    else:
        axis = None
        mouth = None
        warnings.append("head_tail_base_points_unavailable; falling back to position_base/orientation if possible")
    if grasp is None:
        position_base = raw.get("position_base") if isinstance(raw.get("position_base"), dict) else {}
        if position_base.get("available"):
            grasp = _point_xyz(position_base, frame)
            warnings.append("using position_base as tube_xyz fallback")
    if grasp is None:
        return {"ok": False, "errors": parsed["errors"] + ["tube_xyz_unavailable"], "warnings": warnings}
    confidence = parsed.get("confidence")
    pose = {
        "frame": frame,
        "xyz_m": grasp,
        "rotvec_rad": None,
        "source": "object_locator.points_base",
        "confidence": confidence,
        "quality": {"head_point": head, "tail_point": tail},
    }
    return {
        "ok": True,
        "tube_xyz": grasp,
        "tube_pose": pose,
        "tube_mouth_direction": mouth,
        "tube_body_axis": axis,
        "head_point": head,
        "tail_point": tail,
        "confidence": confidence,
        "warnings": warnings,
        "raw": raw,
    }


def select_arm_for_tail_side(tube_geometry: dict[str, Any], deadband_m: float = 0.002, right_y_sign: str = "negative") -> dict[str, Any]:
    head = tube_geometry.get("head_point")
    tail = tube_geometry.get("tail_point")
    if not head or not tail:
        return {"ok": False, "status": "uncertain", "reason": "head_tail_points_required_for_tail_side_arm_selection"}
    dy = float(tail[1]) - float(head[1])
    if abs(dy) < float(deadband_m):
        return {"ok": False, "status": "uncertain", "reason": "tail_head_y_delta_below_deadband", "delta_y_m": dy}
    tail_is_right = dy < 0.0 if right_y_sign == "negative" else dy > 0.0
    selected = "right" if tail_is_right else "left"
    return {
        "ok": True,
        "selected_arm": selected,
        "opposite_arm": "left" if selected == "right" else "right",
        "reason": "selected arm on tail side, opposite tube mouth/head side",
        "delta_y_m": dy,
    }


def build_pregrasp_and_grasp_plan(
    tube_geometry: dict[str, Any],
    *,
    selected_arm: str,
    pregrasp_z_offset_m: float = 0.08,
    grasp_z_offset_m: float = 0.0,
) -> dict[str, Any]:
    xyz = [float(item) for item in tube_geometry["tube_xyz"]]
    pre = [xyz[0], xyz[1], xyz[2] + float(pregrasp_z_offset_m)]
    grasp = [xyz[0], xyz[1], xyz[2] + float(grasp_z_offset_m)]
    warnings = []
    if tube_geometry.get("tube_body_axis") is not None:
        warnings.append(
            "low-level grasp script currently aligns a gripper axis parallel to the tube axis; yaw perpendicular to body normal remains a policy-level target"
        )
    return {
        "selected_arm": selected_arm,
        "pregrasp_pose": {"frame": "base", "xyz_m": pre, "rotvec_rad": None, "source": "task_logic.pregrasp"},
        "grasp_pose": {"frame": "base", "xyz_m": grasp, "rotvec_rad": None, "source": "task_logic.grasp"},
        "warnings": warnings,
    }


def wrap_hole_candidates(result: str | Path | dict[str, Any]) -> list[dict[str, Any]]:
    data = _load(result)
    if isinstance(data.get("candidates"), list):
        candidates = list(data["candidates"])
    else:
        detection = data.get("detection") if isinstance(data.get("detection"), dict) else {}
        position_base = data.get("position_base") if isinstance(data.get("position_base"), dict) else {}
        xyz = _point_xyz(position_base) if position_base.get("available") else None
        candidates = [
            {
                "label": detection.get("label", "single_hole"),
                "confidence": detection.get("confidence", data.get("confidence", 0.0)),
                "pose": None if xyz is None else {"frame": "base", "xyz_m": xyz, "rotvec_rad": None, "source": "object_locator.position_base"},
                "raw": data,
            }
        ]
    return sorted(candidates, key=lambda item: float(item.get("confidence") or 0.0), reverse=True)


def select_highest_confidence_hole(candidates: list[dict[str, Any]], min_confidence: float = 0.0) -> dict[str, Any]:
    if not candidates:
        return {"ok": False, "reason": "no_empty_hole_candidates"}
    selected = sorted(candidates, key=lambda item: float(item.get("confidence") or 0.0), reverse=True)[0]
    confidence = float(selected.get("confidence") or 0.0)
    if confidence < float(min_confidence):
        return {"ok": False, "reason": "hole_confidence_below_threshold", "confidence": confidence}
    pose = selected.get("pose") or selected.get("selected_hole_pose")
    return {"ok": True, "selected_hole_pose": pose, "hole_confidence": confidence, "candidate": selected}


def build_error_policy(max_refine_retries: int, translation_threshold_m: float, rotation_threshold_rad: float) -> dict[str, Any]:
    return {
        "max_refine_retries": int(max_refine_retries),
        "translation_threshold_m": float(translation_threshold_m),
        "rotation_threshold_rad": float(rotation_threshold_rad),
    }


def determine_held_object_state(stage: str, outputs: dict[str, Any] | None = None) -> str:
    del outputs
    mapping = {
        "GRASP_TUBE_BODY": "TUBE_BODY_SELECTED_ARM",
        "HANDOVER": "TUBE_HEAD_HOLDER_ARM",
        "INSERT_DESCEND": "TUBE_INSERTED_NOT_RELEASED",
        "RELEASE": "RELEASED",
    }
    return mapping.get(str(stage), "NONE")
