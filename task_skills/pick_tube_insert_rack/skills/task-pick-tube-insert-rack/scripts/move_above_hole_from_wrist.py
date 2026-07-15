#!/usr/bin/env python3
"""Move the holder TCP above a wrist-camera detected unoccupied rack hole.

Run this after the arm is already at a wrist observation pose above the rack.
The script updates the current wrist calibration, runs object-locator with the
configured wrist camera, then optionally moves the selected arm to the detected
hole x/y at a requested standoff above the detected hole plane.
"""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from typing import Any, Mapping

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
ANY_POSE_DIR = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
SKILL_PATH = SCRIPT_DIR / "tube_insertion_skill.py"

DEFAULT_LEFT_HOLE_Y_OFFSET_M = 0.0
DEFAULT_RIGHT_HOLE_Y_OFFSET_M = -0.000
DEFAULT_HOLE_CONFIG = {
    "left": ANY_POSE_DIR / "config_rack_empty_hole_left_wrist_vlm.yaml",
    "right": ANY_POSE_DIR / "config_rack_empty_hole_right_wrist_vlm.yaml",
}
sys.path.insert(0, str(ANY_POSE_DIR / "src"))
from object_locator.config import load_config  # noqa: E402
from object_locator.grounded_sam_detector import GroundedSamConfig  # noqa: E402
from sam_cache_service import CachedSamRefiner  # noqa: E402
from wrist_cached_perception import (  # noqa: E402
    CachedPerceptionError,
    locate_hole_with_vlm_sam_frame,
    resolve_cached_slot_from_frame,
)
from wrist_camera_service import WristCameraClient  # noqa: E402


def _load_skill_module():
    spec = importlib.util.spec_from_file_location("tube_insertion_skill_for_wrist_hole", SKILL_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load insertion skill from {SKILL_PATH}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


skill = _load_skill_module()


def _log(message: str) -> None:
    print(f"[move_above_hole] {message}", file=sys.stderr, flush=True)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--any-pose-dir", type=Path, default=ANY_POSE_DIR)
    parser.add_argument(
        "--artifact-dir",
        type=Path,
        default=Path("/tmp/agentic_skills_runs/legacy_move_above_hole_from_wrist"),
        help="Runtime artifacts/calibration output directory used unless --write-runtime-calibration-to-repo is set.",
    )
    parser.add_argument(
        "--write-runtime-calibration-to-repo",
        action="store_true",
        help="Explicitly allow writing live base_T_flange calibration files under --any-pose-dir.",
    )
    parser.add_argument("--side", choices=("left", "right"), default="left")
    parser.add_argument(
        "--hole-config",
        type=Path,
        default=None,
        help="object-locator config for the wrist camera. Defaults to the matching left/right unoccupied-hole config.",
    )
    parser.add_argument(
        "--wrist-perception-mode",
        choices=("cached-grid", "legacy-vlm"),
        default="legacy-vlm",
        help="Use a reserved rack-grid slot with the persistent wrist camera, or the legacy camera-opening locator.",
    )
    parser.add_argument("--rack-grid-json", type=Path, default=None)
    parser.add_argument("--target-slot-id", default=None)
    parser.add_argument("--wrist-camera-socket", type=Path, default=None)
    parser.add_argument("--wrist-frame-max-age-ms", type=float, default=500.0)
    parser.add_argument("--max-cached-slot-correction-m", type=float, default=0.015)
    parser.add_argument("--wrist-perception-report", type=Path, default=None)
    parser.add_argument(
        "--standoff-m",
        type=float,
        default=0.12,
        help="Target TCP height above the detected hole plane. Use 0.12 for observation, 0.03/0.02 for pre-insert.",
    )
    parser.add_argument(
        "--hole-plane-z-source",
        choices=("current-standoff", "detected-depth"),
        default="current-standoff",
        help=(
            "How to determine the rack/hole plane z. current-standoff uses the current TCP z minus "
            "--current-standoff-m, then intersects the wrist RGB bbox ray with that base-frame plane. "
            "detected-depth first trusts reliable wrist depth; when --rack-plane-z-base-m is also set, "
            "unreliable wrist depth falls back to intersecting the wrist bbox ray with that plane."
        ),
    )
    parser.add_argument(
        "--current-standoff-m",
        type=float,
        default=0.12,
        help=(
            "Current TCP height above the rack plane when --hole-plane-z-source=current-standoff. "
            "Set this to the observe height used before running this script."
        ),
    )
    parser.add_argument(
        "--rack-plane-z-base-m",
        type=float,
        default=None,
        help=(
            "Rack/hole plane z in base frame. With current-standoff it overrides --current-standoff-m. "
            "With detected-depth it is used only as a ray-plane fallback when wrist depth is unreliable."
        ),
    )
    parser.add_argument(
        "--hole-y-offset-m",
        type=float,
        default=None,
        help="Base-frame y offset added to the wrist-detected hole before planning the TCP target.",
    )
    parser.add_argument(
        "--min-depth-valid-fraction",
        type=float,
        default=0.15,
        help="Minimum valid depth fraction required when trusting detected depth.",
    )
    parser.add_argument(
        "--allow-fallback-depth",
        action="store_true",
        help="Allow non-corner or bbox_corners fallback depth when --hole-plane-z-source=detected-depth.",
    )
    parser.add_argument(
        "--require-reliable-depth",
        action="store_true",
        help="Also require depth quality when using current-standoff ray-plane geometry.",
    )
    parser.add_argument(
        "--roll-target-rad",
        type=float,
        default=float(np.pi),
        help="Euler xyz roll target for the locked vertical TCP posture.",
    )
    parser.add_argument(
        "--pitch-zero-rad",
        type=float,
        default=0.0,
        help="Euler xyz pitch target for the locked vertical TCP posture.",
    )
    parser.add_argument(
        "--max-roll-error-rad",
        type=float,
        default=0.035,
        help="Maximum allowed roll error after motion before release/continuation.",
    )
    parser.add_argument(
        "--max-pitch-error-rad",
        type=float,
        default=0.035,
        help="Maximum allowed pitch error after motion before release/continuation.",
    )
    parser.add_argument(
        "--max-posture-correction-iters",
        type=int,
        default=4,
        help="Maximum in-place vertical TCP posture corrections before stopping.",
    )
    parser.add_argument(
        "--split-descent-threshold-m",
        type=float,
        default=0.02,
        help=(
            "If the final target is lower than the current TCP by more than this distance, "
            "first move XY and posture at the current height, then descend."
        ),
    )
    parser.add_argument(
        "--descent-segment-m",
        type=float,
        default=0.02,
        help=(
            "Maximum z descent per P2P segment after XY/posture approach. "
            "Each segment rechecks and corrects vertical TCP posture."
        ),
    )
    parser.add_argument("--execute", action="store_true", help="Actually move the arm. Without this, only detect/report.")
    parser.add_argument(
        "--release-after-arrival",
        action="store_true",
        help="Open the selected gripper after the move reaches the requested standoff.",
    )
    parser.add_argument(
        "--after-release-sleep-sec",
        type=float,
        default=0.5,
        help="Sleep after opening the gripper before optional retract.",
    )
    parser.add_argument(
        "--retract-after-release-m",
        type=float,
        default=0.0,
        help="If positive, move TCP upward by this distance after release.",
    )
    parser.add_argument(
        "--release-on-force-n",
        type=float,
        default=None,
        help=(
            "If set, open the selected gripper during segmented descent when "
            "the selected arm force norm reaches this threshold, then apply "
            "--retract-after-release-m if positive."
        ),
    )
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--max-translation-speed", type=float, default=0.018)
    parser.add_argument("--max-rotation-speed", type=float, default=0.12)
    parser.add_argument("--max-translation-step", type=float, default=0.0006)
    parser.add_argument("--max-rotation-step", type=float, default=0.006)
    parser.add_argument("--settle-time-sec", type=float, default=0.8)
    parser.add_argument("--position-tolerance-m", type=float, default=0.006)
    parser.add_argument("--rotation-tolerance-rad", type=float, default=0.035)
    parser.add_argument("--max-correction-iters", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=6000)
    parser.add_argument("--compact", action="store_true")
    return parser


def _force_summary(observation: dict[str, Any], side: str) -> dict[str, Any]:
    arm_key = f"{side}_arm"
    force = np.asarray(observation[arm_key]["robot_state"]["wrench"]["force"], dtype=float)
    return {"force": force.tolist(), "force_norm": float(np.linalg.norm(force))}


def force_exceeds_release_threshold(force_summary: Mapping[str, Any], threshold_n: float | None) -> bool:
    if threshold_n is None:
        return False
    threshold = float(threshold_n)
    if not np.isfinite(threshold) or threshold <= 0.0:
        raise ValueError(f"--release-on-force-n must be a positive finite value, got {threshold_n!r}")
    force_norm = _finite_float(force_summary.get("force_norm"))
    return force_norm is not None and force_norm >= threshold


def _move_args(args: argparse.Namespace) -> argparse.Namespace:
    return argparse.Namespace(
        rate_hz=args.rate_hz,
        max_translation_speed=args.max_translation_speed,
        max_rotation_speed=args.max_rotation_speed,
        max_translation_step=args.max_translation_step,
        max_rotation_step=args.max_rotation_step,
        settle_time_sec=args.settle_time_sec,
        position_tolerance_m=args.position_tolerance_m,
        rotation_tolerance_rad=args.rotation_tolerance_rad,
        max_correction_iters=args.max_correction_iters,
        max_steps=args.max_steps,
    )


def _finite_float(value: Any) -> float | None:
    try:
        result = float(value)
    except (TypeError, ValueError):
        return None
    if not np.isfinite(result):
        return None
    return result


def resolve_hole_config(args: argparse.Namespace) -> Path:
    if args.hole_config is not None:
        return Path(args.hole_config)
    return DEFAULT_HOLE_CONFIG[str(args.side)]


def resolve_rack_plane_z_base_m(args: argparse.Namespace) -> float | None:
    raw = getattr(args, "rack_plane_z_base_m", None)
    value = _finite_float(raw)
    if raw is not None and value is None:
        raise ValueError(f"--rack-plane-z-base-m must be finite, got {raw!r}")
    return value


def rack_plane_z_source(args: argparse.Namespace) -> str:
    return "cli" if getattr(args, "rack_plane_z_base_m", None) is not None else "not_set"


def default_hole_y_offset_m(side: str) -> float:
    if side == "left":
        return DEFAULT_LEFT_HOLE_Y_OFFSET_M
    if side == "right":
        return DEFAULT_RIGHT_HOLE_Y_OFFSET_M
    raise ValueError(f"unsupported side: {side!r}")


def resolve_hole_y_offset_m(args: argparse.Namespace) -> float:
    raw = getattr(args, "hole_y_offset_m", None)
    value = _finite_float(raw)
    if raw is not None and value is None:
        raise ValueError(f"--hole-y-offset-m must be finite, got {raw!r}")
    if value is not None:
        return value
    return default_hole_y_offset_m(str(args.side))


def apply_hole_y_offset(hole_base_m: np.ndarray, hole_y_offset_m: float) -> np.ndarray:
    hole = np.asarray(hole_base_m, dtype=float).reshape(3).copy()
    offset = _finite_float(hole_y_offset_m)
    if offset is None:
        raise ValueError(f"hole_y_offset_m must be finite, got {hole_y_offset_m!r}")
    hole[1] += offset
    return hole


def _mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def assess_hole_depth_quality(
    result: Mapping[str, Any],
    *,
    min_valid_fraction: float,
    allow_fallback_depth: bool,
) -> dict[str, Any]:
    anchor = str(result.get("position_anchor") or "")
    position = _mapping(result.get("position"))
    valid_fraction = _finite_float(position.get("valid_fraction"))
    min_fraction = max(0.0, float(min_valid_fraction))
    corner_anchor = anchor == "bbox_corners"
    fallback = "fallback" in anchor
    reasons: list[str] = []
    if fallback and not allow_fallback_depth:
        reasons.append("fallback")
    elif not corner_anchor and not allow_fallback_depth:
        reasons.append("non_corner_anchor")
    if min_fraction > 0.0:
        if valid_fraction is None:
            reasons.append("missing_valid_fraction")
        elif valid_fraction < min_fraction:
            reasons.append("low_valid_fraction")
    return {
        "ok": len(reasons) == 0,
        "anchor": anchor,
        "corner_anchor": corner_anchor,
        "valid_fraction": valid_fraction,
        "min_valid_fraction": min_fraction,
        "allow_fallback_depth": bool(allow_fallback_depth),
        "reasons": reasons,
    }


def _bbox_center_px(result: Mapping[str, Any]) -> tuple[float, float]:
    detection = _mapping(result.get("detection"))
    bbox = _mapping(detection.get("bbox"))
    x_min = _finite_float(bbox.get("x_min"))
    y_min = _finite_float(bbox.get("y_min"))
    x_max = _finite_float(bbox.get("x_max"))
    y_max = _finite_float(bbox.get("y_max"))
    if None not in (x_min, y_min, x_max, y_max):
        return ((x_min + x_max) * 0.5, (y_min + y_max) * 0.5)

    position = _mapping(result.get("position"))
    u_px = _finite_float(position.get("u_px"))
    v_px = _finite_float(position.get("v_px"))
    if u_px is None or v_px is None:
        raise ValueError("hole result has neither a valid detection bbox nor position u/v pixels")
    return (u_px, v_px)


def ray_plane_hole_base_from_detection(
    result: Mapping[str, Any],
    *,
    base_T_camera: np.ndarray,
    plane_z_base_m: float,
) -> np.ndarray:
    intrinsics = _mapping(result.get("intrinsics"))
    fx = _finite_float(intrinsics.get("fx"))
    fy = _finite_float(intrinsics.get("fy"))
    ppx = _finite_float(intrinsics.get("ppx"))
    ppy = _finite_float(intrinsics.get("ppy"))
    if None in (fx, fy, ppx, ppy) or abs(fx) <= 1e-12 or abs(fy) <= 1e-12:
        raise ValueError("hole result is missing valid camera intrinsics")

    u_px, v_px = _bbox_center_px(result)
    ray_camera = np.array([(u_px - ppx) / fx, (v_px - ppy) / fy, 1.0], dtype=float)
    transform = np.asarray(base_T_camera, dtype=float).reshape(4, 4)
    origin_base = transform[:3, 3]
    ray_base = transform[:3, :3] @ ray_camera
    if abs(float(ray_base[2])) <= 1e-12:
        raise ValueError("wrist camera ray is parallel to the requested base z plane")
    scale = (float(plane_z_base_m) - float(origin_base[2])) / float(ray_base[2])
    if not np.isfinite(scale) or scale <= 0.0:
        raise ValueError(f"wrist camera ray does not intersect the plane in front of the camera, scale={scale:g}")
    return origin_base + ray_base * scale


def _detected_position_base(result: Mapping[str, Any]) -> np.ndarray | None:
    try:
        return skill._require_position_base(result, "hole")
    except Exception:
        return None


def _detected_position_camera(result: Mapping[str, Any]) -> np.ndarray | None:
    position = _mapping(result.get("position"))
    values = [_finite_float(position.get(key)) for key in ("x_m", "y_m", "z_m")]
    if any(value is None for value in values):
        return None
    point = np.asarray(values, dtype=float)
    if not np.all(np.isfinite(point)):
        return None
    return point


_BBOX_CORNER_KEYS = ("bbox_x1_y1", "bbox_x2_y1", "bbox_x1_y2", "bbox_x2_y2")
_POINT_BASE_VARIANT_KEYS = ("base", "flange_T_camera", "baseright", "baseleft")


def _point_xyz_from_mapping(value: Mapping[str, Any]) -> np.ndarray | None:
    direct = np.array(
        [
            _finite_float(value.get("x_m")),
            _finite_float(value.get("y_m")),
            _finite_float(value.get("z_m")),
        ],
        dtype=object,
    )
    if None not in direct:
        return np.asarray(direct, dtype=float)
    return None


def _point_base_xyz(point: Any) -> np.ndarray | None:
    if not isinstance(point, Mapping):
        return None
    for key in _POINT_BASE_VARIANT_KEYS:
        nested = point.get(key)
        if isinstance(nested, Mapping):
            xyz = _point_xyz_from_mapping(nested)
            if xyz is not None:
                return xyz
    for key, nested in point.items():
        if key == "source":
            continue
        if isinstance(nested, Mapping):
            xyz = _point_xyz_from_mapping(nested)
            if xyz is not None:
                return xyz
    return _point_xyz_from_mapping(point)


def bbox_corner_center_base_from_detection(result: Mapping[str, Any]) -> tuple[np.ndarray | None, dict[str, Any]]:
    points_base = _mapping(result.get("points_base"))
    points = {
        key: _point_base_xyz(points_base.get(key))
        for key in _BBOX_CORNER_KEYS
    }
    available = {key: value.tolist() for key, value in points.items() if value is not None}
    report: dict[str, Any] = {
        "available_corner_keys": list(available),
        "available_corners_base_m": available,
    }
    if len(available) == len(_BBOX_CORNER_KEYS):
        center = np.mean([points[key] for key in _BBOX_CORNER_KEYS], axis=0)
        report.update({"ok": True, "source": "bbox_4_corner_points", "used_corner_keys": list(_BBOX_CORNER_KEYS)})
        return center, report

    report.update(
        {
            "ok": False,
            "source": "bbox_corner_points_unavailable",
            "reason": "need all four bbox corners",
        }
    )
    return None, report


def resolve_hole_base_for_target(
    result: Mapping[str, Any],
    *,
    current_pose: np.ndarray,
    base_T_camera: np.ndarray,
    hole_plane_z_source: str,
    current_standoff_m: float,
    min_depth_valid_fraction: float,
    allow_fallback_depth: bool,
    require_reliable_depth: bool,
    rack_plane_z_base_m: float | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    quality = assess_hole_depth_quality(
        result,
        min_valid_fraction=min_depth_valid_fraction,
        allow_fallback_depth=allow_fallback_depth,
    )
    detected_base = _detected_position_base(result)
    detected_camera = _detected_position_camera(result)
    corner_base, corner_report = bbox_corner_center_base_from_detection(result)
    report: dict[str, Any] = {
        "source": hole_plane_z_source,
        "depth_quality": quality,
        "detected_position_base_m": None if detected_base is None else detected_base.tolist(),
        "detected_position_camera_m": None if detected_camera is None else detected_camera.tolist(),
        "corner_center": corner_report,
    }
    plane_z_override = _finite_float(rack_plane_z_base_m)
    if rack_plane_z_base_m is not None and plane_z_override is None:
        raise ValueError(f"--rack-plane-z-base-m must be finite, got {rack_plane_z_base_m!r}")

    if corner_base is not None:
        pose = np.asarray(current_pose, dtype=float).reshape(6)
        report.update(
            {
                "source": "bbox-4-corner-average",
                "plane_z_base_m": float(corner_base[2]),
                "current_tcp_z_m": float(pose[2]),
                "current_standoff_m": float(pose[2] - corner_base[2]),
                "ray_plane_used": False,
                "fallback_used": False,
                "corner_average_used": True,
                "ignored_object_locator_base_points": bool(detected_base is not None),
            }
        )
        return corner_base, report

    if hole_plane_z_source == "detected-depth":
        depth_failure_reasons = list(quality["reasons"])
        if detected_camera is None:
            depth_failure_reasons.append("missing_camera_xyz")
        if not depth_failure_reasons:
            transform = np.asarray(base_T_camera, dtype=float).reshape(4, 4)
            dynamic_base = (transform @ np.append(detected_camera, 1.0))[:3]
            if np.all(np.isfinite(dynamic_base)):
                report.update(
                    {
                        "source": "detected-depth-dynamic-camera-transform",
                        "plane_z_base_m": float(dynamic_base[2]),
                        "dynamic_position_base_m": dynamic_base.tolist(),
                        "fallback_used": False,
                        "ignored_object_locator_base_points": bool(
                            detected_base is not None or corner_base is not None
                        ),
                    }
                )
                return dynamic_base, report
            depth_failure_reasons.append("non_finite_dynamic_base")

        if plane_z_override is None:
            raise ValueError(f"unreliable detected hole depth: {', '.join(depth_failure_reasons)}")

        pose = np.asarray(current_pose, dtype=float).reshape(6)
        hole_base = ray_plane_hole_base_from_detection(
            result,
            base_T_camera=base_T_camera,
            plane_z_base_m=plane_z_override,
        )
        report.update(
            {
                "source": "head-rack-plane-fallback-dynamic-ray",
                "fallback_used": True,
                "fallback_from": "detected-depth",
                "fallback_reasons": depth_failure_reasons,
                "plane_z_base_m": plane_z_override,
                "current_tcp_z_m": float(pose[2]),
                "current_standoff_m": float(pose[2] - plane_z_override),
                "ray_plane_used": True,
                "ignored_object_locator_base_points": bool(
                    detected_base is not None or corner_base is not None
                ),
            }
        )
        return hole_base, report

    if plane_z_override is not None:
        if require_reliable_depth and not quality["ok"]:
            raise ValueError(f"unreliable hole depth: {', '.join(quality['reasons'])}")
        pose = np.asarray(current_pose, dtype=float).reshape(6)
        hole_base = ray_plane_hole_base_from_detection(
            result,
            base_T_camera=base_T_camera,
            plane_z_base_m=plane_z_override,
        )
        report.update(
            {
                "source": "rack-plane-z-dynamic-ray",
                "plane_z_base_m": plane_z_override,
                "current_tcp_z_m": float(pose[2]),
                "current_standoff_m": float(pose[2] - plane_z_override),
                "ray_plane_used": True,
                "ignored_object_locator_base_points": bool(
                    detected_base is not None or corner_base is not None
                ),
            }
        )
        return hole_base, report

    if hole_plane_z_source != "current-standoff":
        raise ValueError(f"unsupported hole_plane_z_source: {hole_plane_z_source!r}")
    if require_reliable_depth and not quality["ok"]:
        raise ValueError(f"unreliable hole depth: {', '.join(quality['reasons'])}")
    standoff = _finite_float(current_standoff_m)
    if standoff is None or standoff < 0.0:
        raise ValueError(f"--current-standoff-m must be a finite non-negative value, got {current_standoff_m!r}")
    pose = np.asarray(current_pose, dtype=float).reshape(6)
    plane_z = float(pose[2] - standoff)
    hole_base = ray_plane_hole_base_from_detection(
        result,
        base_T_camera=base_T_camera,
        plane_z_base_m=plane_z,
    )
    report.update(
        {
            "source": "current-standoff-dynamic-ray",
            "plane_z_base_m": plane_z,
            "current_tcp_z_m": float(pose[2]),
            "current_standoff_m": float(standoff),
            "ray_plane_used": True,
            "ignored_object_locator_base_points": bool(
                detected_base is not None or corner_base is not None
            ),
        }
    )
    return hole_base, report


def compute_wrist_hole_target_pose(
    *,
    current_pose: np.ndarray,
    hole_base_m: np.ndarray,
    standoff_m: float,
    roll_target_rad: float,
    pitch_rad: float,
) -> np.ndarray:
    target = np.asarray(current_pose, dtype=float).reshape(6).copy()
    hole = np.asarray(hole_base_m, dtype=float).reshape(3)
    target[0] = float(hole[0])
    target[1] = float(hole[1])
    target[2] = float(hole[2] + standoff_m)
    target[3:] = skill.tcp_pitch_zero_rotvec(
        target,
        pitch_rad=float(pitch_rad),
        roll_rad=float(roll_target_rad),
    )
    return target


def compute_vertical_posture_correction_pose(
    *,
    current_pose: np.ndarray,
    vertical_rotvec: np.ndarray,
) -> np.ndarray:
    correction = np.asarray(current_pose, dtype=float).reshape(6).copy()
    correction[3:] = np.asarray(vertical_rotvec, dtype=float).reshape(3)
    return correction


def compute_xy_posture_approach_pose(
    *,
    current_pose: np.ndarray,
    final_target_pose: np.ndarray,
    vertical_rotvec: np.ndarray,
) -> np.ndarray:
    approach = np.asarray(final_target_pose, dtype=float).reshape(6).copy()
    current = np.asarray(current_pose, dtype=float).reshape(6)
    approach[2] = float(current[2])
    approach[3:] = np.asarray(vertical_rotvec, dtype=float).reshape(3)
    return approach


def compute_segmented_descent_poses(
    *,
    current_pose: np.ndarray,
    final_target_pose: np.ndarray,
    vertical_rotvec: np.ndarray,
    segment_m: float,
) -> list[np.ndarray]:
    current = np.asarray(current_pose, dtype=float).reshape(6)
    final = np.asarray(final_target_pose, dtype=float).reshape(6)
    segment = float(segment_m)
    if not np.isfinite(segment) or segment <= 0.0:
        raise ValueError(f"segment_m must be a positive finite value, got {segment_m!r}")

    descent = float(current[2] - final[2])
    target = final.copy()
    target[3:] = np.asarray(vertical_rotvec, dtype=float).reshape(3)
    if descent <= 1e-12:
        return [target]

    steps = max(1, int(np.ceil(descent / segment)))
    waypoints: list[np.ndarray] = []
    for index in range(1, steps + 1):
        waypoint = target.copy()
        travelled = min(descent, segment * index)
        waypoint[2] = float(current[2] - travelled)
        if index == steps:
            waypoint[2] = float(final[2])
        waypoints.append(waypoint)
    return waypoints


def compute_retract_after_release_pose(
    *,
    current_pose: np.ndarray,
    retract_distance_m: float,
) -> np.ndarray:
    distance = float(retract_distance_m)
    if not np.isfinite(distance) or distance < 0.0:
        raise ValueError(f"retract_distance_m must be finite and non-negative, got {retract_distance_m!r}")
    pose = np.asarray(current_pose, dtype=float).reshape(6).copy()
    pose[2] += distance
    return pose


def check_vertical_roll_pitch(
    pose: np.ndarray,
    *,
    roll_target_rad: float,
    pitch_rad: float,
    max_roll_error_rad: float,
    max_pitch_error_rad: float,
) -> dict[str, Any]:
    pose6 = np.asarray(pose, dtype=float).reshape(6)
    rpy = skill.R.from_rotvec(pose6[3:]).as_euler("xyz")
    roll_target = skill.nearest_equivalent_angle(float(roll_target_rad), float(rpy[0]))
    pitch_target = float(pitch_rad)
    roll_error = float(rpy[0] - roll_target)
    pitch_error = float(rpy[1] - pitch_target)
    max_roll_error = abs(float(max_roll_error_rad))
    max_pitch_error = abs(float(max_pitch_error_rad))
    ok = abs(roll_error) <= max_roll_error and abs(pitch_error) <= max_pitch_error
    return {
        "ok": bool(ok),
        "mode": "roll_pitch_vertical_gate",
        "euler_xyz_rad": rpy.tolist(),
        "euler_xyz_deg": (rpy * 180.0 / np.pi).tolist(),
        "roll_target_rad": float(roll_target_rad),
        "pitch_target_rad": pitch_target,
        "roll_error_rad": roll_error,
        "pitch_error_rad": pitch_error,
        "roll_error_deg": roll_error * 180.0 / np.pi,
        "pitch_error_deg": pitch_error * 180.0 / np.pi,
        "max_roll_error_rad": max_roll_error,
        "max_pitch_error_rad": max_pitch_error,
    }


def run_vertical_posture_correction(
    *,
    client: Any,
    args: argparse.Namespace,
    side: str,
    vertical_rotvec: np.ndarray,
    stage_prefix: str,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    corrections: list[dict[str, Any]] = []
    motion_ok = True
    _observation, poses = skill.read_ee_poses(client)
    posture_check = check_vertical_roll_pitch(
        poses[side],
        roll_target_rad=args.roll_target_rad,
        pitch_rad=args.pitch_zero_rad,
        max_roll_error_rad=args.max_roll_error_rad,
        max_pitch_error_rad=args.max_pitch_error_rad,
    )
    initial_check = posture_check
    max_posture_corrections = max(0, int(args.max_posture_correction_iters))
    for correction_index in range(max_posture_corrections):
        if posture_check["ok"]:
            break
        _log(
            "vertical TCP posture check failed; correcting in place "
            f"{correction_index + 1}/{max_posture_corrections} "
            f"(roll={posture_check['roll_error_deg']:.2f}deg, "
            f"pitch={posture_check['pitch_error_deg']:.2f}deg)"
        )
        correction_target = compute_vertical_posture_correction_pose(
            current_pose=poses[side],
            vertical_rotvec=vertical_rotvec,
        )
        correction_started = time.monotonic()
        correction_stage = skill.move_dual_absolute(
            client=client,
            side=side,
            side_target=correction_target,
            all_current=poses,
            args=_move_args(args),
            stage_name=f"{stage_prefix}_{correction_index + 1}",
            execute=True,
        )
        _observation, poses = skill.read_ee_poses(client)
        translation_check = p2p_stage_arrival_check(
            correction_stage,
            side,
            ignore_z=False,
            require_rotation=False,
        )
        posture_check = check_vertical_roll_pitch(
            poses[side],
            roll_target_rad=args.roll_target_rad,
            pitch_rad=args.pitch_zero_rad,
            max_roll_error_rad=args.max_roll_error_rad,
            max_pitch_error_rad=args.max_pitch_error_rad,
        )
        corrections.append(
            {
                "correction_index": correction_index + 1,
                "target_pose": correction_target.tolist(),
                "stage": correction_stage,
                "translation_check": translation_check,
                "posture_check": posture_check,
                "elapsed_sec": time.monotonic() - correction_started,
            }
        )
        # The low-level P2P result uses a full rotation-vector tolerance.  A
        # correction may therefore report ok=false even though its translation
        # is safe and the measured roll/pitch has improved.  Let the dedicated
        # vertical posture gate decide whether another correction is needed.
        # Only a missing/invalid result or excessive translation aborts early.
        if not translation_check["ok"]:
            motion_ok = False
            break
    return {
        "motion_ok": motion_ok,
        "initial_check": initial_check,
        "corrections": corrections,
        "final_check": posture_check,
    }, poses


def release_and_optional_retract(
    *,
    client: Any,
    args: argparse.Namespace,
    side: str,
    vertical_rotvec: np.ndarray,
    reason: str,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    report: dict[str, Any] = {"reason": reason}
    arm_key = f"{side}_arm"
    _log(f"opening {arm_key} gripper ({reason})")
    report["release_gripper"] = client.open_gripper(arm_key)
    if args.after_release_sleep_sec > 0.0:
        time.sleep(args.after_release_sleep_sec)

    release_observation, retract_poses = skill.read_ee_poses(client)
    if args.retract_after_release_m > 0.0:
        _log(f"retracting upward by {args.retract_after_release_m:.4f}m ({reason})")
        retract_target = compute_retract_after_release_pose(
            current_pose=retract_poses[side],
            retract_distance_m=args.retract_after_release_m,
        )
        retract_stage = skill.move_dual_absolute(
            client=client,
            side=side,
            side_target=retract_target,
            all_current=retract_poses,
            args=_move_args(args),
            stage_name=f"retract_{side}_after_release",
            execute=True,
        )
        report["retract_stage"] = retract_stage
        release_observation, retract_poses = skill.read_ee_poses(client)
        report["retract_ok"] = skill.p2p_stage_succeeded(retract_stage)
    report["force_after_release"] = _force_summary(release_observation, side)
    return report, retract_poses


def run_segmented_vertical_descent(
    *,
    client: Any,
    args: argparse.Namespace,
    side: str,
    current_poses: Mapping[str, np.ndarray],
    final_target_pose: np.ndarray,
    vertical_rotvec: np.ndarray,
) -> tuple[dict[str, Any], dict[str, np.ndarray]]:
    poses = {name: np.asarray(pose, dtype=float).copy() for name, pose in current_poses.items()}
    waypoints = compute_segmented_descent_poses(
        current_pose=poses[side],
        final_target_pose=final_target_pose,
        vertical_rotvec=vertical_rotvec,
        segment_m=args.descent_segment_m,
    )
    report: dict[str, Any] = {
        "ok": True,
        "segment_m": float(args.descent_segment_m),
        "segments": len(waypoints),
        "stages": [],
    }
    for index, waypoint in enumerate(waypoints, start=1):
        _log(
            f"segmented descent {index}/{len(waypoints)} "
            f"target_z={waypoint[2]:.4f}m"
        )
        segment_started = time.monotonic()
        stage = skill.move_dual_absolute(
            client=client,
            side=side,
            side_target=waypoint,
            all_current=poses,
            args=_move_args(args),
            stage_name=f"move_{side}_segmented_descent_{index}",
            execute=True,
        )
        arrival_check = p2p_stage_arrival_check(
            stage,
            side,
            ignore_z=True,
            require_rotation=False,
        )
        segment_report: dict[str, Any] = {
            "index": index,
            "target_pose": waypoint.tolist(),
            "stage": stage,
            "arrival_check": arrival_check,
            "move_elapsed_sec": time.monotonic() - segment_started,
        }
        segment_observation, poses = skill.read_ee_poses(client)
        force_summary = _force_summary(segment_observation, side)
        segment_report["force_after_move"] = force_summary
        if force_exceeds_release_threshold(force_summary, args.release_on_force_n):
            release_report, poses = release_and_optional_retract(
                client=client,
                args=args,
                side=side,
                vertical_rotvec=vertical_rotvec,
                reason="release_on_force",
            )
            segment_report["force_release"] = release_report
            report["stages"].append(segment_report)
            report.update(
                {
                    "ok": True,
                    "completed_reason": "released_on_force",
                    "released_during_descent": True,
                    "force_threshold_n": float(args.release_on_force_n),
                }
            )
            _log(
                "force threshold reached during segmented descent; released gripper "
                f"(force_norm={force_summary['force_norm']:.3f}N)"
            )
            return report, poses
        if not arrival_check["ok"]:
            report.update({"ok": False, "stopped_reason": "segmented_descent_move_failed"})
            report["stages"].append(segment_report)
            _log("segmented descent move failed; stopping")
            return report, poses

        posture_report, poses = run_vertical_posture_correction(
            client=client,
            args=args,
            side=side,
            vertical_rotvec=vertical_rotvec,
            stage_prefix=f"correct_{side}_vertical_tcp_posture_descent_{index}",
        )
        segment_report["posture"] = posture_report
        report["stages"].append(segment_report)
        if not posture_report["motion_ok"] or not posture_report["final_check"]["ok"]:
            report.update({"ok": False, "stopped_reason": "segmented_descent_posture_failed"})
            _log("segmented descent posture correction failed; stopping")
            return report, poses

    return report, poses


def p2p_stage_arrival_check(
    stage: Mapping[str, Any],
    side: str,
    *,
    ignore_z: bool,
    require_rotation: bool,
) -> dict[str, Any]:
    """Validate usable P2P feedback with independently selectable gates.

    The insertion flow has a dedicated measured roll/pitch posture gate.  Its
    approach and descent moves therefore check translation here, then defer
    posture acceptance to ``run_vertical_posture_correction``.
    """
    mode = "xy" if ignore_z else "xyz"
    if require_rotation:
        mode += "_translation_rotation"
    else:
        mode += "_translation_posture_deferred"
    if not bool(stage.get("execute")):
        return {"ok": True, "mode": mode, "reason": "not_executed"}

    result = _mapping(stage.get("result"))
    if bool(result.get("ok")):
        return {"ok": True, "mode": mode, "reason": "p2p_full_xyz_ok"}

    final_error = _mapping(result.get("final_error"))
    tolerances = _mapping(result.get("tolerances"))
    error_delta_raw = final_error.get(f"{side}_error_delta")
    if not isinstance(error_delta_raw, list | tuple) or len(error_delta_raw) < 6:
        return {"ok": False, "mode": mode, "reason": "missing_final_error_delta"}

    error_delta = np.asarray(error_delta_raw, dtype=float).reshape(-1)
    if error_delta.size < 6 or not np.all(np.isfinite(error_delta[:6])):
        return {"ok": False, "mode": mode, "reason": "invalid_final_error_delta"}

    position_tolerance = _finite_float(tolerances.get("position_tolerance_m"))
    rotation_tolerance = _finite_float(tolerances.get("rotation_tolerance_rad"))
    if position_tolerance is None:
        position_tolerance = 0.0
    if rotation_tolerance is None:
        rotation_tolerance = 0.0

    rotation_error = _finite_float(final_error.get(f"{side}_rotation_error_rad"))
    if rotation_error is None:
        rotation_error = _finite_float(final_error.get("max_rotation_error_rad"))
    if rotation_error is None:
        rotation_error = float(np.linalg.norm(error_delta[3:6]))

    xy_error = float(np.linalg.norm(error_delta[:2]))
    z_error = float(abs(error_delta[2]))
    translation_error = xy_error if ignore_z else float(np.linalg.norm(error_delta[:3]))
    translation_ok = translation_error <= float(position_tolerance)
    rotation_ok = float(rotation_error) <= float(rotation_tolerance)
    ok = translation_ok and (rotation_ok or not require_rotation)
    return {
        "ok": bool(ok),
        "mode": mode,
        "translation_error_m": translation_error,
        "xy_error_m": xy_error,
        "z_error_m": z_error,
        "rotation_error_rad": float(rotation_error),
        "translation_ok": bool(translation_ok),
        "rotation_ok": bool(rotation_ok),
        "rotation_gate_deferred": not require_rotation,
        "position_tolerance_m": float(position_tolerance),
        "rotation_tolerance_rad": float(rotation_tolerance),
        "ignored_axes": ["z"] if ignore_z else [],
    }


def p2p_stage_succeeded_ignoring_z(stage: Mapping[str, Any], side: str) -> dict[str, Any]:
    """Compatibility wrapper for the original XY-plus-rotation arrival gate."""
    return p2p_stage_arrival_check(
        stage,
        side,
        ignore_z=True,
        require_rotation=True,
    )


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    flow_started = time.monotonic()
    if (args.release_after_arrival or args.retract_after_release_m > 0.0) and not args.execute:
        raise ValueError("--release-after-arrival/--retract-after-release-m require --execute")
    rack_plane_z_base_m = resolve_rack_plane_z_base_m(args)
    rack_plane_z_base_source = rack_plane_z_source(args)
    hole_y_offset_m = resolve_hole_y_offset_m(args)
    hole_config = resolve_hole_config(args)
    runtime_calibration_dir = args.any_pose_dir if args.write_runtime_calibration_to_repo else args.artifact_dir / "object_locator_runtime"
    if not args.execute:
        report = {
            "execute": False,
            "planned_only": True,
            "requires_execute_for_rpc": True,
            "requires_execute_for_camera": True,
            "requires_execute_for_motion": True,
            "requires_execute_for_gripper": bool(args.release_after_arrival or args.release_on_force_n is not None),
            "abnormal_robot_state_detected": False,
            "side": args.side,
            "standoff_m": float(args.standoff_m),
            "hole_plane_z_source": args.hole_plane_z_source,
            "current_standoff_m": float(args.current_standoff_m),
            "rack_plane_z_base_m": None if rack_plane_z_base_m is None else float(rack_plane_z_base_m),
            "rack_plane_z_base_source": rack_plane_z_base_source,
            "hole_y_offset_m": float(hole_y_offset_m),
            "roll_target_rad": float(args.roll_target_rad),
            "pitch_zero_rad": float(args.pitch_zero_rad),
            "hole_config": str(hole_config),
            "runtime_calibration_dir": str(runtime_calibration_dir),
            "final_error": {
                "translation_m": None,
                "rotation_rad": None,
                "within_threshold": None,
                "source": "planned_only_no_motion",
            },
            "completion_flag": False,
            "physical_verified": False,
            "stopped_reason": "planned_only_requires_execute_for_camera_rpc_and_motion",
        }
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0

    client = skill.DualFrankaRobotiqRpcClient(
        ip=args.server_host,
        port=args.server_port,
        timeout=args.rpc_timeout_sec,
    )
    try:
        report: dict[str, Any] = {
            "execute": bool(args.execute),
            "side": args.side,
            "standoff_m": float(args.standoff_m),
            "hole_plane_z_source": args.hole_plane_z_source,
            "current_standoff_m": float(args.current_standoff_m),
            "rack_plane_z_base_m": None if rack_plane_z_base_m is None else float(rack_plane_z_base_m),
            "rack_plane_z_base_source": rack_plane_z_base_source,
            "hole_y_offset_m": float(hole_y_offset_m),
            "roll_target_rad": float(args.roll_target_rad),
            "pitch_zero_rad": float(args.pitch_zero_rad),
            "split_descent_threshold_m": float(args.split_descent_threshold_m),
            "descent_segment_m": float(args.descent_segment_m),
            "server": f"{args.server_host}:{args.server_port}",
            "hole_config": str(hole_config),
            "timings_sec": {},
        }
        report["ping"] = client.ping()
        _log(f"connected to {report['server']}")

        observation, poses = skill.read_ee_poses(client)
        runtime_calibration_dir.mkdir(parents=True, exist_ok=True)
        skill.write_base_T_flange_files(runtime_calibration_dir, observation)
        report["runtime_calibration_dir"] = str(runtime_calibration_dir)
        report["initial_pose"] = poses[args.side].tolist()
        report["initial_force"] = _force_summary(observation, args.side)
        _log(
            f"initial {args.side} pose xyz={np.array2string(poses[args.side][:3], precision=4)} "
            f"force_norm={report['initial_force']['force_norm']:.3f}N"
        )

        base_T_camera = skill.pose6_to_matrix(poses[args.side]) @ skill.load_flange_T_camera(args.any_pose_dir, args.side)
        _log(f"running wrist hole detection mode={args.wrist_perception_mode} config={hole_config}")
        perception_started = time.monotonic()
        hole_result: dict[str, Any]
        cached_perception_report: dict[str, Any] | None = None
        try:
            if args.wrist_perception_mode == "cached-grid":
                if args.rack_grid_json is None or args.target_slot_id is None:
                    raise ValueError("cached-grid requires --rack-grid-json and --target-slot-id")
                if rack_plane_z_base_m is None:
                    raise ValueError("cached-grid requires --rack-plane-z-base-m")
                grid = json.loads(args.rack_grid_json.read_text(encoding="utf-8"))
                slot = next((item for item in grid.get("slots", []) if item.get("id") == args.target_slot_id), None)
                if slot is None:
                    raise ValueError(f"target slot not found: {args.target_slot_id}")
                if slot.get("state") != "reserved":
                    raise ValueError(f"target slot {args.target_slot_id} is not reserved: {slot.get('state')}")
                frame_started = time.monotonic()
                frame = WristCameraClient(args.wrist_camera_socket).frame(
                    args.side,
                    max_age_ms=args.wrist_frame_max_age_ms,
                )
                report["timings_sec"]["camera_capture"] = time.monotonic() - frame_started
                report["wrist_camera_frame"] = {
                    "generation": frame.get("generation"),
                    "age_ms": frame.get("age_ms"),
                    "reset_count": frame.get("reset_count"),
                    "camera_timestamp_ms": frame.get("camera_timestamp_ms"),
                }
                config = load_config(hole_config)
                source = config.grounded_sam
                sam_detector = CachedSamRefiner(GroundedSamConfig(
                    grounding_model=source.grounding_model,
                    sam_model=source.sam_model,
                    text_prompt=source.text_prompt,
                    selection="score",
                    box_threshold=source.box_threshold,
                    text_threshold=source.text_threshold,
                    device=source.device,
                    use_sam=source.use_sam,
                    refine_bbox_with_mask=source.refine_bbox_with_mask,
                    min_box_area_px=source.min_box_area_px,
                    max_box_area_ratio=source.max_box_area_ratio,
                    min_mask_area_px=source.min_mask_area_px,
                    cap_endpoint_rule=source.cap_endpoint_rule,
                    cap_dark_threshold=source.cap_dark_threshold,
                    cap_min_area_px=source.cap_min_area_px,
                ))
                hole_base, cached_perception_report = resolve_cached_slot_from_frame(
                    image_bgr=frame["color_bgr"],
                    depth_m=frame["depth_m"],
                    intrinsics=frame["intrinsics"],
                    base_T_camera=base_T_camera,
                    cached_position_base_m=np.asarray(slot["position_base_m"], dtype=float),
                    rack_plane_z_base_m=rack_plane_z_base_m,
                    hole_config=config,
                    sam_detector=sam_detector,
                    max_correction_m=args.max_cached_slot_correction_m,
                )
                hole_plane_report = {
                    "source": "cached-grid-wrist-local-correction",
                    "plane_z_base_m": rack_plane_z_base_m,
                    "depth_quality": {"ok": True, "source": "local-gate"},
                    "fallback_used": bool(cached_perception_report.get("fallback_used")),
                }
                hole_result = {
                    "run_id": f"cached-{args.target_slot_id}-{time.time_ns()}",
                    "position_anchor": "cached_slot_local_correction",
                    "detection": cached_perception_report,
                    "position": {"source": cached_perception_report.get("source")},
                }
                report["cached_slot_id"] = args.target_slot_id
                report["wrist_perception"] = cached_perception_report
                if args.wrist_perception_report is not None:
                    args.wrist_perception_report.parent.mkdir(parents=True, exist_ok=True)
                    args.wrist_perception_report.write_text(json.dumps(cached_perception_report, ensure_ascii=False, indent=2), encoding="utf-8")
            else:
                if args.wrist_camera_socket is not None:
                    frame_started = time.monotonic()
                    frame = WristCameraClient(args.wrist_camera_socket, timeout_sec=10.0).frame(
                        args.side,
                        max_age_ms=args.wrist_frame_max_age_ms,
                    )
                    report["timings_sec"]["camera_frame_request"] = time.monotonic() - frame_started
                    config = load_config(hole_config)
                    source = config.grounded_sam
                    sam_detector = CachedSamRefiner(GroundedSamConfig(
                        grounding_model=source.grounding_model,
                        sam_model=source.sam_model,
                        text_prompt=source.text_prompt,
                        selection="score",
                        box_threshold=source.box_threshold,
                        text_threshold=source.text_threshold,
                        device=source.device,
                        use_sam=source.use_sam,
                        refine_bbox_with_mask=source.refine_bbox_with_mask,
                        min_box_area_px=source.min_box_area_px,
                        max_box_area_ratio=source.max_box_area_ratio,
                        min_mask_area_px=source.min_mask_area_px,
                        cap_endpoint_rule=source.cap_endpoint_rule,
                        cap_dark_threshold=source.cap_dark_threshold,
                        cap_min_area_px=source.cap_min_area_px,
                    ))
                    hole_result, supplied_frame_timings = locate_hole_with_vlm_sam_frame(
                        image_bgr=frame["color_bgr"],
                        depth_m=frame["depth_m"],
                        intrinsics=frame["intrinsics"],
                        config=config,
                        sam_detector=sam_detector,
                        camera_timestamp_ms=frame["camera_timestamp_ms"],
                    )
                    report["timings_sec"]["legacy_supplied_frame"] = supplied_frame_timings
                    report["wrist_camera_frame"] = {
                        "generation": frame.get("generation"),
                        "age_ms": frame.get("age_ms"),
                        "reset_count": frame.get("reset_count"),
                    }
                else:
                    hole_result = skill.run_object_locator(args.any_pose_dir, hole_config)
                    report["timings_sec"]["legacy_object_locator"] = hole_result.get("task_timings_sec", {})
                hole_base, hole_plane_report = resolve_hole_base_for_target(
                    hole_result,
                    current_pose=poses[args.side],
                    base_T_camera=base_T_camera,
                    hole_plane_z_source=args.hole_plane_z_source,
                    current_standoff_m=args.current_standoff_m,
                    rack_plane_z_base_m=rack_plane_z_base_m,
                    min_depth_valid_fraction=args.min_depth_valid_fraction,
                    allow_fallback_depth=args.allow_fallback_depth,
                    require_reliable_depth=args.require_reliable_depth,
                )
        except (ValueError, RuntimeError) as exc:
            if isinstance(exc, CachedPerceptionError):
                cached_perception_report = exc.report
                report["wrist_perception"] = cached_perception_report
                if args.wrist_perception_report is not None:
                    args.wrist_perception_report.parent.mkdir(parents=True, exist_ok=True)
                    args.wrist_perception_report.write_text(json.dumps(cached_perception_report, ensure_ascii=False, indent=2), encoding="utf-8")
            report["hole"] = {
                "run_id": locals().get("hole_result", {}).get("run_id"),
                "position_anchor": locals().get("hole_result", {}).get("position_anchor"),
                "detected_position_base_m": None,
                "detection": locals().get("hole_result", {}).get("detection"),
                "position_camera": locals().get("hole_result", {}).get("position"),
            }
            report["stopped_reason"] = "unreliable_hole_geometry"
            report["error"] = str(exc)
            _log(f"hole geometry rejected: {exc}")
            print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
            return 2
        report["timings_sec"]["wrist_perception_total"] = time.monotonic() - perception_started
        hole_base_raw = hole_base.copy()
        hole_base = apply_hole_y_offset(hole_base_raw, hole_y_offset_m)
        report["hole"] = {
            "run_id": hole_result.get("run_id"),
            "position_anchor": hole_result.get("position_anchor"),
            "position_base_raw_m": hole_base_raw.tolist(),
            "position_base_m": hole_base.tolist(),
            "hole_y_offset_m": float(hole_y_offset_m),
            "hole_plane": hole_plane_report,
            "detection": hole_result.get("detection"),
            "position_camera": hole_result.get("position"),
            "panel": hole_result.get("debug_outputs", {}).get("panel_history"),
        }
        _log(
            f"hole run_id={report['hole']['run_id']} "
            f"anchor={report['hole']['position_anchor']} "
            f"base={np.array2string(hole_base, precision=4)} "
            f"z_source={hole_plane_report['source']} "
            f"depth_ok={hole_plane_report['depth_quality']['ok']}"
        )

        target = compute_wrist_hole_target_pose(
            current_pose=poses[args.side],
            hole_base_m=hole_base,
            standoff_m=args.standoff_m,
            roll_target_rad=args.roll_target_rad,
            pitch_rad=args.pitch_zero_rad,
        )
        report["target_pose"] = target.tolist()
        report["target_posture_check"] = check_vertical_roll_pitch(
            target,
            roll_target_rad=args.roll_target_rad,
            pitch_rad=args.pitch_zero_rad,
            max_roll_error_rad=args.max_roll_error_rad,
            max_pitch_error_rad=args.max_pitch_error_rad,
        )
        _log(
            f"target xyz={np.array2string(target[:3], precision=4)} "
            f"standoff={args.standoff_m:.4f}m execute={args.execute}"
        )

        if args.execute:
            released_during_descent = False
            descent_m = float(poses[args.side][2] - target[2])
            if descent_m > max(0.0, float(args.split_descent_threshold_m)):
                approach_target = compute_xy_posture_approach_pose(
                    current_pose=poses[args.side],
                    final_target_pose=target,
                    vertical_rotvec=target[3:],
                )
                report["xy_posture_approach_target_pose"] = approach_target.tolist()
                _log(
                    "large descent requested; moving XY/posture at current height first "
                    f"(descent={descent_m:.4f}m)"
                )
                approach_started = time.monotonic()
                approach_stage = skill.move_dual_absolute(
                    client=client,
                    side=args.side,
                    side_target=approach_target,
                    all_current=poses,
                    args=_move_args(args),
                    stage_name=f"move_{args.side}_xy_posture_approach",
                    execute=True,
                )
                report["xy_posture_approach_stage"] = approach_stage
                report["timings_sec"]["xy_posture_approach"] = time.monotonic() - approach_started
                approach_arrival_check = p2p_stage_arrival_check(
                    approach_stage,
                    args.side,
                    ignore_z=True,
                    require_rotation=False,
                )
                report["xy_posture_approach_arrival_check"] = approach_arrival_check
                if not approach_arrival_check["ok"]:
                    report["stopped_reason"] = "xy_posture_approach_failed"
                    _log("XY/posture approach failed; not descending")
                    print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                    return 3

                approach_posture_report, poses = run_vertical_posture_correction(
                    client=client,
                    args=args,
                    side=args.side,
                    vertical_rotvec=target[3:],
                    stage_prefix=f"correct_{args.side}_vertical_tcp_posture_before_descent",
                )
                report["xy_posture_approach_posture"] = approach_posture_report
                report["timings_sec"]["pre_descent_posture_corrections"] = sum(
                    float(item.get("elapsed_sec", 0.0))
                    for item in approach_posture_report["corrections"]
                )
                if (
                    not approach_posture_report["motion_ok"]
                    or not approach_posture_report["final_check"]["ok"]
                ):
                    report["stopped_reason"] = "vertical_tcp_posture_before_descent_failed"
                    _log("vertical TCP posture check failed before descent; not descending")
                    print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                    return 3
            else:
                report["xy_posture_approach_stage"] = {
                    "skipped": True,
                    "reason": "descent below split threshold",
                    "descent_m": descent_m,
                }

            if descent_m > max(0.0, float(args.split_descent_threshold_m)):
                _log("sending segmented P2P descent to wrist-detected hole standoff")
                segmented_report, poses = run_segmented_vertical_descent(
                    client=client,
                    args=args,
                    side=args.side,
                    current_poses=poses,
                    final_target_pose=target,
                    vertical_rotvec=target[3:],
                )
                report["segmented_descent"] = segmented_report
                report["timings_sec"]["segmented_descent_moves"] = sum(
                    float(item.get("move_elapsed_sec", 0.0))
                    for item in segmented_report.get("stages", [])
                )
                report["timings_sec"]["segmented_descent_posture_corrections"] = sum(
                    float(correction.get("elapsed_sec", 0.0))
                    for item in segmented_report.get("stages", [])
                    for correction in _mapping(item.get("posture")).get("corrections", [])
                )
                if not segmented_report["ok"]:
                    report["stopped_reason"] = segmented_report.get("stopped_reason", "segmented_descent_failed")
                    _log("segmented descent failed; not releasing gripper")
                    print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                    return 3
                released_during_descent = bool(segmented_report.get("released_during_descent"))

                last_segment = segmented_report["stages"][-1] if segmented_report["stages"] else {}
                report["move_stage"] = last_segment.get("stage")
                report["move_arrival_check"] = last_segment.get("arrival_check")
                last_posture = _mapping(last_segment.get("posture"))
                report["posture_check_after_move"] = last_posture.get("initial_check")
                report["posture_corrections"] = last_posture.get("corrections", [])
                report["posture_check_after_corrections"] = last_posture.get("final_check")
            else:
                _log("sending P2P move to wrist-detected hole standoff")
                move_started = time.monotonic()
                stage = skill.move_dual_absolute(
                    client=client,
                    side=args.side,
                    side_target=target,
                    all_current=poses,
                    args=_move_args(args),
                    stage_name=f"move_{args.side}_above_wrist_detected_hole",
                    execute=True,
                )
                report["move_stage"] = stage
                report["timings_sec"]["direct_standoff_move"] = time.monotonic() - move_started
                arrival_check = p2p_stage_arrival_check(
                    stage,
                    args.side,
                    ignore_z=True,
                    require_rotation=False,
                )
                report["move_arrival_check"] = arrival_check
                if not arrival_check["ok"]:
                    report["stopped_reason"] = "move_to_standoff_failed"
                    _log("move failed XY arrival/feedback check; not releasing gripper")
                    print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                    return 3
                _log(
                    "move reached XY tolerance; deferring posture to roll/pitch gate "
                    f"(z ignored: {arrival_check.get('z_error_m', 0.0):.4f}m, "
                    f"rotation={arrival_check.get('rotation_error_rad', 0.0):.4f}rad)"
                )

                posture_report, poses = run_vertical_posture_correction(
                    client=client,
                    args=args,
                    side=args.side,
                    vertical_rotvec=target[3:],
                    stage_prefix=f"correct_{args.side}_vertical_tcp_posture_after_descent",
                )
                report["posture_check_after_move"] = posture_report["initial_check"]
                report["posture_corrections"] = posture_report["corrections"]
                posture_check = posture_report["final_check"]
                report["posture_check_after_corrections"] = posture_check
                report["timings_sec"]["post_move_posture_corrections"] = sum(
                    float(item.get("elapsed_sec", 0.0))
                    for item in posture_report["corrections"]
                )
                if not posture_report["motion_ok"] or not posture_check["ok"]:
                    report["stopped_reason"] = "vertical_tcp_posture_failed"
                    _log(
                        "vertical TCP posture check failed after correction; not releasing gripper "
                        f"(roll={posture_check['roll_error_deg']:.2f}deg, "
                        f"pitch={posture_check['pitch_error_deg']:.2f}deg)"
                    )
                    print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                    return 3

            if args.release_after_arrival and not released_during_descent:
                release_report, _release_poses = release_and_optional_retract(
                    client=client,
                    args=args,
                    side=args.side,
                    vertical_rotvec=target[3:],
                    reason="arrival",
                )
                report["release_after_arrival"] = release_report
                retract_stage = release_report.get("retract_stage")
                if retract_stage is not None and not release_report.get("retract_ok", False):
                    report["stopped_reason"] = "retract_after_release_failed"
                    _log("retract failed")
                    print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
                    return 3

        final_observation, final_poses = skill.read_ee_poses(client)
        final_pose = final_poses[args.side]
        report["final_pose"] = final_pose.tolist()
        report["final_force"] = _force_summary(final_observation, args.side)
        report["tcp_height_above_hole_m"] = float(final_pose[2] - hole_base[2])
        report["tcp_xy_error_to_hole_m"] = float(np.linalg.norm(final_pose[:2] - hole_base[:2]))
        report["timings_sec"]["total"] = time.monotonic() - flow_started
        _log(
            f"done height={report['tcp_height_above_hole_m']:.4f}m "
            f"xy_error={report['tcp_xy_error_to_hole_m']:.4f}m "
            f"force_norm={report['final_force']['force_norm']:.3f}N"
        )

        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    raise SystemExit(main())
