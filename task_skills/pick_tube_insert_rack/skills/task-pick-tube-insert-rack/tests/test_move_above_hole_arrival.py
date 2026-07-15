from __future__ import annotations

from pathlib import Path
import sys

import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
from move_above_hole_from_wrist import p2p_stage_arrival_check, resolve_hole_base_for_target  # noqa: E402


def _failed_only_on_rotation_stage() -> dict:
    return {
        "execute": True,
        "result": {
            "ok": False,
            "final_error": {
                "right_error_delta": [0.0008, -0.0005, 0.0038, 0.02, 0.03, 0.015],
                "right_rotation_error_rad": 0.0394,
            },
            "tolerances": {
                "position_tolerance_m": 0.006,
                "rotation_tolerance_rad": 0.035,
            },
        },
    }


def test_descent_defers_rotation_to_vertical_posture_gate() -> None:
    check = p2p_stage_arrival_check(
        _failed_only_on_rotation_stage(),
        "right",
        ignore_z=True,
        require_rotation=False,
    )

    assert check["ok"] is True
    assert check["translation_ok"] is True
    assert check["rotation_ok"] is False
    assert check["rotation_gate_deferred"] is True


def test_strict_compatibility_gate_still_rejects_rotation_error() -> None:
    check = p2p_stage_arrival_check(
        _failed_only_on_rotation_stage(),
        "right",
        ignore_z=True,
        require_rotation=True,
    )

    assert check["ok"] is False
    assert check["rotation_ok"] is False


def test_posture_correction_rejects_translation_drift() -> None:
    stage = _failed_only_on_rotation_stage()
    stage["result"]["final_error"]["right_error_delta"] = [0.001, 0.001, 0.007, 0.0, 0.0, 0.04]

    check = p2p_stage_arrival_check(
        stage,
        "right",
        ignore_z=False,
        require_rotation=False,
    )

    assert check["ok"] is False
    assert check["translation_ok"] is False
    assert check["rotation_gate_deferred"] is True


def test_missing_final_feedback_is_not_accepted() -> None:
    check = p2p_stage_arrival_check(
        {"execute": True, "result": {"ok": False}},
        "right",
        ignore_z=True,
        require_rotation=False,
    )

    assert check["ok"] is False
    assert check["reason"] == "missing_final_error_delta"


def _hole_result_with_corners(points_base: dict) -> dict:
    return {
        "position_anchor": "bbox_corners",
        "position": {"valid_fraction": 1.0},
        "intrinsics": {"fx": 600.0, "fy": 600.0, "ppx": 320.0, "ppy": 240.0},
        "detection": {"bbox": {"x_min": 360.0, "y_min": 220.0, "x_max": 400.0, "y_max": 260.0}},
        "points_base": points_base,
    }


def _point(x: float, y: float, z: float) -> dict:
    return {"base": {"x_m": x, "y_m": y, "z_m": z}}


def test_hole_base_prefers_four_corner_average() -> None:
    result = _hole_result_with_corners({
        "bbox_x1_y1": _point(0.10, 0.20, 0.90),
        "bbox_x2_y1": _point(0.30, 0.20, 0.90),
        "bbox_x1_y2": _point(0.10, 0.40, 1.10),
        "bbox_x2_y2": _point(0.30, 0.40, 1.10),
    })

    hole_base, report = resolve_hole_base_for_target(
        result,
        current_pose=np.array([0.0, 0.0, 1.2, 0.0, 0.0, 0.0]),
        base_T_camera=np.eye(4),
        hole_plane_z_source="current-standoff",
        current_standoff_m=0.2,
        min_depth_valid_fraction=0.0,
        allow_fallback_depth=True,
        require_reliable_depth=False,
        rack_plane_z_base_m=1.0,
    )

    assert np.allclose(hole_base, [0.20, 0.30, 1.00])
    assert report["source"] == "bbox-4-corner-average"
    assert report["ray_plane_used"] is False
    assert report["corner_average_used"] is True


def test_hole_base_falls_back_to_ray_plane_when_any_corner_missing() -> None:
    result = _hole_result_with_corners({
        "bbox_x1_y1": _point(9.0, 9.0, 9.0),
        "bbox_x2_y2": _point(11.0, 11.0, 11.0),
    })

    hole_base, report = resolve_hole_base_for_target(
        result,
        current_pose=np.array([0.0, 0.0, 1.2, 0.0, 0.0, 0.0]),
        base_T_camera=np.eye(4),
        hole_plane_z_source="current-standoff",
        current_standoff_m=0.2,
        min_depth_valid_fraction=0.0,
        allow_fallback_depth=True,
        require_reliable_depth=False,
        rack_plane_z_base_m=1.0,
    )

    assert np.allclose(hole_base, [0.1, 0.0, 1.0])
    assert report["source"] == "rack-plane-z-dynamic-ray"
    assert report["ray_plane_used"] is True
    assert report["corner_center"]["ok"] is False
