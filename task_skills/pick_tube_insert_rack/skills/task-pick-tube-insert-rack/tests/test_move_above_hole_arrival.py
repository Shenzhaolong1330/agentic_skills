from __future__ import annotations

from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
from move_above_hole_from_wrist import p2p_stage_arrival_check  # noqa: E402


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
