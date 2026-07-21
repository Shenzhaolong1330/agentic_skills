from __future__ import annotations

from pathlib import Path
import sys


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPT_DIR))
from grasp_right_arm_xyz import close_gripper_and_confirm  # noqa: E402


def _observation(position: float, *, enabled: bool = True) -> dict:
    return {
        "right_arm": {
            "gripper": {
                "position": position,
                "velocity": 0.0,
                "enabled": enabled,
                "open_position": 0.0,
                "closed_position": 0.7929,
                "last_command_position": 0.7929,
                "last_command_source": "{'close': True}",
            }
        }
    }


class FakeClient:
    def __init__(self, positions: list[float]) -> None:
        self._positions = iter(positions)
        self._last_position = positions[-1]
        self.close_calls = 0

    def get_observation(self) -> dict:
        self._last_position = next(self._positions, self._last_position)
        return _observation(self._last_position)

    def close_gripper(self, side: str) -> dict:
        self.close_calls += 1
        return {"accepted": True, "side": side, "attempt": self.close_calls}


def test_close_waits_for_physical_position_feedback() -> None:
    client = FakeClient([0.0, 0.0, 0.12, 0.40])

    result, confirmation = close_gripper_and_confirm(
        client,
        "right_arm",
        timeout_sec=0.1,
        poll_sec=0.001,
        min_closed_fraction=0.35,
        retries=1,
    )

    assert result["accepted"] is True
    assert confirmation["ok"] is True
    assert confirmation["attempts"] == 1
    assert confirmation["initial_closed_fraction"] == 0.0
    assert confirmation["final_closed_fraction"] > 0.35
    assert client.close_calls == 1


def test_close_retries_then_reports_unconfirmed_feedback() -> None:
    client = FakeClient([0.0, 0.0, 0.0])

    _, confirmation = close_gripper_and_confirm(
        client,
        "right_arm",
        timeout_sec=0.0,
        poll_sec=0.001,
        min_closed_fraction=0.35,
        retries=1,
    )

    assert confirmation["ok"] is False
    assert confirmation["attempts"] == 2
    assert confirmation["final_closed_fraction"] == 0.0
    assert client.close_calls == 2
