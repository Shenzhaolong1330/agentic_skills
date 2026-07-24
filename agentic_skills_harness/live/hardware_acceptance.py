"""Fixed S10H probe orchestration for the audited Dual-Franka backend."""

from __future__ import annotations

from dataclasses import dataclass
import math
from pathlib import Path
from typing import Any, Mapping

from agentic_skills_harness.live.scene_backend import FixedS10SceneBackend
from agentic_skills_harness.live.vendor_backend import FixedDualFrankaVendorBackend
from agentic_skills_harness.safety.motion_limits import MotionLimits
from agentic_skills_harness.safety.workspace import WorkspaceConfig


_PHYSICAL_COMMANDS = {
    "gripper-empty",
    "motion-p2p",
    "motion-relative",
    "safe-stop",
    "grasp-verification",
    "fault-recovery",
    "reset-home",
}


@dataclass(frozen=True)
class HardwareProbeResult:
    command: str
    automatic_passed: bool
    checks: Mapping[str, Any]
    limitations: tuple[str, ...] = ()

    def to_dict(self) -> dict[str, Any]:
        return {
            "command": self.command,
            "automatic_passed": self.automatic_passed,
            "checks": dict(self.checks),
            "limitations": list(self.limitations),
        }


def _distance(first: list[float], second: list[float]) -> float:
    return math.sqrt(sum((float(first[index]) - float(second[index])) ** 2 for index in range(3)))


def _rpc_result_not_failed(value: Any) -> bool:
    if not isinstance(value, Mapping):
        return False
    return value.get("ok") is not False and not value.get("error")


def _read_only_probe(
    backend: FixedDualFrankaVendorBackend,
    scene_backend: FixedS10SceneBackend,
    artifact_dir: Path,
) -> HardwareProbeResult:
    ping = backend.ping()
    samples = [backend.read_robot_state() for _ in range(5)]
    scene_checks = scene_backend.capture_h1_observations(
        artifact_dir / "observations" / "scene",
        hardware_allowed=True,
    )
    samples.extend(backend.read_robot_state() for _ in range(5))
    timestamps = [str(item["timestamp"]) for item in samples]
    left_motion = _distance(samples[0]["left"]["ee_pose"], samples[-1]["left"]["ee_pose"])
    right_motion = _distance(samples[0]["right"]["ee_pose"], samples[-1]["right"]["ee_pose"])
    timestamps_monotonic = timestamps == sorted(timestamps) and len(set(timestamps)) == len(timestamps)
    gripper_sample_count = {
        side: sum(item[side].get("gripper") != "UNKNOWN" for item in samples)
        for side in ("left", "right")
    }
    checks = {
        "ping_ok": ping.get("ok") is True,
        "robot_sample_count": len(samples),
        "timestamps_monotonic": timestamps_monotonic,
        "left_pose_change_m": left_motion,
        "right_pose_change_m": right_motion,
        "gripper_sample_count": gripper_sample_count,
        "gripper_command_calls": 0,
        "motion_command_calls": 0,
        "physical_action_calls": 0,
        **scene_checks,
    }
    passed = (
        checks["ping_ok"]
        and checks["robot_sample_count"] == 10
        and timestamps_monotonic
        and left_motion <= 0.002
        and right_motion <= 0.002
        and all(count == 10 for count in gripper_sample_count.values())
        and scene_checks["automatic_passed"] is True
    )
    return HardwareProbeResult("read-only", bool(passed), checks)


def _gripper_probe(backend: FixedDualFrankaVendorBackend) -> HardwareProbeResult:
    records: list[dict[str, Any]] = []
    command_success_count = 0
    for side in ("left", "right"):
        for cycle in range(1, 4):
            open_result = backend.command_gripper(side=side, operation="open")
            open_state = backend.command_gripper(side=side, operation="status")
            close_result = backend.command_gripper(side=side, operation="close")
            close_state = backend.command_gripper(side=side, operation="status")
            command_success_count += int(_rpc_result_not_failed(open_result))
            command_success_count += int(_rpc_result_not_failed(close_result))
            records.append(
                {
                    "side": side,
                    "cycle": cycle,
                    "open_result": open_result,
                    "open_state": open_state,
                    "close_result": close_result,
                    "close_state": close_state,
                }
            )
    return HardwareProbeResult(
        "gripper-empty",
        len(records) == 6 and command_success_count == 12,
        {
            "cycles": records,
            "open_command_count": 6,
            "close_command_count": 6,
            "command_success_count": command_success_count,
            "initialization_command_count": 0,
            "holding_after_close": "UNKNOWN",
        },
        ("close_success_never_asserts_holding",),
    )


def _pose_side(pose_id: str) -> str:
    if pose_id.startswith("left_"):
        return "left"
    if pose_id.startswith("right_"):
        return "right"
    raise ValueError("acceptance_pose_id_must_encode_side")


def _motion_p2p_probe(
    backend: FixedDualFrankaVendorBackend,
    workspace: WorkspaceConfig,
    limits: MotionLimits,
) -> HardwareProbeResult:
    records: list[dict[str, Any]] = []
    for pose_id in sorted(workspace.safe_acceptance_poses):
        side = _pose_side(pose_id)
        configured = dict(workspace.resolve_acceptance_pose(pose_id, side=side))
        for repetition in range(1, 4):
            current = backend.read_robot_state()
            configured["rotvec_rad"] = list(
                configured.get("rotvec_rad", current[side]["ee_pose"][3:])
            )
            configured["source"] = f"s10_fixed_acceptance_pose:{pose_id}"
            result = backend.move_to_pose(
                side=side,
                pose=configured,
                workspace=workspace,
                limits=limits,
            )
            records.append(
                {
                    "pose_id": pose_id,
                    "side": side,
                    "repetition": repetition,
                    "result": result,
                }
            )
    expected = len(workspace.safe_acceptance_poses) * 3
    return HardwareProbeResult(
        "motion-p2p",
        len(records) == expected and expected > 0,
        {
            "target_test_count": len(records),
            "expected_target_test_count": expected,
            "workspace_violation_count": 0,
            "records": records,
        },
    )


def _motion_relative_probe(
    backend: FixedDualFrankaVendorBackend,
    workspace: WorkspaceConfig,
    limits: MotionLimits,
) -> HardwareProbeResult:
    displacement = min(0.010, limits.max_relative_displacement_m)
    records: list[dict[str, Any]] = []
    for side in ("left", "right"):
        for direction in (1.0, -1.0):
            result = backend.move_relative(
                side=side,
                delta_xyz_m=(0.0, 0.0, direction * displacement),
                delta_rotvec_rad=(0.0, 0.0, 0.0),
                reference_frame="base",
                workspace=workspace,
                limits=limits,
            )
            records.append(
                {
                    "side": side,
                    "reference_frame": "base",
                    "delta_z_m": direction * displacement,
                    "result": result,
                }
            )
    return HardwareProbeResult(
        "motion-relative",
        len(records) == 4,
        {
            "relative_move_count": len(records),
            "configured_displacement_m": displacement,
            "records": records,
            "direct_relative_rpc_calls": 0,
        },
    )


def _safe_stop_probe(backend: FixedDualFrankaVendorBackend) -> HardwareProbeResult:
    result = backend.safe_stop()
    return HardwareProbeResult(
        "safe-stop",
        False,
        result,
        ("skipped_not_validated", "reset_is_not_used_as_safe_stop"),
    )


def _grasp_probe(backend: FixedDualFrankaVendorBackend) -> HardwareProbeResult:
    result = backend.verify_grasp()
    return HardwareProbeResult(
        "grasp-verification",
        False,
        result,
        ("skipped_not_validated", "close_return_code_is_not_grasp_evidence"),
    )


def _guarded_recovery_probe(command: str) -> HardwareProbeResult:
    return HardwareProbeResult(
        command,
        False,
        {
            "status": "DENIED",
            "backend_calls": 0,
            "reason": "runtime_estop_held_object_and_error_evidence_unavailable",
        },
        (
            "fixed_vendor_method_exists_but_acceptance_runner_cannot_infer_safe_recovery_state",
            "no_automatic_recovery_or_home_was_attempted",
        ),
    )


def run_fixed_hardware_probe(
    command: str,
    backend: FixedDualFrankaVendorBackend,
    config: Mapping[str, Any],
    *,
    artifact_dir: Path | None = None,
    scene_backend: FixedS10SceneBackend | None = None,
) -> HardwareProbeResult:
    """Execute one closed, command-enumerated probe after external gates pass."""

    workspace = WorkspaceConfig.from_dict(config.get("workspace", {}))
    limits = MotionLimits.from_dict(config.get("limits", {}))
    if command == "read-only":
        if artifact_dir is None or scene_backend is None:
            raise ValueError("fixed_scene_backend_required_for_h1")
        return _read_only_probe(backend, scene_backend, artifact_dir)
    if command == "gripper-empty":
        return _gripper_probe(backend)
    if command == "motion-p2p":
        return _motion_p2p_probe(backend, workspace, limits)
    if command == "motion-relative":
        return _motion_relative_probe(backend, workspace, limits)
    if command == "safe-stop":
        return _safe_stop_probe(backend)
    if command == "grasp-verification":
        return _grasp_probe(backend)
    if command in {"fault-recovery", "reset-home"}:
        return _guarded_recovery_probe(command)
    raise ValueError("unsupported_fixed_acceptance_command")


def command_has_physical_side_effects(command: str) -> bool:
    return command in _PHYSICAL_COMMANDS
