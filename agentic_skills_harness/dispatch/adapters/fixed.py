from __future__ import annotations

from dataclasses import dataclass
import hashlib
import json
from pathlib import Path, PurePosixPath
import sys
from typing import Any, Callable, Mapping

from ...capability import CapabilityContract
from ...contracts.serialization import stable_dumps
from ...schema_validation import validate_json
from ..adapter import CapabilityAdapter, UnsupportedAdapter
from ..models import DispatchContext, DispatchRequest, InvocationPlan
from .presets import default_preset_catalog


ArgumentBuilder = Callable[[Mapping[str, Any], DispatchContext], tuple[list[str], tuple[str, ...], tuple[str, ...]]]


@dataclass(frozen=True)
class FixedBinding:
    capability_id: str
    adapter_id: str
    repo_relative_path: str
    entrypoint_type: str
    argument_builder: ArgumentBuilder
    fixed_arguments: tuple[str, ...] = ()
    output_parser: str = "json"

    def __post_init__(self) -> None:
        path = PurePosixPath(self.repo_relative_path)
        if path.is_absolute() or ".." in path.parts or "://" in self.repo_relative_path:
            raise ValueError("fixed binding path must be repository-relative")
        if self.entrypoint_type not in {"python", "shell"}:
            raise ValueError("fixed binding type must be python or shell")


def _mode_flags(context: DispatchContext, *, execute: bool = False) -> tuple[list[str], tuple[str, ...]]:
    args = ["--mode", context.mode.value]
    derived = ["mode"]
    if context.robot_server:
        host, separator, port = context.robot_server.rpartition(":")
        if separator and host and port.isdigit():
            args.extend(["--server-host", host, "--server-port", port])
            derived.extend(["robot_server.host", "robot_server.port"])
    if context.hardware_allowed:
        args.append("--hardware-allowed")
        derived.append("hardware_allowed")
    if execute and context.execute:
        args.append("--execute")
        derived.append("execute")
    return args, tuple(derived)


def _empty(arguments: Mapping[str, Any], context: DispatchContext) -> tuple[list[str], tuple[str, ...], tuple[str, ...]]:
    del arguments
    mode, derived = _mode_flags(context)
    return mode, (), derived


def _gripper(arguments: Mapping[str, Any], context: DispatchContext) -> tuple[list[str], tuple[str, ...], tuple[str, ...]]:
    operation = arguments.get("operation")
    side = arguments.get("side")
    if operation not in {"status", "open", "close", "initialize"} or side not in {"left", "right"}:
        raise ValueError("gripper operation and side must use the manifest enums")
    mode, derived = _mode_flags(context, execute=operation != "status")
    return [str(operation), *mode, "--side", str(side)], ("operation", "side"), derived


def _gripper_status(arguments: Mapping[str, Any], context: DispatchContext) -> tuple[list[str], tuple[str, ...], tuple[str, ...]]:
    if arguments:
        raise ValueError("gripper.observe_status accepts no request arguments")
    mode, derived = _mode_flags(context)
    return ["status", *mode, "--side", "both"], (), (*derived, "fixed.side=both", "fixed.operation=status")


def _motion(arguments: Mapping[str, Any], context: DispatchContext) -> tuple[list[str], tuple[str, ...], tuple[str, ...]]:
    side = arguments.get("side")
    if arguments.get("frame") != "base":
        raise ValueError("move_to_pose binding only supports the script's fixed base frame")
    xyz = arguments.get("xyz_m")
    rotvec = arguments.get("rotvec_rad")
    if side not in {"left", "right"} or not isinstance(xyz, list) or len(xyz) != 3:
        raise ValueError("motion binding requires side and xyz_m")
    if rotvec is None:
        rotvec = [0.0, 0.0, 0.0]
    if not isinstance(rotvec, list) or len(rotvec) != 3:
        raise ValueError("rotvec_rad must contain three values")
    pose = json.dumps([*xyz, *rotvec], ensure_ascii=False, separators=(",", ":"))
    mode, derived = _mode_flags(context, execute=True)
    flag = "--left-pose" if side == "left" else "--right-pose"
    return [*mode, flag, pose], ("side", "xyz_m", "rotvec_rad"), derived


def _handover(arguments: Mapping[str, Any], context: DispatchContext) -> tuple[list[str], tuple[str, ...], tuple[str, ...]]:
    side = arguments.get("holder_side")
    if side not in {"left", "right"}:
        raise ValueError("holder_side must be left or right")
    mode, derived = _mode_flags(context, execute=True)
    # The preset is a first-party binding; no config path is accepted from the request.
    return [*mode, "--active-arm", f"{side}_arm", "--transition-json", "procedure_skills/dual_franka_handover_transition/config/transition.json"], ("holder_side",), (*derived, "preset:handover.transition.v1")


def _capture(arguments: Mapping[str, Any], context: DispatchContext) -> tuple[list[str], tuple[str, ...], tuple[str, ...]]:
    del arguments
    mode, derived = _mode_flags(context, execute=True)
    artifact = str(Path(context.artifact_dir).resolve() / "state_capture")
    return [*mode, "--output-dir", artifact], (), (*derived, "artifact_dir")


def _reset_realsense(arguments: Mapping[str, Any], context: DispatchContext) -> tuple[list[str], tuple[str, ...], tuple[str, ...]]:
    del arguments
    mode, derived = _mode_flags(context, execute=True)
    return [*mode, "--reset-realsense"], (), derived


def _fixed_shell(arguments: Mapping[str, Any], context: DispatchContext) -> tuple[list[str], tuple[str, ...], tuple[str, ...]]:
    del arguments
    mode, derived = _mode_flags(context, execute=True)
    return mode, (), derived


class FirstPartyFixedAdapter(CapabilityAdapter):
    """A capability-specific adapter with an immutable executable and argv shape."""

    def __init__(self, repo_root: str | Path, binding: FixedBinding) -> None:
        self.repo_root = Path(repo_root).resolve()
        self.binding = binding
        self.adapter_id = binding.adapter_id

    def supports(self, capability: CapabilityContract) -> bool:
        return capability.capability_id == self.binding.capability_id

    def _executable(self) -> tuple[str, list[str]]:
        path = (self.repo_root / self.binding.repo_relative_path).resolve()
        path.relative_to(self.repo_root)
        if self.binding.entrypoint_type == "python":
            return sys.executable, [sys.executable, str(path)]
        return str(path), [str(path)]

    def build_plan(self, request: DispatchRequest, capability: CapabilityContract, context: DispatchContext) -> InvocationPlan:
        if not self.supports(capability):
            raise ValueError("fixed adapter is bound to a different capability")
        schema_errors = validate_json(dict(request.arguments), capability.input_schema_ref, self.repo_root)
        if schema_errors:
            raise ValueError(f"input schema validation failed: {schema_errors}")
        executable, argv = self._executable()
        extra, input_names, context_names = self.binding.argument_builder(request.arguments, context)
        argv.extend(self.binding.fixed_arguments)
        argv.extend(extra)
        artifact_dir = Path(context.artifact_dir).resolve()
        artifact_name = f"{capability.capability_id.replace('.', '_')}.json"
        generated = (str(artifact_dir / artifact_name),) if capability.artifact_policy and capability.artifact_policy.get("allow_output") else ()
        return InvocationPlan(
            request_id=request.request_id,
            capability_id=capability.capability_id,
            adapter_id=self.adapter_id,
            mode=context.mode.value,
            executable=executable,
            argv=tuple(argv),
            cwd=str(self.repo_root),
            timeout_s=capability.timeout_s,
            artifact_dir=str(artifact_dir),
            requires_hardware=capability.requires_hardware,
            side_effects=capability.physical_side_effects,
            gate_decision={},
            planned_only=capability.dispatch_support == "plan_only" or context.mode.value in {"mock", "dry_run"},
            capability_version=capability.capability_version,
            risk_class=capability.risk_class.value,
            artifact_policy=capability.artifact_policy or {"allow_output": True, "filename": artifact_name},
            output_parser=self.binding.output_parser,
            error_mapping={code.value: code.value for code in capability.error_codes},
            binding_id=self.binding.adapter_id,
            fixed_arguments=tuple(self.binding.fixed_arguments),
            context_derived_arguments=context_names,
            generated_artifacts=generated,
        )


def _binding(capability_id: str, path: str, kind: str, builder: ArgumentBuilder) -> FirstPartyFixedAdapter:
    raise AssertionError("use build_fixed_adapter_registry")


def build_fixed_adapter_registry(capability_registry: Any) -> dict[str, CapabilityAdapter]:
    """Build the closed registry from first-party constants and the manifest IDs."""
    bindings: dict[str, FixedBinding] = {
        "motion.move_to_pose": FixedBinding("motion.move_to_pose", "fixed.motion.move_to_pose.v1", "atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/scripts/move_to_pose.py", "python", _motion),
        "gripper.observe_status": FixedBinding("gripper.observe_status", "fixed.gripper.observe_status.v1", "atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py", "python", _gripper_status),
        "gripper.command": FixedBinding("gripper.command", "fixed.gripper.command.v1", "atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py", "python", _gripper),
        "robot.observe_health": FixedBinding("robot.observe_health", "fixed.robot.observe_health.v1", "atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/scripts/check_and_reset.py", "python", _empty, ("status",)),
        "robot.recover_reset_home": FixedBinding("robot.recover_reset_home", "fixed.robot.recover_reset_home.v1", "atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/scripts/check_and_reset.py", "python", _empty, ("ensure",)),
        "state.capture_realsense": FixedBinding("state.capture_realsense", "fixed.state.capture_realsense.v1", "atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/scripts/view_state_realsense.py", "python", _capture),
        "state.reset_realsense": FixedBinding("state.reset_realsense", "fixed.state.reset_realsense.v1", "atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/scripts/view_state_realsense.py", "python", _reset_realsense),
        "procedure.handover_transition": FixedBinding("procedure.handover_transition", "fixed.procedure.handover_transition.v1", "procedure_skills/dual_franka_handover_transition/skills/procedure-franka-handover-transition/scripts/handover_transition.py", "python", _handover),
        "recovery.robot_reset": FixedBinding("recovery.robot_reset", "fixed.recovery.robot_reset.v1", "procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_reset.sh", "shell", _fixed_shell),
        "recovery.robot_recover": FixedBinding("recovery.robot_recover", "fixed.recovery.robot_recover.v1", "procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_recover.sh", "shell", _fixed_shell),
        "motion.go_home": FixedBinding("motion.go_home", "fixed.motion.go_home.v1", "procedure_skills/robot_reset_home/skills/procedure-robot-reset-home/scripts/run_robot_go_home.sh", "shell", _fixed_shell),
    }
    result: dict[str, CapabilityAdapter] = {}
    for capability in capability_registry.list(include_internal=True, include_legacy=True):
        binding = bindings.get(capability.capability_id)
        if binding is None or capability.dispatch_support == "unsupported":
            result[capability.capability_id] = UnsupportedAdapter()
        else:
            result[capability.capability_id] = FirstPartyFixedAdapter(capability_registry.repo_root, binding)
    return result
