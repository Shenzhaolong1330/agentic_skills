#!/usr/bin/env python3
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from agentic_skills_harness.command_runner import CommandPlan, CommandResult, CommandRunner
from agentic_skills_harness.manifest import find_entrypoint, load_manifest
from agentic_skills_harness.pose import build_error_6dof
from agentic_skills_harness.reset_recovery import ResetRecoveryController
from agentic_skills_harness.robot_health import RobotHealthMonitor
from agentic_skills_harness.trace import TraceWriter, new_run_id
from agentic_skills_harness.types import (
    HeldObjectState,
    ResetOutcome,
    ResetRecoveryResult,
    RobotHealthState,
    SkillContext,
    SkillMode,
    StageResult,
    TaskResult,
    TaskState,
)
from task_pick_tube_insert_rack_logic import (
    build_error_policy,
    build_pregrasp_and_grasp_plan,
    extract_tube_geometry,
    select_arm_for_tail_side,
    select_highest_confidence_hole,
    wrap_hole_candidates,
)

TASK_ROOT = Path(__file__).resolve().parents[1]
MOCK_DIR = TASK_ROOT / "mock"
OBJECT_LOCATOR_ROOT = REPO_ROOT / "atomic_skills/object_locator"
OBJECT_LOCATOR = OBJECT_LOCATOR_ROOT / ".venv/bin/object-locator"
TUBE_CONFIG = OBJECT_LOCATOR_ROOT / "config_test_tube_cleanup_leftmost_vlm_head.yaml"
GRASP_SCRIPT = TASK_ROOT / "scripts/grasp_right_arm_xyz.sh"
INSERT_FLOW = TASK_ROOT / "scripts/run_manual_grip_to_insert.py"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="pick_tube_insert_rack task harness runner")
    parser.add_argument("--mode", choices=[item.value for item in SkillMode], default=SkillMode.MOCK.value)
    parser.add_argument("--artifact-dir", type=Path, default=None)
    parser.add_argument("--output-json", type=Path, default=None)
    parser.add_argument("--tube-result-json", type=Path, default=None)
    parser.add_argument("--rack-result-json", type=Path, default=None)
    parser.add_argument("--hole-result-json", type=Path, default=None)
    parser.add_argument("--robot-health-json", type=Path, default=None)
    parser.add_argument("--reset-recovery-json", type=Path, default=None)
    parser.add_argument("--mock-robot-health", choices=("ready", "abnormal", "fault", "unreachable", "estop"), default="ready")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--hardware-allowed", action="store_true", help="Allow access to real hardware in live mode.")
    parser.add_argument("--auto-reset-on-abnormal", action="store_true", default=True)
    parser.add_argument("--disable-auto-reset", action="store_false", dest="auto_reset_on_abnormal")
    parser.add_argument("--max-auto-reset-attempts", type=int, default=1)
    parser.add_argument("--resume-after-held-object-reset", action="store_true")
    parser.add_argument("--max-refine-retries", type=int, default=3)
    parser.add_argument("--translation-threshold-m", type=float, default=0.005)
    parser.add_argument("--rotation-threshold-rad", type=float, default=0.0872664626)
    parser.add_argument("--min-tube-confidence", type=float, default=0.0)
    parser.add_argument("--min-hole-confidence", type=float, default=0.0)
    parser.add_argument("--robot-server", default=None)
    parser.add_argument(
        "--allow-telemetry-only-health",
        action="store_true",
        help=(
            "Allow live preflight to continue when both arms have valid telemetry "
            "but the RPC server does not expose native fault diagnostics."
        ),
    )
    parser.add_argument("--manifest", type=Path, default=REPO_ROOT / "skill_manifest.json")
    return parser


def _fixture(path: Path | None, default_name: str) -> Path:
    return path if path is not None else MOCK_DIR / default_name


def _stage(trace: TraceWriter, stages: list[StageResult], state: TaskState, outputs=None, warnings=None, ok=True, status="ok") -> StageResult:
    stage = StageResult(stage=state, ok=ok, status=status, outputs=outputs or {}, warnings=warnings or []).close()
    stages.append(stage)
    trace.write_stage(len(stages), state.value, stage)
    return stage


def _plan(trace: TraceWriter, state: TaskState, argv: list[str], *, recovery: bool = False, reason: str = "planned by task harness") -> None:
    plan = CommandPlan(argv=argv, entrypoint_ref=state.value, requires_hardware=True, would_execute=False, recovery=recovery, reason=reason)
    trace.append_command_plan(plan)


def _last_json_object(text: str) -> dict[str, object] | None:
    try:
        value = json.loads(text)
        return value if isinstance(value, dict) else None
    except json.JSONDecodeError:
        pass
    decoder = json.JSONDecoder()
    result = None
    for index, char in enumerate(text):
        if char != "{":
            continue
        try:
            value, end = decoder.raw_decode(text[index:])
        except json.JSONDecodeError:
            continue
        if isinstance(value, dict) and not text[index + end :].strip():
            result = value
    return result


def _run_gated_command(
    runner: CommandRunner,
    trace: TraceWriter,
    context: SkillContext,
    command: list[str],
    *,
    entrypoint_ref: str,
    entrypoint: dict[str, object],
    artifact_prefix: str,
    recovery: bool = False,
) -> CommandResult:
    result = runner.run(
        command,
        context=context,
        entrypoint_ref=entrypoint_ref,
        entrypoint=entrypoint,
        recovery=recovery,
    )
    trace.append_command_plan(result.plan)
    trace.path(f"{artifact_prefix}_stdout.log").write_text(result.stdout, encoding="utf-8")
    trace.path(f"{artifact_prefix}_stderr.log").write_text(result.stderr, encoding="utf-8")
    return result


def _abort(
    trace: TraceWriter,
    output_json: Path,
    context: SkillContext,
    stages: list[StageResult],
    outputs: dict[str, object],
    reset_results: list[ResetRecoveryResult],
    reason: str,
    *,
    errors: list[str] | None = None,
    warnings: list[str] | None = None,
) -> TaskResult:
    outputs.update(
        {
            "task_state": TaskState.ABORT.value,
            "held_object_state": context.held_object_state.value,
            "reset_recovery": [item.to_dict() for item in reset_results],
            "completion_flag": False,
            "physical_verified": False,
        }
    )
    _stage(trace, stages, TaskState.ABORT, {"stopped_reason": reason}, ok=False, status="aborted")
    result = TaskResult(
        False,
        TaskState.ABORT,
        False,
        False,
        context,
        stages,
        outputs,
        reset_results,
        reason,
        errors=errors or [],
        warnings=warnings or [],
    )
    trace.write_task_result(result, output_json)
    return result


def _live_reset(
    *,
    trace: TraceWriter,
    stages: list[StageResult],
    context: SkillContext,
    command_runner: CommandRunner,
    reset_controller: ResetRecoveryController,
    health_monitor: RobotHealthMonitor,
    before_health,
    reason: str,
) -> ResetRecoveryResult:
    attempt_index = context.reset_attempt_count + 1
    held_risk = context.held_object_state not in {HeldObjectState.NONE, HeldObjectState.RELEASED}
    if context.reset_attempt_count >= context.max_auto_reset_attempts:
        return ResetRecoveryResult(
            ok=False,
            outcome=ResetOutcome.RESET_EXCEEDED_MAX_ATTEMPTS,
            attempted=False,
            executed=False,
            planned_only=False,
            before_health=before_health,
            attempt_index=attempt_index,
            max_attempts=context.max_auto_reset_attempts,
            held_object_state=context.held_object_state,
            held_object_risk=held_risk,
            aborted_after_reset=True,
            errors=[f"max auto reset attempts exceeded while handling {reason}"],
        )
    context.reset_attempt_count += 1
    entrypoint = find_entrypoint(
        command_runner.manifest,
        "atomic-state-dual-franka-reset",
        "check_and_reset_ensure",
    )
    command = reset_controller.build_reset_command(context)
    command_result = _run_gated_command(
        command_runner,
        trace,
        context,
        command,
        entrypoint_ref="AUTO_RESET_RECOVERY",
        entrypoint=entrypoint,
        artifact_prefix=f"reset_recovery_{attempt_index}_command",
        recovery=True,
    )
    raw = command_result.raw_json or _last_json_object(command_result.stdout)
    after_health = None
    if isinstance(raw, dict) and isinstance(raw.get("status_after"), dict):
        after_health = health_monitor.classify_health(raw["status_after"], source="after_live_reset")
    reset_ok = bool(
        command_result.executed
        and command_result.returncode == 0
        and after_health is not None
        and after_health.ok
    )
    reset = ResetRecoveryResult(
        ok=reset_ok,
        outcome=ResetOutcome.RESET_OK if reset_ok else ResetOutcome.RESET_FAILED,
        attempted=True,
        executed=bool(command_result.executed),
        planned_only=bool(command_result.planned_only),
        before_health=before_health,
        after_health=after_health,
        command=command,
        gate_decision=command_result.gate_decision,
        attempt_index=attempt_index,
        max_attempts=context.max_auto_reset_attempts,
        held_object_state=context.held_object_state,
        held_object_risk=held_risk,
        resumed_after_reset=reset_ok and not held_risk,
        aborted_after_reset=held_risk or not reset_ok,
        errors=[] if reset_ok else [f"live reset failed with returncode={command_result.returncode}"],
        warnings=["reset occurred while holding a tube; object state must be reverified"] if held_risk else [],
    )
    trace.write_reset(attempt_index, reset)
    _stage(trace, stages, TaskState.AUTO_RESET_RECOVERY, {"reset_recovery": reset.to_dict()}, ok=reset.ok)
    verified = after_health or health_monitor.classify_health({}, source="after_live_reset_missing")
    trace.write_health("robot_health_after_reset.json", verified)
    _stage(trace, stages, TaskState.RESET_RECOVERY_VERIFY, {"robot_health": verified.to_dict()}, ok=verified.ok)
    return reset


def _run_live(
    args: argparse.Namespace,
    *,
    trace: TraceWriter,
    output_json: Path,
    context: SkillContext,
    manifest: dict[str, object],
    stages: list[StageResult],
    outputs: dict[str, object],
    warnings: list[str],
) -> TaskResult:
    reset_results: list[ResetRecoveryResult] = []
    command_runner = CommandRunner(manifest)
    health_monitor = RobotHealthMonitor(manifest, repo_root=REPO_ROOT)
    reset_controller = ResetRecoveryController(manifest, repo_root=REPO_ROOT)
    locate_entrypoint = find_entrypoint(manifest, "task-pick-tube-insert-rack", "task_live_locate_tube")
    grasp_entrypoint = find_entrypoint(manifest, "task-pick-tube-insert-rack", "task_live_grasp_handover")
    insertion_entrypoint = find_entrypoint(manifest, "task-pick-tube-insert-rack", "task_live_insert_flow")
    health_entrypoint = find_entrypoint(manifest, "atomic-state-dual-franka-reset", "check_and_reset_status")
    task_entrypoint = find_entrypoint(manifest, "task-pick-tube-insert-rack", "task_runner")

    task_decision = command_runner.gate.evaluate(context, task_entrypoint)
    trace.append_command_plan(
        CommandPlan(
            argv=["hardware_preflight", "task"],
            entrypoint_ref="LIVE_TASK_PREFLIGHT",
            requires_hardware=True,
            opens_camera=True,
            connects_robot_rpc=True,
            moves_robot=True,
            controls_gripper=True,
            would_execute=bool(context.execute),
            reason=task_decision.reason,
            gate_decision=task_decision.to_dict(),
        )
    )
    if not task_decision.allowed:
        return _abort(
            trace,
            output_json,
            context,
            stages,
            outputs,
            reset_results,
            "hardware_gate_denied",
            errors=[task_decision.reason],
        )

    health_command = health_monitor.build_status_command(context)
    if args.allow_telemetry_only_health:
        health_command.append("--allow-telemetry-only")
        warnings.append(
            "live preflight accepted telemetry-only robot health; "
            "the RPC status cannot rule out a Franka controller fault"
        )
    health_result = _run_gated_command(
        command_runner,
        trace,
        context,
        health_command,
        entrypoint_ref="PREFLIGHT_ROBOT_HEALTH_CHECK",
        entrypoint=health_entrypoint,
        artifact_prefix="robot_health_preflight_command",
    )
    if not health_result.executed:
        return _abort(trace, output_json, context, stages, outputs, reset_results, "hardware_gate_denied", errors=[str((health_result.gate_decision or {}).get("reason"))])
    health_raw = health_result.raw_json or _last_json_object(health_result.stdout) or health_result.stdout
    health = health_monitor.parse_health_output(health_raw)
    outputs["robot_health"] = health.to_dict()
    trace.write_health("robot_health_preflight.json", health)
    _stage(trace, stages, TaskState.PREFLIGHT_ROBOT_HEALTH_CHECK, {"robot_health": health.to_dict()}, ok=health.state != RobotHealthState.ESTOP_OR_UNSAFE)
    if health.state == RobotHealthState.ESTOP_OR_UNSAFE:
        return _abort(trace, output_json, context, stages, outputs, reset_results, "estop_or_unsafe_requires_manual_intervention")
    if reset_controller.should_reset(health, None, context):
        reset = _live_reset(
            trace=trace,
            stages=stages,
            context=context,
            command_runner=command_runner,
            reset_controller=reset_controller,
            health_monitor=health_monitor,
            before_health=health,
            reason="preflight_robot_health",
        )
        reset_results.append(reset)
        if not reset.ok:
            return _abort(trace, output_json, context, stages, outputs, reset_results, "reset_failed_or_robot_still_abnormal", errors=reset.errors)
    elif not health.ok:
        return _abort(trace, output_json, context, stages, outputs, reset_results, "robot_health_not_ready", errors=health.errors, warnings=health.warnings)

    tube_path = args.tube_result_json or trace.path("tube_detection.json")
    locate_command = [
        str(OBJECT_LOCATOR),
        "--config",
        str(TUBE_CONFIG),
        "--result-json",
        str(tube_path),
        "--json",
    ]
    locate_result = _run_gated_command(
        command_runner,
        trace,
        context,
        locate_command,
        entrypoint_ref="LOCATE_TUBE",
        entrypoint=locate_entrypoint,
        artifact_prefix="locate_tube",
    )
    if locate_result.returncode != 0 or not tube_path.exists():
        if locate_result.abnormal_robot_state_detected and context.auto_reset_on_abnormal:
            reset = _live_reset(
                trace=trace,
                stages=stages,
                context=context,
                command_runner=command_runner,
                reset_controller=reset_controller,
                health_monitor=health_monitor,
                before_health=None,
                reason="locate_tube_abnormal",
            )
            reset_results.append(reset)
        return _abort(trace, output_json, context, stages, outputs, reset_results, "locate_tube_failed", errors=[locate_result.stderr or f"returncode={locate_result.returncode}"])
    tube = extract_tube_geometry(tube_path)
    if not tube.get("ok") or float(tube.get("confidence") or 0.0) < float(args.min_tube_confidence):
        return _abort(trace, output_json, context, stages, outputs, reset_results, "locate_tube_failed", errors=tube.get("errors", []))
    outputs.update({
        "tube_pose": tube["tube_pose"],
        "tube_xyz": tube["tube_xyz"],
        "tube_mouth_direction": tube.get("tube_mouth_direction"),
        "tube_body_axis": tube.get("tube_body_axis"),
    })
    warnings.extend(tube.get("warnings", []))
    _stage(trace, stages, TaskState.LOCATE_TUBE, {"tube_pose": tube["tube_pose"], "executed": True}, warnings=tube.get("warnings", []))

    arm = select_arm_for_tail_side(tube)
    if not arm.get("ok"):
        return _abort(trace, output_json, context, stages, outputs, reset_results, str(arm.get("reason")))
    outputs.update({"selected_arm": arm["selected_arm"], "opposite_arm": arm["opposite_arm"]})
    _stage(trace, stages, TaskState.SELECT_ARM, arm)
    grasp_plan = build_pregrasp_and_grasp_plan(tube, selected_arm=arm["selected_arm"])
    outputs.update({"pregrasp_pose": grasp_plan["pregrasp_pose"], "grasp_pose": grasp_plan["grasp_pose"]})
    warnings.extend(grasp_plan.get("warnings", []))
    _stage(trace, stages, TaskState.MOVE_TO_PREGRASP, {"pregrasp_pose": grasp_plan["pregrasp_pose"], "command_pending": True}, warnings=grasp_plan.get("warnings", []))
    _stage(trace, stages, TaskState.ALIGN_GRASP_POSE, {"grasp_pose": grasp_plan["grasp_pose"], "command_pending": True}, warnings=grasp_plan.get("warnings", []))

    context.held_object_state = HeldObjectState.TUBE_BODY_SELECTED_ARM
    grasp_command = [
        str(GRASP_SCRIPT),
        "--arm",
        str(arm["selected_arm"]),
        "--result-json",
        str(tube_path),
        "--execute",
    ]
    grasp_result = _run_gated_command(
        command_runner,
        trace,
        context,
        grasp_command,
        entrypoint_ref="GRASP_AND_HANDOVER",
        entrypoint=grasp_entrypoint,
        artifact_prefix="grasp_and_handover",
    )
    if grasp_result.returncode != 0:
        if grasp_result.abnormal_robot_state_detected and context.auto_reset_on_abnormal:
            reset = _live_reset(
                trace=trace,
                stages=stages,
                context=context,
                command_runner=command_runner,
                reset_controller=reset_controller,
                health_monitor=health_monitor,
                before_health=None,
                reason="grasp_or_handover_abnormal",
            )
            reset_results.append(reset)
            reason = "reset_while_holding_tube_requires_reverification"
        else:
            reason = "grasp_or_handover_failed"
        return _abort(trace, output_json, context, stages, outputs, reset_results, reason, errors=[grasp_result.stderr or f"returncode={grasp_result.returncode}"])
    _stage(trace, stages, TaskState.GRASP_TUBE_BODY, {"executed": True, "held_object_state": HeldObjectState.TUBE_BODY_SELECTED_ARM.value})
    context.held_object_state = HeldObjectState.TUBE_HEAD_HOLDER_ARM
    outputs["handover_pose"] = {"ref": str(GRASP_SCRIPT), "holder_arm": arm["opposite_arm"]}
    _stage(trace, stages, TaskState.HANDOVER, {"executed": True, "holder_arm": arm["opposite_arm"], "held_object_state": context.held_object_state.value}, warnings=["handover confirmation is based on command success; force/contact validation remains limited"])

    insertion_command = [
        "python3",
        str(INSERT_FLOW),
        "--execute",
        "--holder-side",
        str(arm["opposite_arm"]),
        "--log-file",
        str(trace.path("insertion_flow.log")),
        "--compact",
    ]
    insertion_result = _run_gated_command(
        command_runner,
        trace,
        context,
        insertion_command,
        entrypoint_ref="LOCATE_RACK_HOLE_INSERT_RELEASE_RETRACT",
        entrypoint=insertion_entrypoint,
        artifact_prefix="insertion_flow_command",
    )
    insertion_report = insertion_result.raw_json or _last_json_object(insertion_result.stdout) or {}
    if insertion_result.returncode != 0:
        if insertion_result.abnormal_robot_state_detected and context.auto_reset_on_abnormal:
            reset = _live_reset(
                trace=trace,
                stages=stages,
                context=context,
                command_runner=command_runner,
                reset_controller=reset_controller,
                health_monitor=health_monitor,
                before_health=None,
                reason="insertion_flow_abnormal",
            )
            reset_results.append(reset)
            reason = "reset_while_holding_tube_requires_reverification"
        else:
            reason = str(insertion_report.get("stopped_reason") or "insertion_flow_failed")
        return _abort(trace, output_json, context, stages, outputs, reset_results, reason, errors=[insertion_result.stderr or f"returncode={insertion_result.returncode}"])

    child_stages = insertion_report.get("stages", []) if isinstance(insertion_report, dict) else []
    outputs["insertion_report"] = insertion_report
    _stage(trace, stages, TaskState.LOCATE_RACK, {"executed": True, "insertion_child_stage": child_stages[0] if len(child_stages) > 0 else None})
    _stage(trace, stages, TaskState.NON_HOLDER_HOME, {"non_holder_arm": arm["selected_arm"], "handled_by_insertion_flow": True})
    _stage(trace, stages, TaskState.HOLDER_ALIGN_RACK_ABOVE, {"holder_arm": arm["opposite_arm"], "executed": True})
    _stage(trace, stages, TaskState.WRIST_LOCATE_HOLE, {"executed": True, "insertion_child_stage": child_stages[1] if len(child_stages) > 1 else None})
    _stage(trace, stages, TaskState.SELECT_HOLE, {"executed": True})
    _stage(trace, stages, TaskState.INSERT_ALIGN_REFINE, {"executed": True})
    context.held_object_state = HeldObjectState.TUBE_INSERTED_NOT_RELEASED
    _stage(trace, stages, TaskState.INSERT_DESCEND, {"executed": True, "insertion_child_stage": child_stages[2] if len(child_stages) > 2 else None, "held_object_state": context.held_object_state.value})
    context.held_object_state = HeldObjectState.RELEASED
    _stage(trace, stages, TaskState.RELEASE, {"executed": True, "held_object_state": context.held_object_state.value})
    _stage(
        trace,
        stages,
        TaskState.RETURN_HOME,
        {
            "executed": True,
            "vertical_clearance_before_home": True,
            "single_arm_home": True,
        },
    )

    warnings.extend([
        "handover physical contact/force confirmation remains limited",
        "insertion success is based on child command completion; closed-loop force verification remains limited",
    ])
    outputs.update({
        "retry_count": 0,
        "task_state": TaskState.COMPLETE.value,
        "held_object_state": context.held_object_state.value,
        "reset_recovery": [item.to_dict() for item in reset_results],
        "completion_flag": True,
        "physical_verified": True,
    })
    _stage(trace, stages, TaskState.COMPLETE, {"completion_flag": True, "physical_verified": True})
    result = TaskResult(True, TaskState.COMPLETE, True, True, context, stages, outputs, reset_results, warnings=warnings)
    trace.write_task_result(result, output_json)
    return result


def run(args: argparse.Namespace) -> TaskResult:
    run_id = new_run_id("pick_tube_insert_rack")
    artifact_dir = args.artifact_dir or Path("/tmp/agentic_skills_runs") / run_id
    output_json = args.output_json or artifact_dir / "task_result.json"
    mode = SkillMode(args.mode)
    context = SkillContext(
        run_id=run_id,
        mode=mode,
        hardware_allowed=bool(args.hardware_allowed),
        execute=bool(args.execute),
        artifact_dir=str(artifact_dir),
        manifest_path=str(args.manifest),
        robot_server=args.robot_server,
        thresholds={
            "translation_threshold_m": float(args.translation_threshold_m),
            "rotation_threshold_rad": float(args.rotation_threshold_rad),
            "max_refine_retries": int(args.max_refine_retries),
        },
        recovery_policy={
            "auto_reset_on_abnormal": bool(args.auto_reset_on_abnormal),
            "resume_after_held_object_reset": bool(args.resume_after_held_object_reset),
        },
        max_auto_reset_attempts=int(args.max_auto_reset_attempts),
        auto_reset_on_abnormal=bool(args.auto_reset_on_abnormal),
        resume_after_held_object_reset=bool(args.resume_after_held_object_reset),
        mock_robot_health=args.mock_robot_health,
        robot_health_json=str(args.robot_health_json) if args.robot_health_json else None,
        reset_recovery_json=str(args.reset_recovery_json) if args.reset_recovery_json else None,
    )
    trace = TraceWriter(artifact_dir)
    manifest = load_manifest(args.manifest)
    trace.write_context(context)
    trace.write_manifest_snapshot(manifest)
    stages: list[StageResult] = []
    reset_results = []
    outputs: dict[str, object] = {}
    warnings: list[str] = []

    _stage(trace, stages, TaskState.INIT, {"run_id": run_id, "mode": mode.value})
    if mode == SkillMode.LIVE:
        return _run_live(
            args,
            trace=trace,
            output_json=output_json,
            context=context,
            manifest=manifest,
            stages=stages,
            outputs=outputs,
            warnings=warnings,
        )

    health_monitor = RobotHealthMonitor(manifest, repo_root=REPO_ROOT)
    reset_controller = ResetRecoveryController(manifest, repo_root=REPO_ROOT)

    health = health_monitor.check_health(context, source="preflight")
    trace.write_health("robot_health_preflight.json", health)
    _stage(trace, stages, TaskState.PREFLIGHT_ROBOT_HEALTH_CHECK, {"robot_health": health.to_dict()}, ok=health.state.value != "ESTOP_OR_UNSAFE")
    if health.state.value == "ESTOP_OR_UNSAFE":
        result = TaskResult(False, TaskState.ABORT, False, False, context, stages, outputs, reset_results, "estop_or_unsafe_requires_manual_intervention")
        trace.write_task_result(result, output_json)
        return result
    if reset_controller.should_reset(health, None, context):
        reset = reset_controller.perform_auto_reset(
            context,
            reason="preflight_robot_health",
            held_object_state=context.held_object_state,
            before_health=health,
        )
        reset_results.append(reset)
        trace.write_reset(reset.attempt_index, reset)
        _stage(trace, stages, TaskState.AUTO_RESET_RECOVERY, {"reset_recovery": reset.to_dict()}, ok=reset.ok)
        verified = reset.after_health or reset_controller.verify_after_reset(context)
        trace.write_health("robot_health_after_reset.json", verified)
        _stage(trace, stages, TaskState.RESET_RECOVERY_VERIFY, {"robot_health": verified.to_dict()}, ok=verified.ok)
        if not reset.ok or (reset.held_object_risk and not context.resume_after_held_object_reset):
            reason = "reset_while_holding_tube_requires_reverification" if reset.held_object_risk else "reset_failed_or_robot_still_abnormal"
            result = TaskResult(False, TaskState.ABORT, False, False, context, stages, outputs, reset_results, reason)
            trace.write_task_result(result, output_json)
            return result

    tube_path = _fixture(args.tube_result_json, "mock_tube_detection.json")
    if mode in (SkillMode.DRY_RUN, SkillMode.LIVE):
        _plan(trace, TaskState.LOCATE_TUBE, ["object-locator", "--config", "config_test_tube_cleanup_leftmost_vlm_head.yaml", "--json"])
    tube = extract_tube_geometry(tube_path)
    if not tube.get("ok") or float(tube.get("confidence") or 0.0) < float(args.min_tube_confidence):
        result = TaskResult(False, TaskState.ABORT, False, False, context, stages, outputs, reset_results, "locate_tube_failed", errors=tube.get("errors", []))
        trace.write_task_result(result, output_json)
        return result
    outputs.update(
        {
            "tube_pose": tube["tube_pose"],
            "tube_xyz": tube["tube_xyz"],
            "tube_mouth_direction": tube.get("tube_mouth_direction"),
            "tube_body_axis": tube.get("tube_body_axis"),
        }
    )
    warnings.extend(tube.get("warnings", []))
    _stage(trace, stages, TaskState.LOCATE_TUBE, {"tube_pose": tube["tube_pose"]}, warnings=tube.get("warnings", []))

    arm = select_arm_for_tail_side(tube)
    if not arm.get("ok"):
        result = TaskResult(False, TaskState.ABORT, False, False, context, stages, outputs, reset_results, arm.get("reason"))
        trace.write_task_result(result, output_json)
        return result
    outputs.update({"selected_arm": arm["selected_arm"], "opposite_arm": arm["opposite_arm"]})
    _stage(trace, stages, TaskState.SELECT_ARM, arm)

    plan = build_pregrasp_and_grasp_plan(tube, selected_arm=arm["selected_arm"])
    outputs.update({"pregrasp_pose": plan["pregrasp_pose"], "grasp_pose": plan["grasp_pose"]})
    warnings.extend(plan.get("warnings", []))
    _plan(trace, TaskState.MOVE_TO_PREGRASP, ["python3", "scripts/grasp_right_arm_xyz.py", "--result-json", str(tube_path), "--arm", arm["selected_arm"]])
    _stage(trace, stages, TaskState.MOVE_TO_PREGRASP, {"pregrasp_pose": plan["pregrasp_pose"]}, warnings=plan.get("warnings", []))
    _stage(trace, stages, TaskState.ALIGN_GRASP_POSE, {"grasp_pose": plan["grasp_pose"]}, warnings=plan.get("warnings", []))

    context.held_object_state = HeldObjectState.TUBE_BODY_SELECTED_ARM
    _stage(trace, stages, TaskState.GRASP_TUBE_BODY, {"object_present": mode == SkillMode.MOCK, "held_object_state": context.held_object_state.value})

    context.held_object_state = HeldObjectState.TUBE_HEAD_HOLDER_ARM
    outputs["handover_pose"] = {"ref": "procedure_skills/dual_franka_handover_transition/config/transition.json"}
    _plan(trace, TaskState.HANDOVER, ["python3", "scripts/handover_transition.py", "--active-arm", arm["selected_arm"]])
    _stage(trace, stages, TaskState.HANDOVER, {"held_object_state": context.held_object_state.value}, warnings=["handover physical contact verification still needs live validation"])

    rack_path = _fixture(args.rack_result_json, "mock_rack_detection.json")
    if mode in (SkillMode.DRY_RUN, SkillMode.LIVE):
        _plan(trace, TaskState.LOCATE_RACK, ["object-locator", "--config", "config_rack_center_vlm.yaml", "--json"])
    rack_data = json.loads(Path(rack_path).read_text(encoding="utf-8"))
    rack_pose = rack_data.get("pose") or {
        "frame": "base",
        "xyz_m": [rack_data["position_base"]["x_m"], rack_data["position_base"]["y_m"], rack_data["position_base"]["z_m"]],
        "rotvec_rad": None,
        "source": "mock_rack_detection",
    }
    outputs["rack_pose"] = rack_pose
    _stage(trace, stages, TaskState.LOCATE_RACK, {"rack_pose": rack_pose})
    _stage(trace, stages, TaskState.NON_HOLDER_HOME, {"opposite_arm": arm["selected_arm"]})
    _stage(trace, stages, TaskState.HOLDER_ALIGN_RACK_ABOVE, {"rack_pose": rack_pose})

    hole_path = _fixture(args.hole_result_json, "mock_hole_detection.json")
    if mode in (SkillMode.DRY_RUN, SkillMode.LIVE):
        _plan(trace, TaskState.WRIST_LOCATE_HOLE, ["object-locator", "--config", "config_rack_empty_hole_<side>_wrist_vlm.yaml", "--json"])
    candidates = wrap_hole_candidates(hole_path)
    outputs["empty_hole_candidates"] = candidates
    _stage(trace, stages, TaskState.WRIST_LOCATE_HOLE, {"empty_hole_candidates": candidates})
    hole = select_highest_confidence_hole(candidates, min_confidence=args.min_hole_confidence)
    if not hole.get("ok"):
        result = TaskResult(False, TaskState.ABORT, False, False, context, stages, outputs, reset_results, hole.get("reason"))
        trace.write_task_result(result, output_json)
        return result
    outputs["selected_hole_pose"] = hole["selected_hole_pose"]
    outputs["hole_confidence"] = hole["hole_confidence"]
    _stage(trace, stages, TaskState.SELECT_HOLE, hole)

    error = build_error_6dof(
        translation_m=0.0,
        rotation_rad=0.0,
        translation_threshold_m=args.translation_threshold_m,
        rotation_threshold_rad=args.rotation_threshold_rad,
        source="mock_or_planned_refine",
    )
    outputs["error_xyz"] = error.translation_m
    outputs["error_rot"] = error.rotation_rad
    outputs["error_6dof"] = error.to_dict()
    outputs["retry_count"] = 0
    outputs["task_state"] = TaskState.COMPLETE.value
    outputs["held_object_state"] = HeldObjectState.RELEASED.value
    outputs["robot_health"] = health.to_dict()
    outputs["reset_recovery"] = [item.to_dict() for item in reset_results]
    outputs["completion_flag"] = True
    outputs["physical_verified"] = bool(mode == SkillMode.LIVE and args.execute)
    _stage(trace, stages, TaskState.INSERT_ALIGN_REFINE, {"error_6dof": error.to_dict(), "policy": build_error_policy(args.max_refine_retries, args.translation_threshold_m, args.rotation_threshold_rad)})
    context.held_object_state = HeldObjectState.TUBE_INSERTED_NOT_RELEASED
    _stage(trace, stages, TaskState.INSERT_DESCEND, {"insert_stage_ok": True, "held_object_state": context.held_object_state.value})
    context.held_object_state = HeldObjectState.RELEASED
    _stage(trace, stages, TaskState.RELEASE, {"release_gate": "insert_stage_ok", "held_object_state": context.held_object_state.value})
    _stage(trace, stages, TaskState.RETURN_HOME, {"planned": True})
    _stage(trace, stages, TaskState.COMPLETE, {"completion_flag": True, "physical_verified": outputs["physical_verified"]})

    result = TaskResult(True, TaskState.COMPLETE, True, bool(outputs["physical_verified"]), context, stages, outputs, reset_results, warnings=warnings)
    trace.write_task_result(result, output_json)
    return result


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    result = run(args)
    print(json.dumps(result.to_dict(), ensure_ascii=False, indent=2))
    return 0 if result.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
