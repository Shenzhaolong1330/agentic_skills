from __future__ import annotations

from agentic_skills_harness.planning.envelope import ApprovalPolicy, ExecutionEnvelope, TargetMode, WorkspaceConstraint

from .capabilities import ACTION_CAPABILITIES, COMPUTE_CAPABILITIES, OBSERVE_CAPABILITY


def canonical_envelope(*, mode: str = "mock") -> ExecutionEnvelope:
    if mode == TargetMode.LIVE.value:
        raise ValueError("pick_tube_insert_rack TaskDefinition supports offline modes only")
    allowed_internal = (*COMPUTE_CAPABILITIES, OBSERVE_CAPABILITY, *ACTION_CAPABILITIES.values())
    return ExecutionEnvelope(
        envelope_id=f"pick_tube_insert_rack.offline.{mode}", target_mode=mode,
        allowed_capabilities=(), forbidden_capabilities=("recovery.robot_reset", "recovery.robot_recover", "state.reset_realsense"),
        risk_ceiling="NONE", allowed_resources=("task.pick_tube_insert_rack.compute",),
        workspace_constraints=(WorkspaceConstraint("mock_workspace", "base", (-1.0, -1.0, -1.0), (1.0, 1.0, 1.0), ("tube", "rack", "rack_hole", "gripper"), "offline fixture workspace"),),
        max_nodes=64, max_depth=64, max_elapsed_s=120.0, max_tool_calls=32, max_replans=2,
        max_recovery_actions=3, max_same_error_retries=1, no_progress_limit=2,
        approval_policy=ApprovalPolicy((), True, True, "E-stop always requires a human; live is disabled."),
        metadata={"task_id": "pick_tube_insert_rack", "task_version": "1.0.0", "internal_capability_allowlist": list(allowed_internal), "supported_offline_modes": ["mock", "dry_run", "from_artifacts"]},
    )


__all__ = ["canonical_envelope"]
