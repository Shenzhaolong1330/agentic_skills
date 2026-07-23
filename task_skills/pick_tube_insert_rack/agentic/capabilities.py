from __future__ import annotations

"""Trusted offline capabilities for the tube task package.

These contracts are intentionally not added to the public manifest.  They are
available only through the TaskDefinition compilation context.
"""

from pathlib import Path
from typing import Any

from agentic_skills_harness.capability import CapabilityContract, VerifierContract
from agentic_skills_harness.contracts.enums import CapabilityKind, ErrorCode, ResourceMode, RiskClass
from agentic_skills_harness.contracts.resources import ResourceRequirement
from agentic_skills_harness.registry import CapabilityRegistry
from agentic_skills_harness.schema_validation import load_local_schema


TASK_PREFIX = "task.pick_tube_insert_rack"
COMPUTE_CAPABILITIES = (
    f"{TASK_PREFIX}.compute.extract_geometry",
    f"{TASK_PREFIX}.compute.select_arm",
    f"{TASK_PREFIX}.compute.build_grasp_plan",
    f"{TASK_PREFIX}.compute.select_hole",
    f"{TASK_PREFIX}.compute.project_facts",
)
OBSERVE_CAPABILITY = f"{TASK_PREFIX}.observe.fixture"
ACTION_CAPABILITIES = {
    "move": f"{TASK_PREFIX}.action.plan_move",
    "grasp": f"{TASK_PREFIX}.action.plan_grasp",
    "handover": f"{TASK_PREFIX}.action.plan_handover",
    "insert": f"{TASK_PREFIX}.action.plan_insert",
    "release": f"{TASK_PREFIX}.action.plan_release",
    "retract": f"{TASK_PREFIX}.action.plan_retract",
}


def _internal_capability(capability_id: str, *, kind: CapabilityKind, output_schema_ref: str) -> CapabilityContract:
    is_observation = kind == CapabilityKind.OBSERVATION
    return CapabilityContract(
        name=capability_id.rsplit(".", 1)[-1], capability_id=capability_id, capability_version="1.0.0",
        kind=kind, visibility="internal", path="task_skills/pick_tube_insert_rack/agentic", type="python",
        input_schema_ref="schemas/capabilities/empty_input.schema.json", output_schema_ref=output_schema_ref,
        requires_hardware=False, opens_camera=False, connects_robot_rpc=False, moves_robot=False, controls_gripper=False,
        physical_side_effects=(), risk_class=RiskClass.NONE,
        resources=(ResourceRequirement("task.pick_tube_insert_rack.compute", ResourceMode.SHARED, "task-local deterministic computation"),),
        preconditions=(), effects=("offline_result_available",), invalidates=(), timeout_s=5.0,
        verifier=VerifierContract("output_schema", None, False, False, "Offline task fixture contract only."),
        error_codes=(ErrorCode.CAPABILITY_EXECUTION_FAILED if not is_observation else ErrorCode.PERCEPTION_NOT_FOUND,),
        default_safe_to_run=True, allowed_as_recovery=False, execute_flag=None,
        notes="First-party offline task capability; no hardware or subprocess access.",
        description="Trusted task-local offline capability.", adapter_id="builtin.task.pick_tube_insert_rack.v1",
        dispatch_support="supported", artifact_policy={"allow_output": True, "filename": "task_result.json"},
        mode_support={"mock": "supported", "dry_run": "supported", "from_artifacts": "supported", "live": "disabled"},
    )


def internal_capabilities() -> tuple[CapabilityContract, ...]:
    values = [_internal_capability(item, kind=CapabilityKind.COMPUTE, output_schema_ref="schemas/action_result.schema.json") for item in COMPUTE_CAPABILITIES]
    values.append(_internal_capability(OBSERVE_CAPABILITY, kind=CapabilityKind.OBSERVATION, output_schema_ref="schemas/observation_result.schema.json"))
    values.extend(_internal_capability(item, kind=CapabilityKind.ACTION, output_schema_ref="schemas/action_result.schema.json") for item in ACTION_CAPABILITIES.values())
    return tuple(values)


class TaskCapabilityRegistry:
    """Read-only composite registry used only by a trusted task definition."""

    def __init__(self, base: Any, additional: tuple[CapabilityContract, ...] | None = None) -> None:
        self.base = base
        self._additional = {item.capability_id: item for item in (additional or internal_capabilities())}
        overlap = set(self._additional) & {item.capability_id for item in base.list(include_internal=True, include_legacy=True)}
        if overlap:
            raise ValueError(f"task capability IDs overlap manifest IDs: {sorted(overlap)}")
        self.repo_root = Path(base.repo_root)

    def get(self, capability_id: str) -> CapabilityContract | None:
        return self._additional.get(capability_id) or self.base.get(capability_id)

    def require(self, capability_id: str) -> CapabilityContract:
        value = self.get(capability_id)
        if value is None:
            raise KeyError(capability_id)
        return value

    def list(self, **kwargs: Any) -> tuple[CapabilityContract, ...]:
        values = list(self.base.list(**kwargs))
        include_internal = bool(kwargs.get("include_internal", False))
        include_legacy = bool(kwargs.get("include_legacy", False))
        visibility = kwargs.get("visibility")
        if include_internal or visibility == "internal":
            values.extend(self._additional.values())
        if visibility == "public":
            values = [item for item in values if item.visibility == "public"]
        if kwargs.get("kind") is not None:
            expected = kwargs["kind"].value if hasattr(kwargs["kind"], "value") else kwargs["kind"]
            values = [item for item in values if item.kind.value == expected]
        return tuple(sorted(values, key=lambda item: item.capability_id))

    def resolve_input_schema(self, capability_id: str) -> Any:
        item = self.require(capability_id)
        return load_local_schema(item.input_schema_ref, self.repo_root)

    def resolve_output_schema(self, capability_id: str) -> Any:
        item = self.require(capability_id)
        return load_local_schema(item.output_schema_ref, self.repo_root)

    def validate(self) -> list[str]:
        return list(self.base.validate())


__all__ = ["ACTION_CAPABILITIES", "COMPUTE_CAPABILITIES", "OBSERVE_CAPABILITY", "TaskCapabilityRegistry", "internal_capabilities"]
