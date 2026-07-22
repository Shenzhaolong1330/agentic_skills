from __future__ import annotations

from pathlib import Path
from typing import Any

from .adapter_registry import AdapterRegistry
from .adapter import UnsupportedAdapter
from .adapters.fixed import FirstPartyFixedAdapter, build_fixed_adapter_registry
from .models import DispatchContext, DispatchRequest


CORE_SAMPLE_ARGUMENTS: dict[str, dict[str, Any]] = {
    "motion.move_to_pose": {"frame": "base", "xyz_m": [0.2, 0.0, 0.3], "source": "safe_fixture", "side": "left"},
    "gripper.command": {"operation": "status", "side": "left"},
    "procedure.handover_transition": {"holder_side": "left"},
}


def sample_arguments(capability_id: str) -> dict[str, Any]:
    return dict(CORE_SAMPLE_ARGUMENTS.get(capability_id, {}))


def _binding_metadata(capability_id: str) -> tuple[list[str], list[str]]:
    mappings = {
        "motion.move_to_pose": ["side -> --left-pose/--right-pose", "xyz_m + rotvec_rad -> one JSON argv"],
        "gripper.command": ["operation -> fixed positional command", "side -> --side"],
        "gripper.observe_status": ["fixed status command", "fixed both side"],
        "robot.observe_health": ["fixed status command"],
        "robot.recover_reset_home": ["fixed ensure command"],
        "state.capture_realsense": ["fixed capture entrypoint", "artifact_dir -> fixed output directory"],
        "state.reset_realsense": ["fixed --reset-realsense"],
        "procedure.handover_transition": ["holder_side -> --active-arm", "fixed preset -> --transition-json"],
        "recovery.robot_reset": ["fixed shell wrapper"],
        "recovery.robot_recover": ["fixed shell wrapper"],
        "motion.go_home": ["fixed shell wrapper"],
    }
    return mappings.get(capability_id, []), ["mode", "hardware_allowed", "execute", "artifact_dir"]


def audit_adapter_coverage(registry: Any, *, artifact_dir: str | Path = "/tmp/agentic_skills_adapter_audit") -> dict[str, Any]:
    adapters = build_fixed_adapter_registry(registry)
    adapter_registry = AdapterRegistry(registry, adapters)
    rows: list[dict[str, Any]] = []
    samples: list[dict[str, Any]] = []
    core_with_plans = 0
    for capability in registry.list(include_internal=True, include_legacy=True):
        adapter = adapters.get(capability.capability_id)
        input_mapping, context_mapping = _binding_metadata(capability.capability_id)
        adapter_id = capability.adapter_id or (f"unsupported.{capability.capability_id}" if isinstance(adapter, UnsupportedAdapter) else getattr(adapter, "adapter_id", None)) or f"unsupported.{capability.capability_id}"
        status = capability.dispatch_support
        row: dict[str, Any] = {
            "capability_id": capability.capability_id,
            "visibility": capability.visibility,
            "adapter_id": adapter_id,
            "dispatch_status": status,
            "mode_support": dict(capability.mode_support),
            "binding_source": f"first_party:{adapter_id}" if isinstance(adapter, FirstPartyFixedAdapter) else "none:contract-insufficient",
            "input_mapping": input_mapping,
            "fixed_arguments": list(getattr(getattr(adapter, "binding", None), "fixed_arguments", ())),
            "context_derived_arguments": context_mapping if isinstance(adapter, FirstPartyFixedAdapter) else [],
            "generated_artifacts": [],
            "output_parser": "json" if isinstance(adapter, FirstPartyFixedAdapter) else None,
            "error_mapping": {code.value: code.value for code in capability.error_codes} if isinstance(adapter, FirstPartyFixedAdapter) else {},
            "limitations": [],
            "evidence": ["manifest contract", "first-party adapter registry", "offline dry-run plan probe"],
        }
        if not isinstance(adapter, FirstPartyFixedAdapter):
            row["limitations"] = ["no safe fixed binding is available", "live execution disabled"]
        elif capability.requires_hardware:
            row["limitations"] = ["plan_only; live is not validated and disabled in this phase", "no hardware invocation performed"]
        try:
            if isinstance(adapter, FirstPartyFixedAdapter):
                plan = adapter.build_plan(DispatchRequest(capability.capability_id, sample_arguments(capability.capability_id), request_id=f"coverage_{capability.capability_id.replace('.', '_')}"), capability, DispatchContext(mode="dry_run", artifact_dir=artifact_dir))
                row["generated_artifacts"] = list(plan.generated_artifacts)
                row["output_parser"] = plan.output_parser
                samples.append(plan.to_public_dict())
                core_with_plans += 1
        except Exception as exc:
            row["limitations"].append(f"sample plan rejected: {exc}")
        rows.append(row)
    duplicate_bindings = len(rows) - len({item["adapter_id"] for item in rows})
    live_hardware_supported = sum(item["mode_support"].get("live") == "supported" for item in rows if item["visibility"] != "legacy")
    return {
        "total_capabilities": len(rows),
        "reviewed_capabilities": len(rows),
        "supported": sum(item["dispatch_status"] == "supported" for item in rows),
        "plan_only": sum(item["dispatch_status"] == "plan_only" for item in rows),
        "unsupported": sum(item["dispatch_status"] == "unsupported" for item in rows),
        "core_capabilities_with_plan": core_with_plans,
        "live_hardware_supported": live_hardware_supported,
        "unreviewed": 0,
        "duplicate_adapter_bindings": duplicate_bindings,
        "adapter_registry_errors": adapter_registry.validate_against_capability_registry(),
        "capabilities": rows,
        "adapter_plan_samples": samples,
    }
