#!/usr/bin/env python3
"""Static S10 capability implementation audit; it never imports entrypoints."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import re
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))


TARGETS = {
    "observe.robot_state": ("robot.observe_health", "H1_READ_ONLY"),
    "observe.scene": ("perception.locate_object_3d", "H1_READ_ONLY"),
    "geometry.transform_pose": (None, "H0_CONFIG"),
    "motion.move_to_pose": ("motion.move_to_pose", "H3_MOTION_P2P"),
    "motion.move_relative": (None, "H4_MOTION_RELATIVE"),
    "motion.guarded_move": (None, "H5_STOP"),
    "gripper.command": ("gripper.command", "H2_GRIPPER_EMPTY"),
    "gripper.verify_grasp": (None, "H6_GRASP_VERIFICATION"),
    "motion.safe_stop": (None, "H5_STOP"),
    "robot.recover_fault": (None, "H7_RECOVERY"),
    "procedure.reset_home": ("robot.recover_reset_home", "H8_RESET_HOME"),
}


def audit(repo: Path) -> dict[str, Any]:
    from agentic_skills_harness.manifest import load_manifest
    from agentic_skills_harness.registry import CapabilityRegistry
    from agentic_skills_harness.dispatch.adapters.fixed import build_fixed_adapter_registry

    manifest = load_manifest(repo / "skill_manifest.json")
    registry = CapabilityRegistry.from_manifest(manifest, repo_root=repo)
    adapters = build_fixed_adapter_registry(registry)
    rows: list[dict[str, Any]] = []
    for operation_id, (capability_id, level) in TARGETS.items():
        capability = registry.get(capability_id) if capability_id else None
        entrypoint = None
        if capability:
            for skill in manifest["skills"]:
                for entry in skill["entrypoints"]:
                    if entry["capability_id"] == capability.capability_id:
                        entrypoint = entry
        implementation_exists = bool(capability and entrypoint and (repo / entrypoint["path"]).exists())
        skill_md_exists = False
        skill_text = ""
        entrypoint_text = ""
        if capability and entrypoint:
            skill = next((item for item in manifest["skills"] if any(ep.get("capability_id") == capability.capability_id for ep in item["entrypoints"])), None)
            if skill:
                skill_path = repo / skill["path"]
                skill_md_exists = skill_path.exists()
                skill_text = skill_path.read_text(encoding="utf-8", errors="replace") if skill_md_exists else ""
            entrypoint_path = repo / entrypoint["path"]
            entrypoint_text = entrypoint_path.read_text(encoding="utf-8", errors="replace") if entrypoint_path.exists() else ""
        source = skill_text + "\n" + entrypoint_text
        audit_details = {
            "manifest_entrypoint": bool(entrypoint), "skill_md_exists": skill_md_exists, "entrypoint_exists": implementation_exists,
            "argparse_contract": "argparse" in source or bool(capability and capability.kind.value == "compute"),
            "shell_wrapper": bool(entrypoint and entrypoint.get("type") == "shell") or "wrapper" in source.lower(),
            "rpc_client_evidence": bool(re.search(r"rpc|zerorpc|robot[_-]?client|server[_-]?(host|port)", source, re.I)),
            "output_json_evidence": "json" in source.lower() or bool(capability and capability.output_schema_ref),
            "error_handling_evidence": bool(re.search(r"except|error|returncode|timeout", source, re.I)),
            "timeout_evidence": bool(capability and capability.timeout_s > 0) or bool(re.search(r"timeout", source, re.I)),
            "stop_semantics_evidence": bool(re.search(r"cancel|stop|hold|pause|servo", source, re.I)),
            "side_effects_from_manifest": list(capability.physical_side_effects) if capability else [],
            "verifier_from_manifest": capability.verifier.to_dict() if capability else None,
        }
        static_binding = bool(capability and capability.adapter_id and capability.dispatch_support != "unsupported" and capability.capability_id in adapters and adapters[capability.capability_id].supports(capability))
        limitations: list[str] = []
        gaps: list[str] = []
        if operation_id in {"motion.guarded_move", "motion.safe_stop", "robot.recover_fault", "geometry.transform_pose", "motion.move_relative", "gripper.verify_grasp"} and not capability_id:
            limitations.append("no manifest capability has a fixed underlying API binding")
        if operation_id == "motion.guarded_move": gaps.append("no statically verified real-time force-limited primitive and stop API")
        if operation_id == "motion.safe_stop": gaps.append("no statically verified cancel/hold/servo-stop API")
        if operation_id == "robot.recover_fault": gaps.append("existing reset/home binding is not evidence of fault-recovery API")
        if operation_id == "gripper.verify_grasp": gaps.append("close return code alone is insufficient; independent evidence remains limited")
        if operation_id == "observe.robot_state": limitations.append("legacy health entrypoint does not prove all telemetry fields")
        if operation_id in {"observe.scene", "perception.locate_object_3d"}: limitations.append("locator uses a fixed preset/config binding; hardware not opened by audit")
        status = "IMPLEMENTATION_READY" if implementation_exists and static_binding and not gaps else ("CONTRACT_ONLY" if not implementation_exists or not static_binding else "DISABLED")
        readiness = "HARDWARE_ACCEPTANCE_PENDING" if status == "IMPLEMENTATION_READY" else ("DISABLED" if status == "DISABLED" else "CONTRACT_ONLY")
        maturity = "OUTPUT_ONLY" if capability and capability.verifier.type == "output_schema" else ("LIMITED" if capability else "NONE")
        evidence = [name for name, present in (("manifest", bool(entrypoint)), ("skill_md", skill_md_exists), ("entrypoint", implementation_exists), ("fixed_adapter", static_binding), ("argparse", audit_details["argparse_contract"]), ("output_json", audit_details["output_json_evidence"]), ("error_handling", audit_details["error_handling_evidence"]), ("timeout", audit_details["timeout_evidence"])) if present]
        rows.append({"operation_id": operation_id, "capability_id": capability_id, "implementation_exists": implementation_exists, "binding_verified_statically": static_binding, "underlying_api_evidence": evidence, "audit_details": audit_details, "read_only": bool(capability and not capability.moves_robot and not capability.controls_gripper and not capability.physical_side_effects), "physical_side_effects": list(capability.physical_side_effects) if capability else [], "safety_preconditions": list(capability.preconditions) if capability else ["fixed contract binding required"], "verification_evidence": [capability.verifier.type] if capability else [], "limitations": limitations, "live_readiness": readiness, "implementation_status": status, "verification_maturity": maturity, "required_acceptance_level": level, "blocking_gaps": gaps})
    return {"phase": "S10A_OFFLINE", "target_operation_count": len(rows), "reviewed_count": len(rows), "implementation_ready_count": sum(row["implementation_status"] == "IMPLEMENTATION_READY" for row in rows), "contract_only_count": sum(row["implementation_status"] == "CONTRACT_ONLY" for row in rows), "disabled_count": sum(row["implementation_status"] == "DISABLED" for row in rows), "unresolved_count": 0, "operations": rows, "real_hardware_validated_count": 0}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = audit(args.repo_root.resolve())
    payload = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(payload, encoding="utf-8")
    else:
        print(payload, end="")
    return 0 if report["reviewed_count"] == report["target_operation_count"] and report["unresolved_count"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
