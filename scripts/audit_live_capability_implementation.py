#!/usr/bin/env python3
"""Static S10 capability implementation audit; it never imports entrypoints."""

from __future__ import annotations

import argparse
import ast
import hashlib
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

VENDOR_BACKEND_PATH = Path("agentic_skills_harness/live/vendor_backend.py")
VENDOR_RUNNER_PATH = Path("scripts/run_s10_hardware_acceptance.py")
VENDOR_CLIENT_PATH = Path(
    "atomic_skills/dual_franka_p2p/skills/"
    "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py"
)
VENDOR_OPERATION_METHODS = {
    "observe.robot_state": ("read_robot_state", "get_observation"),
    "motion.move_to_pose": ("move_to_pose", "dual_robot_move_to_ee_pose"),
    "motion.move_relative": ("move_relative", "dual_robot_move_to_ee_pose"),
    "gripper.command": ("command_gripper", "command_gripper"),
    "robot.recover_fault": ("recover_fault", "recover_robot"),
    "procedure.reset_home": ("reset_home", "go_home"),
}
SCENE_BACKEND_PATH = Path("agentic_skills_harness/live/scene_backend.py")
SCENE_VIEWER_PATH = Path(
    "atomic_skills/state_realsense_viewer/skills/"
    "atomic-state-realsense-viewer/scripts/view_state_realsense.py"
)
SCENE_EXTERNAL_ROOT = Path(
    "/home/deepcybo/agentic_skills/atomic_skills/object_locator"
)
SCENE_EXTERNAL_PATHS = {
    "locator_entrypoint": SCENE_EXTERNAL_ROOT / ".venv/bin/object-locator",
    "locator_editable_link": (
        SCENE_EXTERNAL_ROOT
        / ".venv/lib/python3.10/site-packages/"
        "__editable__.realsense_vlm_object_locator-0.1.0.pth"
    ),
    "locator_grounded_sam_config": (
        SCENE_EXTERNAL_ROOT / "config_rack_center_grounded_sam.yaml"
    ),
    "locator_openrouter_vlm_config": (
        SCENE_EXTERNAL_ROOT / "config_rack_center_vlm.yaml"
    ),
    "locator_cli": SCENE_EXTERNAL_ROOT / "src/object_locator/cli.py",
    "locator_realsense": (
        SCENE_EXTERNAL_ROOT / "src/object_locator/realsense_camera.py"
    ),
    "locator_grounded_sam": (
        SCENE_EXTERNAL_ROOT / "src/object_locator/grounded_sam_detector.py"
    ),
    "locator_openrouter_vlm": (
        SCENE_EXTERNAL_ROOT / "src/object_locator/openrouter_vlm.py"
    ),
    "locator_extrinsics": SCENE_EXTERNAL_ROOT / "calibration/extrinsics.yaml",
    "all_camera_calibration": Path(
        "/home/deepcybo/Le-nero/dual_arm_teleop/"
        "calibration/all_camera_calibration_4.json"
    ),
    "head_to_base": Path(
        "/home/deepcybo/dual_arm_camera_calibration/"
        "calibration/head_eye_result/head_to_base.json"
    ),
    "realsense_python": Path("/home/deepcybo/miniconda3/bin/python"),
}


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _class_methods(source: str, class_name: str) -> set[str]:
    tree = ast.parse(source)
    for node in tree.body:
        if isinstance(node, ast.ClassDef) and node.name == class_name:
            return {
                item.name
                for item in node.body
                if isinstance(item, (ast.FunctionDef, ast.AsyncFunctionDef))
            }
    return set()


def audit_fixed_vendor_backend(repo: Path) -> dict[str, Any]:
    """Statically inspect the one S10H binding without importing vendor code."""

    backend_path = repo / VENDOR_BACKEND_PATH
    client_path = repo / VENDOR_CLIENT_PATH
    runner_path = repo / VENDOR_RUNNER_PATH
    files_exist = all(path.is_file() and not path.is_symlink() for path in (backend_path, client_path, runner_path))
    if not files_exist:
        return {
            "status": "MISSING",
            "backend_id": "dual_franka_robotiq_zerorpc.s10.v1",
            "binding_verified_statically": False,
            "real_hardware_validated": False,
            "hardware_calls_during_audit": 0,
            "blocking_gaps": ["fixed backend, bundled client, or runner is missing"],
        }

    backend_source = backend_path.read_text(encoding="utf-8")
    client_source = client_path.read_text(encoding="utf-8")
    runner_source = runner_path.read_text(encoding="utf-8")
    backend_methods = _class_methods(backend_source, "FixedDualFrankaVendorBackend")
    client_methods = _class_methods(client_source, "DualFrankaRobotiqRpcClient")
    required_backend_methods = {
        "from_config",
        "ping",
        "read_robot_state",
        "command_gripper",
        "move_to_pose",
        "move_relative",
        "safe_stop",
        "verify_grasp",
        "recover_fault",
        "reset_home",
        "close",
    }
    required_client_methods = {
        "ping",
        "get_observation",
        "command_gripper",
        "open_gripper",
        "close_gripper",
        "reactivate_gripper",
        "dual_robot_move_to_ee_pose",
        "recover_robot",
        "go_home",
        "close",
    }
    forbidden_cli_options = (
        "--backend",
        "--adapter",
        "--client-path",
        "--executable",
        "--script",
        "--reset-script",
        "--argv",
        "--env",
        "--cwd",
    )
    fixed_path_present = VENDOR_CLIENT_PATH.as_posix() in backend_source.replace(
        '"\n    "', ""
    )
    import_position = runner_source.index(
        "from agentic_skills_harness.live.vendor_backend import"
    )
    gate_markers = (
        "if not args.hardware_allowed",
        "if args.command != \"read-only\" and not args.execute",
        "if not sys.stdin.isatty()",
        "automation_context_rejected",
        "operator_confirmation_required_in_local_config",
        "previous_acceptance_level_not_satisfied",
    )
    backend_import_after_gates = all(
        runner_source.index(marker) < import_position for marker in gate_markers
    )
    support: dict[str, dict[str, Any]] = {}
    for operation_id, (backend_method, client_method) in VENDOR_OPERATION_METHODS.items():
        support[operation_id] = {
            "supported": backend_method in backend_methods and client_method in client_methods,
            "backend_method": backend_method,
            "vendor_client_method": client_method,
            "real_hardware_validated": False,
        }
    support["motion.safe_stop"] = {
        "supported": False,
        "backend_method": "safe_stop",
        "vendor_client_method": None,
        "reason": "no cancel, hold, pause, or servo-stop method",
        "real_hardware_validated": False,
    }
    support["gripper.verify_grasp"] = {
        "supported": False,
        "backend_method": "verify_grasp",
        "vendor_client_method": None,
        "reason": "no independent grasp evidence method",
        "real_hardware_validated": False,
    }
    checks = {
        "fixed_files_exist": files_exist,
        "fixed_client_path": fixed_path_present,
        "backend_methods_complete": required_backend_methods <= backend_methods,
        "vendor_client_methods_complete": required_client_methods <= client_methods,
        "no_cli_backend_selector": not any(option in runner_source for option in forbidden_cli_options),
        "backend_import_after_hardware_gate": backend_import_after_gates,
        "safe_stop_does_not_call_reset": '"reset"' not in backend_source[
            backend_source.index("def safe_stop"):backend_source.index("def verify_grasp")
        ],
        "recovery_uses_vendor_recover": '"recover_robot"' in backend_source,
        "home_uses_vendor_go_home": '"go_home"' in backend_source,
    }
    return {
        "status": "PASS_IMPLEMENTATION_READY_OFFLINE_AUDITED" if all(checks.values()) else "FAIL",
        "backend_id": "dual_franka_robotiq_zerorpc.s10.v1",
        "binding_verified_statically": all(checks.values()),
        "backend_path": VENDOR_BACKEND_PATH.as_posix(),
        "client_path": VENDOR_CLIENT_PATH.as_posix(),
        "backend_sha256": _sha256(backend_path),
        "client_sha256": _sha256(client_path),
        "checks": checks,
        "operation_support": support,
        "real_hardware_validated": False,
        "hardware_calls_during_audit": 0,
        "blocking_gaps": [
            "motion.safe_stop has no vendor cancel/hold/servo-stop API",
            "gripper.verify_grasp has no independent evidence API",
            "recovery and reset remain denied when E-stop, held-object, or error state is unknown",
        ],
    }


def audit_fixed_scene_backend(repo: Path) -> dict[str, Any]:
    """Statically inspect the fixed H1 camera/locator binding."""

    backend_path = repo / SCENE_BACKEND_PATH
    viewer_path = repo / SCENE_VIEWER_PATH
    runner_path = repo / VENDOR_RUNNER_PATH
    local_files = (backend_path, viewer_path, runner_path)
    local_files_exist = all(
        path.is_file() and not path.is_symlink() for path in local_files
    )
    external_files_exist = all(
        path.is_file() for path in SCENE_EXTERNAL_PATHS.values()
    )
    if not local_files_exist or not external_files_exist:
        return {
            "status": "MISSING",
            "backend_id": "realsense_object_locator.s10.v1",
            "binding_verified_statically": False,
            "real_hardware_validated": False,
            "hardware_calls_during_audit": 0,
            "blocking_gaps": [
                "fixed scene backend, viewer, locator, calibration, or runtime is missing"
            ],
        }

    backend_source = backend_path.read_text(encoding="utf-8")
    viewer_source = viewer_path.read_text(encoding="utf-8")
    runner_source = runner_path.read_text(encoding="utf-8")
    locator_source = SCENE_EXTERNAL_PATHS["locator_cli"].read_text(
        encoding="utf-8"
    )
    grounded_sam_config = SCENE_EXTERNAL_PATHS[
        "locator_grounded_sam_config"
    ].read_text(
        encoding="utf-8"
    )
    openrouter_vlm_config = SCENE_EXTERNAL_PATHS[
        "locator_openrouter_vlm_config"
    ].read_text(
        encoding="utf-8"
    )
    methods = _class_methods(backend_source, "FixedS10SceneBackend")
    required_methods = {
        "from_config",
        "capture_h1_observations",
        "_capture_camera_samples",
        "_locate_samples",
    }
    import_position = runner_source.index(
        "from agentic_skills_harness.live.scene_backend import FixedS10SceneBackend"
    )
    gate_markers = (
        "if not args.hardware_allowed",
        "if not sys.stdin.isatty()",
        "automation_context_rejected",
        "operator_confirmation_required_in_local_config",
        "previous_acceptance_level_not_satisfied",
    )
    import_after_gates = all(
        runner_source.index(marker) < import_position for marker in gate_markers
    )
    forbidden_patterns = (
        "shell=True",
        "os.system(",
        "--scene-backend",
        "--locator-config",
        "--locator-executable",
        "--camera-serial",
    )
    fixed_serials = (
        "348522072761",
        "347622074336",
        "337322072568",
    )
    checks = {
        "fixed_files_exist": local_files_exist and external_files_exist,
        "backend_methods_complete": required_methods <= methods,
        "fixed_camera_serials_bound": all(
            serial in backend_source and serial in viewer_source
            for serial in fixed_serials
        ),
        "fixed_grounded_sam_preset": (
            'mode: "grounded_sam"' in grounded_sam_config
            and 'name: "black test tube rack"' in grounded_sam_config
        ),
        "fixed_openrouter_vlm_preset": (
            'mode: "vlm"' in openrouter_vlm_config
            and 'model: "google/gemini-3.5-flash"' in openrouter_vlm_config
            and 'name: "black test tube rack"' in openrouter_vlm_config
        ),
        "fixed_head_calibration": (
            all(
                'active_camera: "head"' in source
                and 'file: "calibration/extrinsics.yaml"' in source
                for source in (grounded_sam_config, openrouter_vlm_config)
            )
        ),
        "fixed_locator_runtime_has_result_json": (
            "--result-json" in locator_source
            and "--history-dir" in locator_source
            and "--no-reset-realsense" in backend_source
        ),
        "fixed_locator_editable_source": (
            SCENE_EXTERNAL_PATHS["locator_editable_link"]
            .read_text(encoding="utf-8")
            .strip()
            == str(SCENE_EXTERNAL_ROOT / "src")
        ),
        "hardware_allowed_guard_in_backend": (
            "if not hardware_allowed:" in backend_source
            and "hardware_allowed_required" in backend_source
        ),
        "scene_import_after_operator_gates": import_after_gates,
        "no_scene_cli_selector_or_unsafe_execution": not any(
            pattern in backend_source or pattern in runner_source
            for pattern in forbidden_patterns
        )
        and re.search(
            r"(?<![.\w])(eval|exec)\s*\(",
            backend_source + "\n" + runner_source,
        )
        is None,
        "h1_requires_scene_backend": (
            "fixed_scene_backend_required_for_h1"
            in (repo / "agentic_skills_harness/live/hardware_acceptance.py").read_text(
                encoding="utf-8"
            )
        ),
    }
    return {
        "status": (
            "PASS_IMPLEMENTATION_READY_OFFLINE_AUDITED"
            if all(checks.values())
            else "FAIL"
        ),
        "backend_id": "realsense_object_locator.s10.v1",
        "binding_verified_statically": all(checks.values()),
        "backend_path": SCENE_BACKEND_PATH.as_posix(),
        "viewer_path": SCENE_VIEWER_PATH.as_posix(),
        "external_sources": {
            name: {
                "path": str(path),
                "sha256": _sha256(path.resolve()),
            }
            for name, path in SCENE_EXTERNAL_PATHS.items()
        },
        "backend_sha256": _sha256(backend_path),
        "viewer_sha256": _sha256(viewer_path),
        "checks": checks,
        "operation_support": {
            "observe.scene": {
                "supported": all(checks.values()),
                "backend_method": "capture_h1_observations",
                "camera_samples_per_camera": 5,
                "locator_samples": 5,
                "fixed_target": "black test tube rack",
                "available_modes": {
                    "grounded-sam": {
                        "detector": "grounded_sam",
                        "preset": "config_rack_center_grounded_sam.yaml",
                    },
                    "openrouter-vlm": {
                        "detector": "vlm",
                        "preset": "config_rack_center_vlm.yaml",
                    },
                },
                "real_hardware_validated": False,
            }
        },
        "real_hardware_validated": False,
        "hardware_calls_during_audit": 0,
        "blocking_gaps": [
            "physical camera identity, image orientation, and localization accuracy remain operator-validated at H1",
            "the fixed black test tube rack must be visible from the head camera",
        ],
    }


def audit(repo: Path) -> dict[str, Any]:
    from agentic_skills_harness.manifest import load_manifest
    from agentic_skills_harness.registry import CapabilityRegistry
    from agentic_skills_harness.dispatch.adapters.fixed import build_fixed_adapter_registry

    manifest = load_manifest(repo / "skill_manifest.json")
    registry = CapabilityRegistry.from_manifest(manifest, repo_root=repo)
    adapters = build_fixed_adapter_registry(registry)
    vendor_backend = audit_fixed_vendor_backend(repo)
    scene_backend = audit_fixed_scene_backend(repo)
    vendor_support = vendor_backend.get("operation_support", {})
    scene_support = scene_backend.get("operation_support", {})
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
        rows.append({"operation_id": operation_id, "capability_id": capability_id, "implementation_exists": implementation_exists, "binding_verified_statically": static_binding, "underlying_api_evidence": evidence, "audit_details": audit_details, "vendor_acceptance_binding": vendor_support.get(operation_id), "scene_acceptance_binding": scene_support.get(operation_id), "read_only": bool(capability and not capability.moves_robot and not capability.controls_gripper and not capability.physical_side_effects), "physical_side_effects": list(capability.physical_side_effects) if capability else [], "safety_preconditions": list(capability.preconditions) if capability else ["fixed contract binding required"], "verification_evidence": [capability.verifier.type] if capability else [], "limitations": limitations, "live_readiness": readiness, "implementation_status": status, "verification_maturity": maturity, "required_acceptance_level": level, "blocking_gaps": gaps})
    return {"phase": "S10A_OFFLINE", "target_operation_count": len(rows), "reviewed_count": len(rows), "implementation_ready_count": sum(row["implementation_status"] == "IMPLEMENTATION_READY" for row in rows), "contract_only_count": sum(row["implementation_status"] == "CONTRACT_ONLY" for row in rows), "disabled_count": sum(row["implementation_status"] == "DISABLED" for row in rows), "unresolved_count": 0, "operations": rows, "vendor_backend": vendor_backend, "scene_backend": scene_backend, "real_hardware_validated_count": 0}


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
