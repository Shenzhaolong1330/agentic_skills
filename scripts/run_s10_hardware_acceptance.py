#!/usr/bin/env python3
"""Run or validate the fixed S10H acceptance sequence.

The default and ``--fake`` paths are hardware-free.  Physical subcommands are
operator-only and fail before any adapter/backend is touched when run from CI,
pytest, automation, or a non-interactive stdin.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import ipaddress
import json
import os
from pathlib import Path
import re
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import yaml

from agentic_skills_harness.live.acceptance import AcceptanceLevel, default_acceptance_plan
from agentic_skills_harness.live.fingerprint import HardwareFingerprint, digest_json
from agentic_skills_harness.live.acceptance import write_json
from agentic_skills_harness.safety.motion_limits import MotionLimits
from agentic_skills_harness.safety.workspace import WorkspaceConfig


FORBIDDEN = {"executable", "argv", "env", "cwd", "adapter", "backend", "script", "reset_script", "client_path", "config_path", "result_path", "hardware_allowed", "execute", "pose", "workspace_override", "speed_override", "force_override", "acceptance_state", "validated"}
SAFE_CONFIG_DIRS = ("config/templates", "config/local")
ARTIFACT_ROOT = Path("/tmp/agentic_skills_gen_agent/S10")
COMMAND_LEVEL = {
    "read-only": "H1_READ_ONLY",
    "gripper-empty": "H2_GRIPPER_EMPTY",
    "motion-p2p": "H3_MOTION_P2P",
    "motion-relative": "H4_MOTION_RELATIVE",
    "safe-stop": "H5_STOP",
    "grasp-verification": "H6_GRASP_VERIFICATION",
    "fault-recovery": "H7_RECOVERY",
    "reset-home": "H8_RESET_HOME",
}
PREVIOUS_LEVEL = {
    "gripper-empty": "H1_READ_ONLY",
    "motion-p2p": "H2_GRIPPER_EMPTY",
    "motion-relative": "H3_MOTION_P2P",
    "safe-stop": "H4_MOTION_RELATIVE",
    "grasp-verification": "H5_STOP",
    "fault-recovery": "H6_GRASP_VERIFICATION",
    "reset-home": "H7_RECOVERY",
}
FIXED_CONTROLLER_PATTERN = re.compile(
    r"dual_franka_robotiq_rpc_server@(?P<host>[0-9]{1,3}(?:\.[0-9]{1,3}){3}):4242"
)


def _walk(value: Any, path: str = "$config") -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            if str(key).lower() in FORBIDDEN: raise ValueError(f"forbidden config field: {path}.{key}")
            _walk(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value): _walk(item, f"{path}[{index}]")


def _config_path(repo: Path, value: str | None) -> Path:
    path = (repo / "config/local/s10_hardware_acceptance.yaml") if value is None else Path(value).expanduser().resolve()
    if value is None and not path.exists(): path = repo / "config/templates/s10_hardware_acceptance.example.yaml"
    if not path.exists(): raise ValueError("acceptance config does not exist")
    if not any(str(path).startswith(str((repo / item).resolve()) + os.sep) for item in SAFE_CONFIG_DIRS): raise ValueError("config path must be inside config/local or config/templates")
    return path


def _artifact_path(config: dict[str, Any], value: str | None) -> Path:
    root = Path(config.get("acceptance", {}).get("artifact_root", str(ARTIFACT_ROOT / "hardware"))).resolve()
    path = root if value is None else Path(value).expanduser().resolve()
    if not (str(path) == str(root) or str(path).startswith(str(root) + os.sep)): raise ValueError("artifact path escapes configured acceptance root")
    return path


def load_config(repo: Path, value: str | None) -> tuple[Path, dict[str, Any]]:
    path = _config_path(repo, value)
    data = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict) or data.get("version") != 1: raise ValueError("config version must be 1")
    _walk(data)
    if data.get("acceptance", {}).get("graph_live_enabled") is not False: raise ValueError("GraphExecutor live must remain disabled")
    MotionLimits.from_dict(data.get("limits", {}))
    WorkspaceConfig.from_dict(data.get("workspace", {}))
    return path, data


def _fixed_vendor_config_audit(config: dict[str, Any]) -> dict[str, Any]:
    hardware = config.get("hardware", {})
    if any(str(value).startswith("FILL_ME") for value in hardware.values()):
        return {"configured": False, "status": "TEMPLATE_REQUIRES_LOCAL_HARDWARE_VALUES"}
    HardwareFingerprint.from_config(hardware)
    if hardware.get("robot_type") != "franka_dual_arm":
        raise ValueError("fixed_backend_robot_type_mismatch")
    if hardware.get("gripper_type") != "dual_robotiq_2f_85":
        raise ValueError("fixed_backend_gripper_type_mismatch")
    if hardware.get("arm_ids") != ["left_arm", "right_arm"]:
        raise ValueError("fixed_backend_arm_ids_mismatch")
    protocol = hardware.get("rpc_protocol_version")
    if not isinstance(protocol, str) or not protocol.startswith("zerorpc-nero_compatible@sha256:"):
        raise ValueError("fixed_backend_rpc_protocol_mismatch")
    match = FIXED_CONTROLLER_PATTERN.fullmatch(str(hardware.get("controller_type")))
    if match is None:
        raise ValueError("fixed_backend_controller_type_mismatch")
    address = ipaddress.ip_address(match.group("host"))
    if address.version != 4 or not address.is_private:
        raise ValueError("fixed_backend_private_ipv4_required")
    expected_workspace_digest = "sha256:" + digest_json(config["workspace"])
    if hardware.get("workspace_digest") != expected_workspace_digest:
        raise ValueError("workspace_digest_mismatch")
    if config.get("acceptance", {}).get("calibration_hash") != hardware.get("calibration_hash"):
        raise ValueError("calibration_hash_mismatch")
    return {
        "configured": True,
        "status": "FIXED_VENDOR_CONFIG_VALID",
        "backend_id": "dual_franka_robotiq_zerorpc.s10.v1",
        "endpoint": "private-ipv4:4242",
        "hardware_fingerprint_digest": HardwareFingerprint.from_config(hardware).digest,
        "workspace_digest": expected_workspace_digest,
    }


def _fixed_scene_config_audit(
    repo: Path,
    config: dict[str, Any],
    locator_mode: str | None = None,
) -> dict[str, Any]:
    hardware = config.get("hardware", {})
    if any(str(value).startswith("FILL_ME") for value in hardware.values()):
        return {
            "configured": False,
            "status": "TEMPLATE_REQUIRES_LOCAL_HARDWARE_VALUES",
        }
    from agentic_skills_harness.live.scene_backend import (
        FIXED_LOCATOR_MODES,
        S10SceneBinding,
    )

    selected_mode = locator_mode or config.get("acceptance", {}).get(
        "scene_locator_mode",
        "grounded-sam",
    )
    bindings = {
        mode: S10SceneBinding.from_config(
            repo,
            config,
            locator_mode=mode,
        )
        for mode in FIXED_LOCATOR_MODES
    }
    if selected_mode not in bindings:
        raise ValueError("fixed_scene_locator_mode_invalid")
    binding = bindings[selected_mode]
    return {
        "configured": True,
        "status": "FIXED_SCENE_CONFIG_VALID",
        "selected_mode": selected_mode,
        "available_modes": {
            mode: {
                "preset": item.locator_config.name,
                "detector": item.detector_mode,
                "adapter_digest": item.adapter_digest,
            }
            for mode, item in bindings.items()
        },
        **binding.public_dict(),
    }


def _run_id() -> str:
    return datetime.now(timezone.utc).strftime("s10-%Y%m%dT%H%M%SZ")


def _write_fake_artifact(config: dict[str, Any], out: Path, level: str, capability_id: str) -> None:
    out.mkdir(parents=True, exist_ok=True)
    (out / "observations").mkdir(exist_ok=True)
    (out / "steps").mkdir(exist_ok=True)
    hardware = config.get("hardware", {})
    try:
        fingerprint = HardwareFingerprint.from_config(hardware)
        fingerprint_digest = fingerprint.digest
    except Exception:
        fingerprint_digest = "unbound-fake-hardware"
    result = {"capability_id": capability_id, "capability_version": "1.0.0", "adapter_digest": "fake-adapter", "input_schema_digest": "fake-input-schema", "output_schema_digest": "fake-output-schema", "hardware_fingerprint_digest": fingerprint_digest, "workspace_digest": str(hardware.get("workspace_digest", "unbound")), "calibration_hash": str(hardware.get("calibration_hash", "unbound")), "acceptance_level": level, "automatic_checks": {"backend_calls": 0, "real_hardware_calls": 0, "fake_backend": True}, "operator_checks": {}, "passed": False, "failed_reasons": ["offline_fake_fixture_not_hardware_validation"], "started_at": datetime.now(timezone.utc).isoformat(), "ended_at": datetime.now(timezone.utc).isoformat(), "real_hardware": False}
    write_json(out / "acceptance_result.json", result)
    write_json(out / "hardware_fingerprint.json", {"digest": fingerprint_digest, "redacted": True})
    write_json(out / "acceptance_plan.json", default_acceptance_plan().to_dict())
    (out / "events.jsonl").write_text(json.dumps({"event": "fake_fixture", "backend_calls": 0}) + "\n", encoding="utf-8")
    (out / "operator_review.md").write_text("# Operator review\n\nThis is an offline fake fixture; it is not hardware acceptance evidence.\n", encoding="utf-8")


def _previous_level_satisfied(root: Path, command: str) -> bool:
    previous = PREVIOUS_LEVEL.get(command)
    if previous is None:
        return True
    for result_path in sorted(root.glob("*/acceptance_result.json"), reverse=True):
        try:
            result = json.loads(result_path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if result.get("acceptance_level") != previous:
            continue
        if result.get("passed") is True and result.get("real_hardware") is True and not result.get("failed_reasons"):
            return True
        reasons = set(result.get("failed_reasons", ()))
        if {
            "skipped_not_validated",
            "capability_unsupported",
        }.issubset(reasons):
            return True
    return False


def _write_real_pending_artifact(
    config: dict[str, Any],
    out: Path,
    *,
    step: Any,
    backend: Any,
    scene_backend: Any,
    probe: Any,
    started_at: str,
) -> None:
    out.mkdir(parents=True, exist_ok=True)
    observations = out / "observations"
    observations.mkdir(exist_ok=True)
    (out / "steps").mkdir(exist_ok=True)
    fingerprint = HardwareFingerprint.from_config(config["hardware"])
    unsupported = probe.checks.get("status") == "CAPABILITY_UNSUPPORTED"
    failed_reasons = list(probe.limitations)
    if unsupported:
        failed_reasons.append("capability_unsupported")
    if not probe.automatic_passed:
        failed_reasons.append("automatic_checks_not_passed")
    failed_reasons.append("operator_review_required")
    scene_process_calls = (
        scene_backend.process_calls if scene_backend is not None else 0
    )
    total_backend_calls = backend.backend_calls + scene_process_calls
    real_hardware = backend.rpc_connection_count > 0 or scene_process_calls > 0
    adapter_digest = (
        digest_json(
            {
                "vendor_backend": backend.binding.adapter_digest,
                "scene_backend": scene_backend.binding.adapter_digest,
            }
        )
        if scene_backend is not None
        else backend.binding.adapter_digest
    )
    automatic_checks = dict(probe.checks)
    automatic_checks.update(
        {
            "backend_calls": total_backend_calls,
            "vendor_backend_calls": backend.backend_calls,
            "scene_process_calls": scene_process_calls,
            "rpc_connection_count": backend.rpc_connection_count,
            "real_hardware_calls": total_backend_calls,
            "automatic_passed": probe.automatic_passed,
        }
    )
    result = {
        "capability_id": step.capability_id,
        "capability_version": "1.0.0",
        "adapter_digest": adapter_digest,
        "input_schema_digest": digest_json(
            {
                "command": probe.command,
                "fixed_inputs": True,
                "config_version": config["version"],
                "scene_locator_mode": probe.checks.get("locator_mode"),
            }
        ),
        "output_schema_digest": digest_json(
            {"schema": "schemas/live/acceptance_result.schema.json", "version": 1}
        ),
        "hardware_fingerprint_digest": fingerprint.digest,
        "workspace_digest": config["hardware"]["workspace_digest"],
        "calibration_hash": config["hardware"]["calibration_hash"],
        "acceptance_level": step.level,
        "automatic_checks": automatic_checks,
        "operator_checks": {},
        "passed": False,
        "failed_reasons": sorted(set(failed_reasons)),
        "started_at": started_at,
        "ended_at": datetime.now(timezone.utc).isoformat(),
        "real_hardware": real_hardware,
    }
    write_json(out / "acceptance_result.json", result)
    write_json(
        out / "hardware_fingerprint.json",
        {"digest": fingerprint.digest, "fingerprint": fingerprint.to_dict(), "redacted": True},
    )
    write_json(out / "acceptance_plan.json", default_acceptance_plan().to_dict())
    write_json(observations / "probe_result.json", probe.to_dict())
    (out / "events.jsonl").write_text(
        json.dumps(
            {
                "event": "fixed_hardware_probe_complete",
                "command": probe.command,
                "backend_calls": total_backend_calls,
                "vendor_backend_calls": backend.backend_calls,
                "scene_process_calls": scene_process_calls,
                "rpc_connection_count": backend.rpc_connection_count,
                "automatic_passed": probe.automatic_passed,
            },
            ensure_ascii=False,
        )
        + "\n",
        encoding="utf-8",
    )
    (out / "operator_review.md").write_text(
        "\n".join(
            (
                "# Operator review",
                "",
                "This artifact is not accepted until the on-site operator reviews",
                "the automatic checks, records the required visual observations,",
                "and explicitly marks a reviewed copy as passed.",
                "",
                f"- command: `{probe.command}`",
                f"- acceptance level: `{step.level}`",
                f"- automatic checks passed: `{str(probe.automatic_passed).lower()}`",
                f"- backend calls: `{total_backend_calls}`",
                f"- scene process calls: `{scene_process_calls}`",
                f"- limitations: `{', '.join(probe.limitations) or 'none'}`",
                "",
            )
        ),
        encoding="utf-8",
    )


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", choices=("audit", "read-only", "gripper-empty", "motion-p2p", "motion-relative", "safe-stop", "grasp-verification", "fault-recovery", "reset-home", "analyze"), default="audit")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--config", type=str)
    parser.add_argument("--artifact-dir", type=str)
    parser.add_argument("--hardware-allowed", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument(
        "--scene-locator-mode",
        choices=("grounded-sam", "openrouter-vlm"),
        help="select one audited fixed locator preset for H1; no config path is accepted",
    )
    parser.add_argument("--fake", action="store_true", help="create a non-validating fake fixture without hardware")
    args = parser.parse_args(argv)
    repo = args.repo_root.resolve()
    backend = None
    scene_backend = None
    try:
        config_path, config = load_config(repo, args.config)
        plan = default_acceptance_plan()
        vendor_config = _fixed_vendor_config_audit(config)
        scene_locator_mode = args.scene_locator_mode or config.get(
            "acceptance",
            {},
        ).get("scene_locator_mode", "grounded-sam")
        scene_config = _fixed_scene_config_audit(
            repo,
            config,
            scene_locator_mode,
        )
        print(json.dumps({"status": "CONFIG_VALID", "config": str(config_path), "levels": [step.level for step in plan.steps], "graph_live_enabled": False, "vendor_binding": vendor_config, "scene_binding": scene_config}, ensure_ascii=False))
        if args.command == "audit":
            from scripts.audit_live_capability_implementation import audit
            print(json.dumps(audit(repo), ensure_ascii=False))
            return 0
        out = _artifact_path(config, args.artifact_dir) / _run_id()
        if args.fake:
            step = next(item for item in plan.steps if item.level == COMMAND_LEVEL.get(args.command, "H0_CONFIG"))
            _write_fake_artifact(config, out, step.level, step.capability_id)
            print(json.dumps({"status": "FAKE_FIXTURE_WRITTEN", "artifact": str(out), "real_hardware": False}))
            return 0
        if args.command == "analyze":
            print(json.dumps({"status": "USE_ANALYZE_SCRIPT", "artifact_root": str(out.parent)}))
            return 0
        if not args.hardware_allowed: raise PermissionError("hardware_allowed_required")
        if args.command != "read-only" and not args.execute: raise PermissionError("execute_required_for_side_effects")
        if not sys.stdin.isatty(): raise PermissionError("non_interactive_operator_required")
        if os.environ.get("CI") or os.environ.get("PYTEST_CURRENT_TEST") or os.environ.get("CODEX_AUTOMATION"):
            raise PermissionError("automation_context_rejected")
        if not config.get("operator", {}).get("operator_confirmed", False): raise PermissionError("operator_confirmation_required_in_local_config")
        artifact_root = out.parent
        if not _previous_level_satisfied(artifact_root, args.command):
            raise PermissionError(f"previous_acceptance_level_not_satisfied:{PREVIOUS_LEVEL[args.command]}")

        # Import and instantiate the one fixed backend only after every
        # authorization, automation, operator, and sequence gate has passed.
        from agentic_skills_harness.live.hardware_acceptance import HardwareProbeResult, run_fixed_hardware_probe
        from agentic_skills_harness.live.scene_backend import FixedS10SceneBackend
        from agentic_skills_harness.live.vendor_backend import FixedDualFrankaVendorBackend

        step = next(item for item in plan.steps if item.level == COMMAND_LEVEL[args.command])
        started_at = datetime.now(timezone.utc).isoformat()
        backend = FixedDualFrankaVendorBackend.from_config(repo, config)
        if args.command == "read-only":
            scene_backend = FixedS10SceneBackend.from_config(
                repo,
                config,
                locator_mode=scene_locator_mode,
            )
        try:
            probe = run_fixed_hardware_probe(
                args.command,
                backend,
                config,
                artifact_dir=out,
                scene_backend=scene_backend,
            )
        except Exception as exc:
            scene_process_calls = (
                scene_backend.process_calls if scene_backend is not None else 0
            )
            probe = HardwareProbeResult(
                args.command,
                False,
                {
                    "status": "FAILED",
                    "error_type": type(exc).__name__,
                    "backend_calls_before_failure": backend.backend_calls,
                    "scene_process_calls_before_failure": scene_process_calls,
                },
                ("probe_failed_no_automatic_retry",),
            )
            _write_real_pending_artifact(
                config,
                out,
                step=step,
                backend=backend,
                scene_backend=scene_backend,
                probe=probe,
                started_at=started_at,
            )
            raise
        _write_real_pending_artifact(
            config,
            out,
            step=step,
            backend=backend,
            scene_backend=scene_backend,
            probe=probe,
            started_at=started_at,
        )
        print(
            json.dumps(
                {
                    "status": "OPERATOR_REVIEW_REQUIRED",
                    "artifact": str(out),
                    "acceptance_level": step.level,
                    "automatic_passed": probe.automatic_passed,
                    "passed": False,
                    "backend_calls": (
                        backend.backend_calls
                        + (
                            scene_backend.process_calls
                            if scene_backend is not None
                            else 0
                        )
                    ),
                    "real_hardware": (
                        backend.rpc_connection_count > 0
                        or (
                            scene_backend is not None
                            and scene_backend.process_calls > 0
                        )
                    ),
                },
                ensure_ascii=False,
            )
        )
        return 0 if probe.automatic_passed else 2
    except Exception as exc:
        backend_calls = backend.backend_calls if backend is not None else 0
        if scene_backend is not None:
            backend_calls += scene_backend.process_calls
        print(json.dumps({"status": "REJECTED", "reason": str(exc), "backend_calls": backend_calls, "real_hardware_calls": backend_calls}, ensure_ascii=False))
        return 1
    finally:
        if backend is not None:
            backend.close()


if __name__ == "__main__": raise SystemExit(main())
