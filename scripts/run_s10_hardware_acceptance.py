#!/usr/bin/env python3
"""Run or validate the fixed S10H acceptance sequence.

The default and ``--fake`` paths are hardware-free.  Physical subcommands are
operator-only and fail before any adapter/backend is touched when run from CI,
pytest, automation, or a non-interactive stdin.
"""

from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
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


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", nargs="?", choices=("audit", "read-only", "gripper-empty", "motion-p2p", "motion-relative", "safe-stop", "grasp-verification", "fault-recovery", "reset-home", "analyze"), default="audit")
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--config", type=str)
    parser.add_argument("--artifact-dir", type=str)
    parser.add_argument("--hardware-allowed", action="store_true")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--fake", action="store_true", help="create a non-validating fake fixture without hardware")
    args = parser.parse_args(argv)
    repo = args.repo_root.resolve()
    try:
        config_path, config = load_config(repo, args.config)
        plan = default_acceptance_plan()
        print(json.dumps({"status": "CONFIG_VALID", "config": str(config_path), "levels": [step.level for step in plan.steps], "graph_live_enabled": False}, ensure_ascii=False))
        if args.command == "audit":
            from scripts.audit_live_capability_implementation import audit
            print(json.dumps(audit(repo), ensure_ascii=False))
            return 0
        out = _artifact_path(config, args.artifact_dir) / _run_id()
        if args.fake:
            command_level = {"read-only": "H1_READ_ONLY", "gripper-empty": "H2_GRIPPER_EMPTY", "motion-p2p": "H3_MOTION_P2P", "motion-relative": "H4_MOTION_RELATIVE", "safe-stop": "H5_STOP", "grasp-verification": "H6_GRASP_VERIFICATION", "fault-recovery": "H7_RECOVERY", "reset-home": "H8_RESET_HOME"}
            step = next(item for item in plan.steps if item.level == command_level.get(args.command, "H0_CONFIG"))
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
        # The repository has no vendor runtime.  Keep the fixed runner fail
        # closed until the operator supplies the separately audited adapter.
        raise RuntimeError("operator_backend_not_installed; no hardware call was made")
    except Exception as exc:
        print(json.dumps({"status": "REJECTED", "reason": str(exc), "backend_calls": 0, "real_hardware_calls": 0}, ensure_ascii=False))
        return 1


if __name__ == "__main__": raise SystemExit(main())
