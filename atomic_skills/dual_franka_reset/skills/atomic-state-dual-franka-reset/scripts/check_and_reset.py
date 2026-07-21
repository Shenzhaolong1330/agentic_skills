#!/usr/bin/env python3
"""Inspect dual-Franka health and conditionally run robot-reset."""

from __future__ import annotations

import argparse
import importlib.util
import json
import math
import os
from pathlib import Path
import re
import signal
import subprocess
import sys
import time
from typing import Any, Mapping, Sequence

REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.hardware_preflight import evaluate_operation


DEFAULT_CLIENT_PATH = Path(
    "/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/"
    "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py"
)
DEFAULT_RESET_SCRIPT = Path(
    "/home/deepcybo/agentic_skills/procedure_skills/robot_reset_home/skills/"
    "procedure-robot-reset-home/scripts/run_robot_reset.sh"
)
DEFAULT_RESET_CONFIG = Path(
    os.environ.get(
        "DUAL_FRANKA_RESET_CONFIG",
        "/home/deepcybo/Le-nero/dual_arm_teleop/scripts/config/record_cfg.yaml",
    )
)
SIDES = ("left_arm", "right_arm")
RESET_BOOLEAN_KEYS = {
    "needs_reset",
    "reset_needed",
    "reset_required",
    "has_error",
    "has_errors",
    "has_fault",
    "faulted",
    "in_error",
    "error_active",
}
CURRENT_ERROR_KEYS = {"current_errors", "active_errors", "current_faults", "active_faults"}
NON_ARM_DIAGNOSTIC_SUBTREES = {"camera", "cameras", "gripper", "grippers", "robotiq", "sensors"}
DIAGNOSTIC_CONTAINER_KEYS = {
    "arm",
    "arms",
    "controller",
    "controller_state",
    "diagnostic",
    "diagnostics",
    "franka_robot_state",
    "franka_state",
    "health",
    "left",
    "left_arm",
    "right",
    "right_arm",
    "robot_state",
    "state",
    "status",
}
SAFE_MODES = {"idle", "move", "moving", "guiding"}
RESET_MODES = {"reflex"}
MANUAL_MODES = {"user_stopped", "user_stop", "stopped_by_user"}
RECOVERING_MODES = {"automatic_error_recovery", "auto_error_recovery", "recovering"}
NUMERIC_ROBOT_MODES = {
    0: "other",
    1: "idle",
    2: "move",
    3: "guiding",
    4: "reflex",
    5: "user_stopped",
    6: "automatic_error_recovery",
}


def _load_rpc_client(client_path: Path):
    spec = importlib.util.spec_from_file_location("dual_franka_rpc_client_reset_skill", client_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load RPC client from {client_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _is_finite_vector(value: Any, length: int) -> bool:
    if isinstance(value, (str, bytes, bytearray)) or not isinstance(value, Sequence):
        return False
    if len(value) < length:
        return False
    try:
        return all(math.isfinite(float(item)) for item in value[:length])
    except (TypeError, ValueError):
        return False


def _normalize_key(value: Any) -> str:
    return re.sub(r"[^a-z0-9]+", "_", str(value).strip().lower()).strip("_")


def _positive_float(value: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(f"expected a positive finite number, got {value!r}") from exc
    if not math.isfinite(parsed) or parsed <= 0:
        raise argparse.ArgumentTypeError(f"expected a positive finite number, got {value!r}")
    return parsed


def _server_host(value: str) -> str:
    parsed = str(value).strip()
    if not parsed:
        raise argparse.ArgumentTypeError("server host must not be empty")
    return parsed


def _post_check_delay(value: str) -> float:
    try:
        parsed = float(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(f"expected a finite number from 0 to 60, got {value!r}") from exc
    if not math.isfinite(parsed) or not 0 <= parsed <= 60:
        raise argparse.ArgumentTypeError(f"expected a finite number from 0 to 60, got {value!r}")
    return parsed


def _tcp_port(value: str) -> int:
    try:
        parsed = int(value)
    except (TypeError, ValueError) as exc:
        raise argparse.ArgumentTypeError(f"expected a TCP port from 1 to 65535, got {value!r}") from exc
    if not 1 <= parsed <= 65535:
        raise argparse.ArgumentTypeError(f"expected a TCP port from 1 to 65535, got {value!r}")
    return parsed


def _as_optional_bool(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    if isinstance(value, (int, float)) and value in (0, 1):
        return bool(value)
    if isinstance(value, str):
        normalized = _normalize_key(value)
        if normalized in {"1", "true", "yes", "y", "on", "active", "fault", "error"}:
            return True
        if normalized in {"0", "false", "no", "n", "off", "inactive", "ok", "healthy", "none"}:
            return False
    return None


def _current_error_analysis(
    value: Any,
    path: str,
    *,
    string_is_error_name: bool = False,
) -> tuple[list[str], list[str]]:
    if value is None:
        return [], []
    if isinstance(value, bool):
        return ([path] if value else []), []
    if isinstance(value, (int, float)):
        if not math.isfinite(float(value)):
            return [], [f"{path} is non-finite"]
        return ([path] if value != 0 else []), []
    if isinstance(value, str):
        normalized = _normalize_key(value)
        if normalized in {
            "",
            "0",
            "none",
            "null",
            "false",
            "no",
            "n",
            "off",
            "inactive",
            "disabled",
            "ok",
            "healthy",
            "no_error",
            "no_errors",
            "no_fault",
            "no_faults",
            "{}",
            "[]",
        }:
            return [], []
        if normalized in {"1", "true", "yes", "y", "on", "active", "fault", "error"}:
            return [f"{path}={value}"], []
        if string_is_error_name:
            return [f"{path}={value}"], []
        return [], [f"{path} has unrecognized string value {value!r}"]
    if isinstance(value, Mapping):
        active: list[str] = []
        unresolved: list[str] = []
        for key, item in value.items():
            item_active, item_unresolved = _current_error_analysis(item, f"{path}.{key}")
            active.extend(item_active)
            unresolved.extend(item_unresolved)
        return active, unresolved
    if isinstance(value, Sequence):
        active: list[str] = []
        unresolved: list[str] = []
        for index, item in enumerate(value):
            item_active, item_unresolved = _current_error_analysis(
                item,
                f"{path}[{index}]",
                string_is_error_name=True,
            )
            active.extend(item_active)
            unresolved.extend(item_unresolved)
        return active, unresolved
    return [], [f"{path} has unsupported value {value!r}"]


def _normalize_robot_mode(value: Any) -> str:
    if isinstance(value, bool):
        return "unknown"
    if isinstance(value, int) or isinstance(value, float) and value.is_integer():
        return NUMERIC_ROBOT_MODES.get(int(value), f"unknown_{int(value)}")
    normalized = _normalize_key(value)
    for prefix in ("robotmode_", "robot_mode_", "mode_"):
        if normalized.startswith(prefix):
            normalized = normalized[len(prefix) :]
    if "automatic" in normalized and "recovery" in normalized:
        return "automatic_error_recovery"
    if "user" in normalized and "stop" in normalized:
        return "user_stopped"
    if "reflex" in normalized:
        return "reflex"
    return normalized or "unknown"


def _extract_diagnostics(value: Any, path: str) -> dict[str, Any]:
    result: dict[str, Any] = {
        "reset_reasons": [],
        "healthy_evidence": [],
        "manual_reasons": [],
        "recovering_reasons": [],
        "unresolved_reasons": [],
        "observed": [],
    }

    def visit(item: Any, item_path: str) -> None:
        if not isinstance(item, Mapping):
            return
        for raw_key, child in item.items():
            key = _normalize_key(raw_key)
            child_path = f"{item_path}.{raw_key}"
            if key in NON_ARM_DIAGNOSTIC_SUBTREES:
                continue
            if key in {"last_motion_errors", "previous_errors", "historical_errors"}:
                result["observed"].append({"path": child_path, "ignored": "historical"})
                continue
            if key in RESET_BOOLEAN_KEYS:
                parsed = _as_optional_bool(child)
                result["observed"].append({"path": child_path, "value": child, "parsed": parsed})
                if parsed is True:
                    result["reset_reasons"].append(f"{child_path} is true")
                elif parsed is False:
                    result["healthy_evidence"].append(f"{child_path} is false")
                else:
                    result["unresolved_reasons"].append(
                        f"{child_path} has unrecognized boolean value {child!r}"
                    )
                continue
            if key in CURRENT_ERROR_KEYS:
                active, unresolved = _current_error_analysis(child, child_path)
                result["observed"].append(
                    {"path": child_path, "active": active, "unresolved": unresolved}
                )
                if active:
                    result["reset_reasons"].extend(f"active current error: {entry}" for entry in active)
                if unresolved:
                    result["unresolved_reasons"].extend(unresolved)
                if not active and not unresolved:
                    result["healthy_evidence"].append(f"{child_path} has no active entries")
                continue
            if key == "robot_mode":
                mode = _normalize_robot_mode(child)
                result["observed"].append({"path": child_path, "value": child, "mode": mode})
                if mode in RESET_MODES:
                    result["reset_reasons"].append(f"{child_path} is {mode}")
                elif mode in MANUAL_MODES:
                    result["manual_reasons"].append(f"{child_path} is {mode}")
                elif mode in RECOVERING_MODES:
                    result["recovering_reasons"].append(f"{child_path} is {mode}")
                elif mode in SAFE_MODES:
                    result["healthy_evidence"].append(f"{child_path} is {mode}")
                else:
                    result["unresolved_reasons"].append(
                        f"{child_path} has unsupported robot mode {mode!r}"
                    )
                continue
            if key in DIAGNOSTIC_CONTAINER_KEYS:
                visit(child, child_path)

    visit(value, path)
    return result


def _validate_side(side_value: Any, side: str) -> dict[str, Any]:
    problems: list[str] = []
    if not isinstance(side_value, Mapping):
        return {"valid_telemetry": False, "problems": [f"{side} is missing or not an object"]}

    robot_state = side_value.get("robot_state", side_value)
    if not isinstance(robot_state, Mapping):
        return {"valid_telemetry": False, "problems": [f"{side}.robot_state is not an object"]}

    if not _is_finite_vector(robot_state.get("joint_positions"), 7):
        problems.append(f"{side}.robot_state.joint_positions must contain 7 finite values")

    if _is_finite_vector(side_value.get("end_pose"), 6) or _is_finite_vector(robot_state.get("end_pose"), 6):
        pose_valid = True
    else:
        eef_pose = robot_state.get("eef_pose")
        pose_valid = isinstance(eef_pose, Mapping) and _is_finite_vector(eef_pose.get("position"), 3)
        orientation_valid = isinstance(eef_pose, Mapping) and _is_finite_vector(
            eef_pose.get("orientation_xyzw"), 4
        )
        if orientation_valid:
            orientation = [float(item) for item in eef_pose["orientation_xyzw"][:4]]
            orientation_valid = math.sqrt(sum(item * item for item in orientation)) > 1e-9
        pose_valid = bool(pose_valid and orientation_valid)
    if not pose_valid:
        problems.append(f"{side} must contain a finite end_pose or eef_pose")

    if "joint_velocities" in robot_state and not _is_finite_vector(robot_state.get("joint_velocities"), 7):
        problems.append(f"{side}.robot_state.joint_velocities is present but invalid")

    return {"valid_telemetry": not problems, "problems": problems}


def evaluate_state(state: Any, *, allow_telemetry_only: bool = False) -> dict[str, Any]:
    if not isinstance(state, Mapping):
        return {
            "classification": "unknown",
            "needs_reset": None,
            "auto_reset_allowed": False,
            "reasons": [f"state payload is not an object: {type(state).__name__}"],
            "reset_reasons": [],
            "diagnostics_complete": False,
            "diagnostic_coverage": "none",
            "arms": {},
            "global_diagnostics": {},
        }

    arms: dict[str, Any] = {}
    all_reset_reasons: list[str] = []
    all_manual_reasons: list[str] = []
    all_recovering_reasons: list[str] = []
    all_unresolved_reasons: list[str] = []
    side_health_evidence = True
    side_diagnostics_complete = True
    for side in SIDES:
        side_value = state.get(side)
        arm = _validate_side(side_value, side)
        diagnostics = _extract_diagnostics(side_value, side)
        arm["diagnostics"] = diagnostics
        arm["diagnostics_complete"] = any(
            "ignored" not in observed for observed in diagnostics["observed"]
        )
        arms[side] = arm
        all_reset_reasons.extend(diagnostics["reset_reasons"])
        all_manual_reasons.extend(diagnostics["manual_reasons"])
        all_recovering_reasons.extend(diagnostics["recovering_reasons"])
        all_unresolved_reasons.extend(diagnostics["unresolved_reasons"])
        side_health_evidence = side_health_evidence and bool(
            diagnostics["healthy_evidence"] and not diagnostics["unresolved_reasons"]
        )
        side_diagnostics_complete = side_diagnostics_complete and arm["diagnostics_complete"]

    global_payload = {key: value for key, value in state.items() if key not in SIDES}
    global_diagnostics = _extract_diagnostics(global_payload, "state")
    all_reset_reasons.extend(global_diagnostics["reset_reasons"])
    all_manual_reasons.extend(global_diagnostics["manual_reasons"])
    all_recovering_reasons.extend(global_diagnostics["recovering_reasons"])
    all_unresolved_reasons.extend(global_diagnostics["unresolved_reasons"])
    telemetry_valid = all(arm["valid_telemetry"] for arm in arms.values())
    explicit_health = side_health_evidence

    observations = [
        observed
        for arm in arms.values()
        for observed in arm["diagnostics"]["observed"]
    ] + global_diagnostics["observed"]
    actionable_observations = [observed for observed in observations if "ignored" not in observed]

    if all_manual_reasons:
        classification = "manual_intervention"
        needs_reset: bool | None = bool(all_reset_reasons)
        reasons = list(
            dict.fromkeys([*all_manual_reasons, *all_recovering_reasons, *all_reset_reasons])
        )
    elif all_recovering_reasons:
        classification = "recovering"
        needs_reset = bool(all_reset_reasons)
        reasons = list(dict.fromkeys([*all_recovering_reasons, *all_reset_reasons]))
    elif all_reset_reasons:
        classification = "needs_reset"
        needs_reset = True
        reasons = list(dict.fromkeys(all_reset_reasons))
    elif telemetry_valid and explicit_health and not all_unresolved_reasons:
        classification = "healthy"
        needs_reset = False
        reasons = ["both arms have valid telemetry and explicit healthy diagnostic evidence"]
    elif telemetry_valid and allow_telemetry_only and not actionable_observations:
        classification = "healthy"
        needs_reset = False
        reasons = [
            "both arms have valid telemetry; native fault diagnostics were not available",
            "telemetry-only mode cannot rule out a Franka controller fault",
        ]
    else:
        classification = "unknown"
        needs_reset = None
        reasons = []
        for arm in arms.values():
            reasons.extend(arm["problems"])
        reasons.extend(all_unresolved_reasons)
        if telemetry_valid and not explicit_health:
            reasons.append("no current_errors, robot_mode, or equivalent reset diagnostic was found")

    if actionable_observations:
        coverage = "native_or_explicit"
    elif observations:
        coverage = "historical_only"
    elif telemetry_valid:
        coverage = "telemetry_only"
    else:
        coverage = "none"
    return {
        "classification": classification,
        "needs_reset": needs_reset,
        "auto_reset_allowed": (
            classification == "needs_reset"
            and telemetry_valid
            and side_diagnostics_complete
            and not all_unresolved_reasons
        ),
        "reasons": list(dict.fromkeys(reasons)),
        "reset_reasons": list(dict.fromkeys(all_reset_reasons)),
        "unresolved_reasons": list(dict.fromkeys(all_unresolved_reasons)),
        "diagnostics_complete": side_diagnostics_complete,
        "diagnostic_coverage": coverage,
        "arms": arms,
        "global_diagnostics": global_diagnostics,
    }


def _unwrap_state_file(value: Any) -> Any:
    if not isinstance(value, Mapping):
        return value
    if not any(side in value for side in SIDES):
        for key in ("observation", "state", "status"):
            candidate = value.get(key)
            if isinstance(candidate, Mapping) and any(side in candidate for side in SIDES):
                return candidate
    return value


def _read_live_state(args: argparse.Namespace) -> tuple[Any, Any]:
    rpc = _load_rpc_client(DEFAULT_CLIENT_PATH)
    client = rpc.DualFrankaRobotiqRpcClient(
        ip=args.server_host,
        port=args.server_port,
        timeout=args.rpc_timeout_sec,
    )
    try:
        ping = client.ping()
        state = client.get_full_state()
    finally:
        client.close()
    if isinstance(ping, Mapping) and ping.get("ok") is False:
        raise RuntimeError(f"RPC ping reported failure: {ping}")
    return ping, state


def _read_status(args: argparse.Namespace) -> tuple[dict[str, Any], Any, Any]:
    if args.state_file is not None:
        raw = json.loads(args.state_file.expanduser().read_text(encoding="utf-8"))
        state = _unwrap_state_file(raw)
        ping = None
    else:
        ping, state = _read_live_state(args)
    status = evaluate_state(state, allow_telemetry_only=args.allow_telemetry_only)
    return status, ping, state


def _build_reset_command(args: argparse.Namespace) -> list[str]:
    reset_script = DEFAULT_RESET_SCRIPT.expanduser()
    reset_config = args.reset_config.expanduser()
    reset_repo_root = args.reset_repo_root.expanduser()
    if not reset_script.is_file():
        raise FileNotFoundError(f"reset script not found: {reset_script}")
    if not reset_config.is_file():
        raise FileNotFoundError(f"reset config not found: {reset_config}")
    if not reset_repo_root.is_dir():
        raise FileNotFoundError(f"reset repo root not found: {reset_repo_root}")
    target_host, target_port = _reset_target_from_config(reset_config, reset_repo_root)
    if target_host != args.server_host or target_port != args.server_port:
        raise RuntimeError(
            "status/reset target mismatch: "
            f"status used {args.server_host}:{args.server_port}, but robot-reset config selects "
            f"{target_host}:{target_port}"
        )
    return [
        "bash",
        str(reset_script),
        "--repo-root",
        str(reset_repo_root),
        "--config",
        str(reset_config),
    ]


def _load_yaml_mapping(path: Path) -> Mapping[str, Any]:
    try:
        import yaml
    except ImportError as exc:
        raise RuntimeError("PyYAML is required to verify the robot-reset target") from exc
    loaded = yaml.safe_load(path.read_text(encoding="utf-8"))
    if not isinstance(loaded, Mapping):
        raise ValueError(f"YAML root must be an object: {path}")
    return loaded


def _reset_target_from_config(reset_config: Path, reset_repo_root: Path) -> tuple[str, int]:
    loaded = _load_yaml_mapping(reset_config)
    record = loaded.get("record")
    if not isinstance(record, Mapping):
        raise ValueError(f"reset config must contain a record object: {reset_config}")
    robot_type = str(record.get("robot_type", "dobot_dual_arm"))
    if robot_type != "franka_dual_arm":
        raise ValueError(f"reset config selects {robot_type!r}, not dual Franka")

    robot_config = record.get("robot")
    if not isinstance(robot_config, Mapping) or not robot_config:
        detail_path = reset_repo_root / "scripts" / "config" / "robots" / "franka_config.yaml"
        detail = _load_yaml_mapping(detail_path)
        detail_record = detail.get("record", detail)
        if not isinstance(detail_record, Mapping) or not isinstance(detail_record.get("robot"), Mapping):
            raise ValueError(f"Franka detail config must contain a robot object: {detail_path}")
        robot_config = detail_record["robot"]

    host = str(robot_config.get("robot_ip", "127.0.0.1")).strip()
    if not host:
        raise ValueError("robot-reset target robot_ip is empty")
    port = _tcp_port(str(robot_config.get("robot_port", 4242)))
    return host, port


def _terminate_process_group(process: subprocess.Popen[str]) -> tuple[str | None, str | None]:
    if process.poll() is None:
        try:
            os.killpg(process.pid, signal.SIGTERM)
        except ProcessLookupError:
            pass
    try:
        return process.communicate(timeout=5.0)
    except subprocess.TimeoutExpired:
        try:
            os.killpg(process.pid, signal.SIGKILL)
        except ProcessLookupError:
            pass
        return process.communicate()


def _run_reset(command: list[str], timeout_sec: float) -> dict[str, Any]:
    if not math.isfinite(timeout_sec) or timeout_sec <= 0:
        return {
            "attempted": False,
            "command": command,
            "returncode": None,
            "stdout": None,
            "stderr": None,
            "ok": False,
            "error": f"reset timeout must be positive and finite, got {timeout_sec!r}",
        }
    process: subprocess.Popen[str] | None = None
    try:
        process = subprocess.Popen(
            command,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            text=True,
            start_new_session=True,
        )
        stdout, stderr = process.communicate(timeout=timeout_sec)
        return {
            "attempted": True,
            "command": command,
            "returncode": process.returncode,
            "stdout": stdout,
            "stderr": stderr,
            "ok": process.returncode == 0,
        }
    except subprocess.TimeoutExpired:
        stdout, stderr = _terminate_process_group(process) if process is not None else (None, None)
        return {
            "attempted": True,
            "command": command,
            "returncode": None,
            "stdout": stdout,
            "stderr": stderr,
            "ok": False,
            "error": f"robot-reset timed out after {timeout_sec:g} seconds",
        }
    except Exception as exc:
        stdout, stderr = _terminate_process_group(process) if process is not None else (None, None)
        return {
            "attempted": process is not None,
            "command": command,
            "returncode": process.returncode if process is not None else None,
            "stdout": stdout,
            "stderr": stderr,
            "ok": False,
            "error": f"robot-reset process failed: {exc}",
        }
    finally:
        if process is not None and process.poll() is None:
            _terminate_process_group(process)


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("status", "ensure"))
    parser.add_argument("--mode", choices=("mock", "dry_run", "from_artifacts", "live"), default="live")
    parser.add_argument("--hardware-allowed", action="store_true", help="Allow access to the real robot in live mode.")
    parser.add_argument(
        "--server-host",
        type=_server_host,
        default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"),
    )
    parser.add_argument("--server-port", type=_tcp_port, default=os.environ.get("FRANKA_RPC_PORT", "4242"))
    parser.add_argument(
        "--rpc-timeout-sec",
        type=_positive_float,
        default=os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "10"),
    )
    parser.add_argument("--state-file", type=Path, help="Evaluate a saved JSON state instead of live RPC.")
    parser.add_argument(
        "--allow-telemetry-only",
        action="store_true",
        help="Treat complete dual-arm telemetry as healthy when native diagnostics are absent.",
    )
    parser.add_argument("--reset-config", type=Path, default=DEFAULT_RESET_CONFIG)
    parser.add_argument(
        "--reset-repo-root",
        type=Path,
        default=Path(
            os.environ.get("DUAL_FRANKA_RESET_REPO_ROOT", "/home/deepcybo/Le-nero/dual_arm_teleop")
        ),
    )
    parser.add_argument("--reset-timeout-sec", type=_positive_float, default="180")
    parser.add_argument("--post-check-delay-sec", type=_post_check_delay, default="1")
    parser.add_argument("--execute", action="store_true", help="Allow ensure to run robot-reset.")
    parser.add_argument("--include-state", action="store_true", help="Include raw state payloads in JSON output.")
    parser.add_argument("--compact", action="store_true")
    return parser


def _print_report(report: Mapping[str, Any], compact: bool) -> None:
    print(json.dumps(report, ensure_ascii=False, default=str, indent=None if compact else 2))


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    operation = "status" if args.command == "status" else "ensure"
    _, _, decision = evaluate_operation(
        operation,
        mode=args.mode,
        hardware_allowed=args.hardware_allowed,
        execute=args.execute,
        manifest_path=REPO_ROOT / "skill_manifest.json",
    )
    report: dict[str, Any] = {
        "command": args.command,
        "mode": args.mode,
        "server": None if args.state_file else f"{args.server_host}:{args.server_port}",
        "source": str(args.state_file.expanduser()) if args.state_file else "live_rpc",
        "execute": bool(args.execute),
        "status_before": None,
        "reset": {"attempted": False},
        "status_after": None,
    }

    if not decision.allowed and not (args.mode == "from_artifacts" and args.state_file is not None):
        report["gate_decision"] = decision.to_dict()
        _print_report(report, args.compact)
        return 0 if decision.planned_only else 1

    if args.execute and args.command != "ensure":
        report["error"] = "--execute is valid only with the ensure command"
        _print_report(report, args.compact)
        return 1
    if args.execute and args.state_file is not None:
        report["error"] = "refusing real reset from an offline state file; use live RPC for verification"
        _print_report(report, args.compact)
        return 1
    if args.command == "ensure" and args.allow_telemetry_only:
        report["error"] = "--allow-telemetry-only is valid only with status, never with conditional reset"
        _print_report(report, args.compact)
        return 1

    try:
        status_before, ping_before, state_before = _read_status(args)
    except Exception as exc:
        report["status_before"] = {
            "classification": "unknown",
            "needs_reset": None,
            "reasons": [str(exc)],
            "diagnostic_coverage": "none",
            "arms": {},
        }
        report["error"] = f"status read failed: {exc}"
        _print_report(report, args.compact)
        return 1

    report["ping_before"] = ping_before
    report["status_before"] = status_before
    if args.include_state:
        report["state_before"] = state_before

    classification = status_before["classification"]
    if args.command == "status":
        _print_report(report, args.compact)
        return 0 if classification == "healthy" else 2 if classification == "needs_reset" else 3

    if classification == "healthy":
        report["reset"]["skipped"] = "reset not needed"
        _print_report(report, args.compact)
        return 0
    if classification != "needs_reset":
        report["reset"]["skipped"] = f"status is {classification}; refusing automatic robot-reset"
        _print_report(report, args.compact)
        return 3
    if not status_before.get("auto_reset_allowed", False):
        report["reset"]["skipped"] = (
            "fault evidence exists, but complete dual-arm telemetry and diagnostics are unavailable"
        )
        _print_report(report, args.compact)
        return 3

    try:
        reset_command = _build_reset_command(args)
    except Exception as exc:
        report["reset"] = {"attempted": False, "error": str(exc)}
        _print_report(report, args.compact)
        return 1
    if not args.execute:
        report["reset"] = {"attempted": False, "planned": True, "command": reset_command}
        _print_report(report, args.compact)
        return 2

    report["reset"] = _run_reset(reset_command, args.reset_timeout_sec)
    if args.post_check_delay_sec > 0:
        time.sleep(args.post_check_delay_sec)
    try:
        status_after, ping_after, state_after = _read_status(args)
        report["ping_after"] = ping_after
        report["status_after"] = status_after
        if args.include_state:
            report["state_after"] = state_after
    except Exception as exc:
        report["status_after"] = {
            "classification": "unknown",
            "needs_reset": None,
            "reasons": [str(exc)],
        }
        report["post_check_error"] = str(exc)
        _print_report(report, args.compact)
        return 1

    ok = bool(report["reset"].get("ok")) and status_after["classification"] == "healthy"
    if not ok and status_after["classification"] == "unknown":
        report["verification_note"] = (
            "robot-reset returned, but RPC still lacks enough native diagnostics to prove the fault cleared"
        )
    _print_report(report, args.compact)
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
