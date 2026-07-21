#!/usr/bin/env python3
"""Move both arms to the handover transition pose, then transfer gripper hold."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
from typing import Any

DEFAULT_TRANSITION_JSON = Path(__file__).resolve().parents[3] / "config" / "transition.json"
REPO_ROOT = Path(__file__).resolve().parents[5]
DEFAULT_CLIENT_PATH = Path(
    "/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/"
    "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py"
)
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.hardware_preflight import evaluate_operation


def _load_rpc_client(client_path: Path):
    spec = importlib.util.spec_from_file_location("dual_franka_rpc_client_handover", client_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load RPC client from {client_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def active_arm(value: str) -> str:
    if value in ("left", "left_arm"):
        return "left_arm"
    if value in ("right", "right_arm"):
        return "right_arm"
    raise argparse.ArgumentTypeError("active-arm must be left/right/left_arm/right_arm")


def short_side(side: str) -> str:
    return side[:-4] if side.endswith("_arm") else side


def other_side(side: str) -> str:
    return "right_arm" if side == "left_arm" else "left_arm"


def xyz_rotvec(value: Any, name: str) -> list[float]:
    if isinstance(value, dict):
        value = value.get("xyz_rotvec")
    if not isinstance(value, list | tuple) or len(value) != 6:
        raise ValueError(f"{name} must contain xyz_rotvec with 6 numbers")
    pose = [float(item) for item in value]
    if not all(item == item and abs(item) != float("inf") for item in pose):
        raise ValueError(f"{name} contains non-finite values")
    return pose


def transition_pair(value: Any, name: str) -> tuple[list[float], list[float]]:
    if not isinstance(value, dict):
        raise ValueError(f"{name} must be a JSON object")
    left = xyz_rotvec(value.get("left_arm"), f"{name}.left_arm")
    right = xyz_rotvec(value.get("right_arm"), f"{name}.right_arm")
    return left, right


def load_transition_targets(path: Path, active_side: str) -> tuple[list[float], list[float], str]:
    data = json.loads(path.expanduser().read_text(encoding="utf-8"))
    transition_states = data.get("transition_states")
    if isinstance(transition_states, dict):
        for key in (active_side, short_side(active_side)):
            if key in transition_states:
                left, right = transition_pair(transition_states[key], f"transition_states.{key}")
                return left, right, f"transition_states.{key}"
    left, right = transition_pair(data.get("summary"), "summary")
    return left, right, "summary"


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("mock", "dry_run", "from_artifacts", "live"), default="dry_run")
    parser.add_argument("--hardware-allowed", action="store_true", help="Allow access to the real robot in live mode.")
    parser.add_argument("--active-arm", type=active_arm, required=True, help="Arm currently holding the object.")
    parser.add_argument("--transition-json", type=Path, default=DEFAULT_TRANSITION_JSON)
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--rate-hz", type=float, default=80.0)
    parser.add_argument("--max-translation-speed", type=float, default=0.08)
    parser.add_argument("--max-rotation-speed", type=float, default=0.4)
    parser.add_argument("--max-translation-step", type=float, default=0.002)
    parser.add_argument("--max-rotation-step", type=float, default=0.02)
    parser.add_argument("--settle-time-sec", type=float, default=1.0)
    parser.add_argument("--position-tolerance-m", type=float, default=0.005)
    parser.add_argument("--rotation-tolerance-rad", type=float, default=0.05)
    parser.add_argument("--max-correction-iters", type=int, default=4)
    parser.add_argument("--after-partner-close-sleep-sec", type=float, default=0.5)
    parser.add_argument("--after-active-open-sleep-sec", type=float, default=0.5)
    parser.add_argument("--no-transfer-release", action="store_true")
    parser.add_argument("--execute", action="store_true", help="Send robot and gripper commands.")
    parser.add_argument("--compact", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    active = args.active_arm
    partner = other_side(active)
    left_target, right_target, source = load_transition_targets(args.transition_json, active)

    report: dict[str, Any] = {
        "active_arm": active,
        "partner_arm": partner,
        "transition_json": str(args.transition_json.expanduser()),
        "transition_source": source,
        "left_target": left_target,
        "right_target": right_target,
        "execute": bool(args.execute),
    }
    if args.mode != "live":
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0

    _, _, decision = evaluate_operation(
        "handover",
        mode=args.mode,
        hardware_allowed=args.hardware_allowed,
        execute=args.execute,
        manifest_path=REPO_ROOT / "skill_manifest.json",
    )
    if not decision.allowed:
        print(json.dumps({"mode": args.mode, "gate_decision": decision.to_dict()}, indent=None if args.compact else 2))
        return 1

    rpc = _load_rpc_client(DEFAULT_CLIENT_PATH)
    client = rpc.DualFrankaRobotiqRpcClient(
        ip=args.server_host,
        port=args.server_port,
        timeout=args.rpc_timeout_sec,
    )
    try:
        client.step(None)
        time.sleep(0.2)
        report["move_result"] = client.dual_robot_move_to_ee_pose(
            left_target,
            right_target,
            delta=False,
            wait=False,
            smooth=True,
            rate_hz=args.rate_hz,
            max_translation_speed=args.max_translation_speed,
            max_rotation_speed=args.max_rotation_speed,
            max_translation_step=args.max_translation_step,
            max_rotation_step=args.max_rotation_step,
            settle_time_sec=args.settle_time_sec,
            position_tolerance_m=args.position_tolerance_m,
            rotation_tolerance_rad=args.rotation_tolerance_rad,
            max_correction_iters=args.max_correction_iters,
            max_steps=3000,
        )
        if not args.no_transfer_release:
            report["partner_close_result"] = client.close_gripper(partner)
            if args.after_partner_close_sleep_sec > 0:
                time.sleep(args.after_partner_close_sleep_sec)
            report["active_open_result"] = client.open_gripper(active)
            if args.after_active_open_sleep_sec > 0:
                time.sleep(args.after_active_open_sleep_sec)
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0
    finally:
        client.close()


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
