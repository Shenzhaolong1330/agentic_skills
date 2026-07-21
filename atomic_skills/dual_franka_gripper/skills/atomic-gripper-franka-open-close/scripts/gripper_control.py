#!/usr/bin/env python3
"""Open, close, initialize, or inspect one/both Robotiq grippers on dual Franka."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any

DEFAULT_CLIENT_PATH = Path(
    "/home/deepcybo/agentic_skills/atomic_skills/dual_franka_p2p/skills/"
    "atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py"
)
REPO_ROOT = Path(__file__).resolve().parents[5]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from scripts.hardware_preflight import evaluate_operation


def _load_rpc_client(client_path: Path):
    spec = importlib.util.spec_from_file_location("dual_franka_rpc_client_gripper_skill", client_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load RPC client from {client_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def side_to_arm(side: str) -> str:
    if side == "left":
        return "left_arm"
    if side == "right":
        return "right_arm"
    raise ValueError(f"unsupported side: {side}")


def gripper_state(side_obs: Any) -> dict[str, Any]:
    if not isinstance(side_obs, dict):
        return {}
    if isinstance(side_obs.get("gripper"), dict):
        return side_obs["gripper"]
    sensors = side_obs.get("sensors")
    if isinstance(sensors, dict) and isinstance(sensors.get("robotiq"), dict):
        return sensors["robotiq"]
    return {}


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("open", "close", "initialize", "status"))
    parser.add_argument("--mode", choices=("mock", "dry_run", "from_artifacts", "live"), default="dry_run")
    parser.add_argument("--hardware-allowed", action="store_true", help="Allow access to the real robot in live mode.")
    parser.add_argument("--side", choices=("left", "right", "both"), default="both")
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--execute", action="store_true", help="Send gripper command. Status never needs this.")
    parser.add_argument("--compact", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    sides = ["left", "right"] if args.side == "both" else [args.side]
    operation = "gripper-status" if args.command == "status" else "gripper"
    if args.mode != "live":
        report = {
            "mode": args.mode,
            "command": args.command,
            "side": args.side,
            "execute": False,
            "planned": [f"{args.command}_{side}" for side in sides],
        }
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0
    _, _, decision = evaluate_operation(
        operation,
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
        report: dict[str, Any] = {
            "server": f"{args.server_host}:{args.server_port}",
            "command": args.command,
            "mode": args.mode,
            "side": args.side,
            "execute": bool(args.execute),
            "results": {},
        }
        if args.command == "status":
            obs = client.get_observation()
            for side in sides:
                report["results"][side] = gripper_state(obs.get(side_to_arm(side), {}) if isinstance(obs, dict) else {})
        elif not args.execute:
            report["planned"] = [f"{args.command}_{side}" for side in sides]
        else:
            for side in sides:
                arm = side_to_arm(side)
                if args.command == "open":
                    result = client.open_gripper(arm)
                elif args.command == "close":
                    result = client.close_gripper(arm)
                else:
                    result = client.reactivate_gripper(arm)
                report["results"][side] = result
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
