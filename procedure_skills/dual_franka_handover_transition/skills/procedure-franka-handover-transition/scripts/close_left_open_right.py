#!/usr/bin/env python3
"""Close the left gripper, then open the right gripper on dual Franka."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
import time
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
    spec = importlib.util.spec_from_file_location("dual_franka_rpc_client_close_left_open_right", client_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load RPC client from {client_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def gripper_state(side_obs: Any) -> dict[str, Any]:
    if not isinstance(side_obs, dict):
        return {}
    if isinstance(side_obs.get("gripper"), dict):
        return side_obs["gripper"]
    sensors = side_obs.get("sensors")
    if isinstance(sensors, dict) and isinstance(sensors.get("robotiq"), dict):
        return sensors["robotiq"]
    return {}


def gripper_states_from_observation(obs: Any) -> dict[str, dict[str, Any]]:
    if not isinstance(obs, dict):
        return {"left": {}, "right": {}}
    return {
        "left": gripper_state(obs.get("left_arm")),
        "right": gripper_state(obs.get("right_arm")),
    }


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mode", choices=("mock", "dry_run", "from_artifacts", "live"), default="dry_run")
    parser.add_argument("--hardware-allowed", action="store_true", help="Allow access to the real robot in live mode.")
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--between-sleep-sec", type=float, default=0.5)
    parser.add_argument("--execute", action="store_true", help="Send gripper commands.")
    parser.add_argument("--compact", action="store_true")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.between_sleep_sec < 0:
        raise ValueError("--between-sleep-sec must be non-negative")

    if args.mode != "live":
        print(json.dumps({"mode": args.mode, "execute": False, "planned": ["close_left_gripper", "open_right_gripper"]}, indent=None if args.compact else 2))
        return 0
    _, _, decision = evaluate_operation(
        "gripper",
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
            "sequence": ["close_left_gripper", "open_right_gripper"],
            "between_sleep_sec": args.between_sleep_sec,
            "execute": bool(args.execute),
            "results": {},
        }

        report["before"] = gripper_states_from_observation(client.get_observation())
        if not args.execute:
            report["planned"] = list(report["sequence"])
        else:
            report["results"]["close_left"] = client.close_gripper("left_arm")
            if args.between_sleep_sec > 0:
                time.sleep(args.between_sleep_sec)
            report["results"]["open_right"] = client.open_gripper("right_arm")
            report["after"] = gripper_states_from_observation(client.get_observation())

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
