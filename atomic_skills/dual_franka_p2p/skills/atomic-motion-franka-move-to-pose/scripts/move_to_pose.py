#!/usr/bin/env python3
"""Move one or both Franka end effectors to absolute base-frame poses."""

from __future__ import annotations

import argparse
import importlib.util
import json
import os
from pathlib import Path
import sys
from typing import Any

import numpy as np

SKILL_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CLIENT_PATH = SKILL_ROOT / "dual_franka_robotiq_rpc_client.py"


def _load_rpc_client(client_path: Path):
    spec = importlib.util.spec_from_file_location("dual_franka_rpc_client_skill", client_path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f"failed to load RPC client from {client_path}")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def parse_pose(value: str) -> list[float]:
    try:
        parsed = json.loads(value)
    except json.JSONDecodeError as exc:
        raise argparse.ArgumentTypeError(f"pose must be JSON [x,y,z,rx,ry,rz]: {exc}") from exc
    pose = np.asarray(parsed, dtype=float).reshape(-1)
    if pose.size != 6 or not np.all(np.isfinite(pose)):
        raise argparse.ArgumentTypeError(f"pose must contain 6 finite numbers, got {parsed!r}")
    return pose.tolist()


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--client-path", type=Path, default=DEFAULT_CLIENT_PATH)
    parser.add_argument("--server-host", default=os.environ.get("FRANKA_RPC_HOST", "172.16.0.1"))
    parser.add_argument("--server-port", type=int, default=int(os.environ.get("FRANKA_RPC_PORT", "4242")))
    parser.add_argument("--rpc-timeout-sec", type=float, default=float(os.environ.get("FRANKA_RPC_TIMEOUT_SEC", "30")))
    parser.add_argument("--left-pose", type=parse_pose, help="Absolute left_arm xyz_rotvec in base frame.")
    parser.add_argument("--right-pose", type=parse_pose, help="Absolute right_arm xyz_rotvec in base frame.")
    parser.add_argument("--rate-hz", type=float, default=50.0)
    parser.add_argument("--max-translation-speed", type=float, default=0.04)
    parser.add_argument("--max-rotation-speed", type=float, default=0.20)
    parser.add_argument("--max-translation-step", type=float, default=0.001)
    parser.add_argument("--max-rotation-step", type=float, default=0.012)
    parser.add_argument("--settle-time-sec", type=float, default=1.0)
    parser.add_argument("--position-tolerance-m", type=float, default=0.003)
    parser.add_argument("--rotation-tolerance-rad", type=float, default=0.03)
    parser.add_argument("--max-correction-iters", type=int, default=2)
    parser.add_argument("--max-steps", type=int, default=3000)
    parser.add_argument("--execute", action="store_true", help="Send motion command. Without this, only print plan.")
    parser.add_argument("--compact", action="store_true")
    return parser


def _pose_from_side_observation(rpc_module: Any, observation: dict[str, Any], side: str) -> list[float]:
    return list(map(float, rpc_module._pose_from_side_observation(observation, side)))


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.left_pose is None and args.right_pose is None:
        raise ValueError("pass --left-pose and/or --right-pose")

    if not args.execute and args.left_pose is not None and args.right_pose is not None:
        report: dict[str, Any] = {
            "server": f"{args.server_host}:{args.server_port}",
            "execute": False,
            "left_target": args.left_pose,
            "right_target": args.right_pose,
        }
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False, default=str))
        return 0

    rpc = _load_rpc_client(args.client_path.expanduser())
    client = rpc.DualFrankaRobotiqRpcClient(
        ip=args.server_host,
        port=args.server_port,
        timeout=args.rpc_timeout_sec,
    )
    try:
        obs = client.get_observation()
        left_current = _pose_from_side_observation(rpc, obs, "left_arm")
        right_current = _pose_from_side_observation(rpc, obs, "right_arm")
        left_target = args.left_pose or left_current
        right_target = args.right_pose or right_current
        report: dict[str, Any] = {
            "server": f"{args.server_host}:{args.server_port}",
            "execute": bool(args.execute),
            "left_target": left_target,
            "right_target": right_target,
        }
        if args.execute:
            report["result"] = client.dual_robot_move_to_ee_pose(
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
                max_steps=args.max_steps,
            )
            final_obs = client.get_observation()
            report["left_final"] = _pose_from_side_observation(rpc, final_obs, "left_arm")
            report["right_final"] = _pose_from_side_observation(rpc, final_obs, "right_arm")
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
