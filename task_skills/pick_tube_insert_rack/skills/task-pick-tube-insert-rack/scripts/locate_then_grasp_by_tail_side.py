#!/usr/bin/env python3
"""Choose the first arm from an object-locator result, then optionally execute grasp.

Default behavior is structured planning only. Passing --execute is required before
this wrapper calls the lower-level grasp script.
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Any

for _parent in Path(__file__).resolve().parents:
    if (_parent / "skill_manifest.json").is_file():
        sys.path.insert(0, str(_parent))
        break

from scripts.hardware_preflight import evaluate_operation

ANY_POSE_ROOT = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
DEFAULT_GRASP_SCRIPT = Path(__file__).resolve().with_name("grasp_right_arm_xyz.sh")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result-json", "--result-json-from-file", dest="result_json", type=Path, required=True)
    parser.add_argument("--result-base-frame", choices=("base", "baseright", "baseleft"), default="base")
    parser.add_argument("--right-y-sign", choices=("negative", "positive"), default="negative")
    parser.add_argument("--side-deadband-m", type=float, default=0.002)
    parser.add_argument("--grasp-arg", action="append", default=[])
    parser.add_argument("--preferred-grasp-point", default="tail_to_head_1_5")
    parser.add_argument("--fallback-grasp-point", default="bbox_center")
    parser.add_argument("--mode", choices=("mock", "dry_run", "from_artifacts", "live"), default="dry_run")
    parser.add_argument("--hardware-allowed", action="store_true", help="Allow real hardware access in live mode.")
    parser.add_argument("--execute", action="store_true", help="Actually call the lower-level grasp script.")
    parser.add_argument("--dry-run", action="store_true", help="Alias for the default plan-only behavior.")
    parser.add_argument("--compact", action="store_true")
    return parser


def point_xyz(point: Any, name: str, frame: str) -> dict[str, float]:
    if not isinstance(point, dict):
        raise ValueError(f"{name} is missing or is not an object.")
    frame_point = point if all(axis in point for axis in ("x_m", "y_m", "z_m")) else point.get(frame)
    if not isinstance(frame_point, dict):
        raise ValueError(f"{name} does not contain frame {frame!r}.")
    xyz = {axis: float(frame_point[axis]) for axis in ("x_m", "y_m", "z_m")}
    if not all(value == value and abs(value) != float("inf") for value in xyz.values()):
        raise ValueError(f"{name}.{frame} contains non-finite coordinates.")
    return xyz


def choose_arm(result: dict[str, Any], frame: str, right_y_sign: str, deadband_m: float) -> tuple[str, float, str]:
    points_base = result.get("points_base")
    if not isinstance(points_base, dict) or not points_base.get("available"):
        reason = points_base.get("reason") if isinstance(points_base, dict) else "missing points_base"
        raise ValueError(f"points_base is unavailable: {reason}")
    try:
        head = point_xyz(points_base.get("head"), "points_base.head", frame)
        tail = point_xyz(points_base.get("tail"), "points_base.tail", frame)
    except ValueError:
        orientation = result.get("orientation")
        if not isinstance(orientation, dict):
            raise
        head_px = orientation.get("head_px")
        tail_px = orientation.get("tail_px")
        if not isinstance(head_px, dict) or not isinstance(tail_px, dict):
            raise
        dx_px = float(tail_px["x"]) - float(head_px["x"])
        if abs(dx_px) < 1.0:
            raise ValueError(f"tail_px.x - head_px.x is {dx_px:.3f} px; cannot choose arm.")
        return ("right" if dx_px > 0.0 else "left"), dx_px, "image_px_x"
    dy = tail["y_m"] - head["y_m"]
    if abs(dy) < deadband_m:
        raise ValueError(f"tail_y - head_y is {dy:.6f} m, below deadband {deadband_m:.6f} m; cannot choose arm.")
    tail_is_right = dy < 0.0 if right_y_sign == "negative" else dy > 0.0
    return ("right" if tail_is_right else "left"), dy, f"{frame}_y"


def point_available(result: dict[str, Any], key: str, frame: str) -> bool:
    points_base = result.get("points_base")
    if not isinstance(points_base, dict):
        return False
    try:
        point_xyz(points_base.get(key), f"points_base.{key}", frame)
    except (KeyError, TypeError, ValueError):
        return False
    return True


def resolve_result_json(path: Path) -> Path:
    resolved = path.expanduser()
    if not resolved.is_absolute():
        resolved = ANY_POSE_ROOT / resolved
    return resolved


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    _, _, gate_decision = evaluate_operation(
        "grasp",
        mode=args.mode,
        hardware_allowed=args.hardware_allowed,
        execute=args.execute,
        manifest_path=next(parent / "skill_manifest.json" for parent in Path(__file__).resolve().parents if (parent / "skill_manifest.json").is_file()),
    )
    if not gate_decision.allowed:
        if gate_decision.planned_only:
            args.execute = False
        else:
            print(json.dumps({"ok": False, "gate_decision": gate_decision.to_dict()}, ensure_ascii=False), file=sys.stderr)
            return 1
    result_json = resolve_result_json(args.result_json)
    if not result_json.exists():
        raise FileNotFoundError(f"result.json does not exist: {result_json}")
    payload = json.loads(result_json.read_text(encoding="utf-8"))
    arm, side_value, side_source = choose_arm(payload, args.result_base_frame, args.right_y_sign, args.side_deadband_m)
    grasp_point = args.preferred_grasp_point
    grasp_args = list(args.grasp_arg)
    if not point_available(payload, grasp_point, args.result_base_frame):
        fallback = args.fallback_grasp_point
        if not point_available(payload, fallback, args.result_base_frame):
            raise ValueError(f"Neither preferred grasp point {grasp_point!r} nor fallback {fallback!r} is available.")
        grasp_point = fallback
        grasp_args.extend(["--no-result-orientation"])
    grasp_cmd = [
        str(DEFAULT_GRASP_SCRIPT),
        "--arm",
        arm,
        "--result-json",
        str(result_json),
        "--result-grasp-point",
        grasp_point,
        *grasp_args,
    ]
    report = {
        "ok": True,
        "execute": bool(args.execute and not args.dry_run),
        "selected_arm": arm,
        "opposite_arm": "left" if arm == "right" else "right",
        "side_delta": side_value,
        "side_source": side_source,
        "result_json": str(result_json),
        "result_grasp_point": grasp_point,
        "planned_command": grasp_cmd,
    }
    if not report["execute"]:
        report["planned_only"] = True
        print(json.dumps(report, indent=None if args.compact else 2, ensure_ascii=False))
        return 0
    grasp_cmd.extend(["--mode", args.mode])
    if args.hardware_allowed:
        grasp_cmd.append("--hardware-allowed")
    return subprocess.call([*grasp_cmd, "--execute"], cwd=str(DEFAULT_GRASP_SCRIPT.parent.parent))


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
