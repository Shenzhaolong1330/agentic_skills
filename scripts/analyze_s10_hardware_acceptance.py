#!/usr/bin/env python3
"""Analyze an S10 acceptance artifact without contacting hardware."""

from __future__ import annotations

import argparse
import json
from pathlib import Path


def analyze(path: Path) -> dict:
    result = json.loads((path / "acceptance_result.json" if path.is_dir() else path).read_text(encoding="utf-8"))
    failed = list(result.get("failed_reasons", []))
    if result.get("real_hardware") is not True and result.get("acceptance_level") != "H0_CONFIG":
        failed.append("not_real_hardware_evidence")
    return {"passed": bool(result.get("passed")) and not failed, "capability_id": result.get("capability_id"), "acceptance_level": result.get("acceptance_level"), "failed_reasons": sorted(set(failed)), "real_hardware": bool(result.get("real_hardware")), "backend_calls": result.get("automatic_checks", {}).get("backend_calls", 0), "graph_live_enabled": False}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("artifact", type=Path)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    report = analyze(args.artifact)
    payload = json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n"
    if args.output: args.output.write_text(payload, encoding="utf-8")
    else: print(payload, end="")
    return 0 if report["passed"] else 1


if __name__ == "__main__": raise SystemExit(main())
