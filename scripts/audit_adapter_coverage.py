#!/usr/bin/env python3
"""Audit fixed Adapter disposition for every manifest capability."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agentic_skills_harness.dispatch.coverage import audit_adapter_coverage
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path("."))
    parser.add_argument("--manifest", type=Path, default=Path("skill_manifest.json"))
    parser.add_argument("--output-json", type=Path, required=True)
    parser.add_argument("--output-md", type=Path, required=True)
    args = parser.parse_args(argv)
    root = args.repo_root.resolve()
    manifest_path = (root / args.manifest).resolve() if not args.manifest.is_absolute() else args.manifest.resolve()
    registry = CapabilityRegistry.from_manifest(load_manifest(manifest_path), repo_root=root)
    result = audit_adapter_coverage(registry)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_md.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(result, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    lines = ["# Adapter Coverage", "", f"- total: {result['total_capabilities']}", f"- reviewed: {result['reviewed_capabilities']}", f"- supported: {result['supported']}", f"- plan_only: {result['plan_only']}", f"- unsupported: {result['unsupported']}", f"- complete plans: {result['core_capabilities_with_plan']}", f"- hardware live supported: {result['live_hardware_supported']}", "", "| capability | visibility | adapter | disposition | mock | dry_run | from_artifacts | live |", "| --- | --- | --- | --- | --- | --- | --- | --- |"]
    for item in result["capabilities"]:
        modes = item["mode_support"]
        lines.append(f"| `{item['capability_id']}` | {item['visibility']} | `{item['adapter_id']}` | {item['dispatch_status']} | {modes['mock']} | {modes['dry_run']} | {modes['from_artifacts']} | {modes['live']} |")
    args.output_md.write_text("\n".join(lines) + "\n", encoding="utf-8")
    return 0 if result["unreviewed"] == 0 and result["live_hardware_supported"] == 0 and not result["adapter_registry_errors"] and result["duplicate_adapter_bindings"] == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
