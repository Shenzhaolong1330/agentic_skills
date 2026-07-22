#!/usr/bin/env python3
"""Compile a GoalSpec and TaskGraph into a static, non-executable plan."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from agentic_skills_harness.registry import CapabilityRegistry


def _read(path: Path) -> dict:
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError(f"{path} must contain a JSON object")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--goal", type=Path, required=True)
    parser.add_argument("--envelope", type=Path, required=True)
    parser.add_argument("--graph", type=Path, required=True)
    parser.add_argument("--manifest", type=Path, default=Path("skill_manifest.json"))
    parser.add_argument("--output-dir", type=Path, default=Path("/tmp/agentic_skills_compiled"))
    parser.add_argument("--check", action="store_true", help="Validate and compile without writing the compiled graph.")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        manifest = load_manifest(args.manifest)
        registry = CapabilityRegistry.from_manifest(manifest, repo_root=args.manifest.resolve().parent)
        report = TaskGraphCompiler(registry, manifest=manifest, manifest_path=args.manifest).compile(_read(args.goal), _read(args.envelope), _read(args.graph))
    except Exception as exc:
        print(json.dumps({"ok": False, "executable": False, "issues": [{"code": "INVALID_INPUT", "severity": "ERROR", "path": "$", "message": str(exc)}], "compiled_graph": None}, ensure_ascii=False, indent=2), file=sys.stderr)
        return 2
    payload = report.to_dict()
    if not args.check:
        args.output_dir.mkdir(parents=True, exist_ok=True)
        (args.output_dir / "compilation_report.json").write_text(json.dumps(payload, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        if report.compiled_graph is not None:
            (args.output_dir / "compiled_task_graph.json").write_text(json.dumps(report.compiled_graph.to_dict(), ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"ok": report.ok, "executable": False, "issue_count": len(report.issues), "plan_hash": report.compiled_graph.plan_hash if report.compiled_graph else None}, ensure_ascii=False, sort_keys=True))
    return 0 if report.ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
