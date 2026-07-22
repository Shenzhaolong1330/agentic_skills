#!/usr/bin/env python3
"""Run one previously compiled graph in an offline S7 mode."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

from agentic_skills_harness.execution import GraphExecutor
from agentic_skills_harness.execution.models import ExecutionOutcome
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning.compiler import CompiledTaskGraph
from agentic_skills_harness.registry import CapabilityRegistry


def _read_json(path: Path) -> dict:
    if path.is_symlink() or not path.is_file():
        raise ValueError("input file must be a regular non-symlink file")
    value = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(value, dict):
        raise ValueError("JSON input must be an object")
    return value


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Execute a compiled task graph offline.")
    parser.add_argument("--compiled-graph", required=True, type=Path)
    parser.add_argument("--manifest", type=Path, default=Path("skill_manifest.json"))
    parser.add_argument("--artifact-dir", type=Path, default=Path("/tmp/agentic_skills_runs"))
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--from-artifacts-root", type=Path)
    parser.add_argument("--output-json", type=str)
    parser.add_argument("--mode", choices=("mock", "dry_run", "from_artifacts"), default="dry_run")
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    try:
        manifest = load_manifest(args.manifest)
        registry = CapabilityRegistry.from_manifest(manifest, repo_root=args.manifest.resolve().parent)
        graph = CompiledTaskGraph.from_dict(_read_json(args.compiled_graph))
        executor = GraphExecutor(graph, registry=registry, manifest=manifest, mode=args.mode, artifact_dir=args.artifact_dir, from_artifacts_root=args.from_artifacts_root, resume=args.resume)
        result = executor.run()
        payload = result.to_dict()
        if args.output_json is not None:
            output = Path(args.output_json)
            if output.is_absolute() or ".." in output.parts:
                raise ValueError("--output-json must be an artifact-relative path")
            executor.artifacts.write_json(output, payload)
        print(result.to_json())
        return 0 if result.outcome in {ExecutionOutcome.GOAL_VERIFIED, ExecutionOutcome.GRAPH_COMPLETED_UNVERIFIED, ExecutionOutcome.PLAN_COMPLETED} else 2 if result.outcome == ExecutionOutcome.NEEDS_HUMAN else 1
    except Exception as exc:
        print(json.dumps({"status": "FAIL", "error": str(exc)}, ensure_ascii=False, sort_keys=True), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
