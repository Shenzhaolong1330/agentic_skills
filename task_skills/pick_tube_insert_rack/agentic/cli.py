from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[3]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from agentic_skills_harness.execution import GraphExecutor
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from agentic_skills_harness.recovery import RecoveryOrchestrator, RecoverySelector, build_first_party_recovery_policy_registry
from agentic_skills_harness.registry import CapabilityRegistry
from agentic_skills_harness.world.store import WorldStateStore

from .compatibility import legacy_output_mapper
from .definition import PICK_TUBE_INSERT_RACK
from .dispatcher import PickTubeOfflineDispatcher
from .facts import populate_success_fixture


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Run the pick_tube_insert_rack task graph offline.")
    parser.add_argument("--mode", choices=("mock", "dry_run", "from_artifacts"), default="mock")
    parser.add_argument("--artifact-dir", type=Path, default=Path("/tmp/agentic_skills_runs/pick_tube_insert_rack"))
    parser.add_argument("--from-artifacts-root", type=Path, default=None)
    parser.add_argument("--fixture", choices=("success", "empty"), default="success")
    parser.add_argument("--resume", action="store_true")
    parser.add_argument("--output-json", type=Path, default=None)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if args.mode == "from_artifacts" and args.from_artifacts_root is None:
        # The task fixtures are controlled local artifacts; no hardware or
        # subprocess is needed for the deterministic replay implementation.
        args.from_artifacts_root = args.artifact_dir
    manifest = load_manifest(ROOT / "skill_manifest.json")
    base_registry = CapabilityRegistry.from_manifest(manifest, repo_root=ROOT)
    report, task_registry, goal, envelope, graph, context = PICK_TUBE_INSERT_RACK.compile(base_registry, manifest, mode=args.mode)
    if not report.ok or report.compiled_graph is None:
        payload = {"status": "FAIL", "compilation": report.to_dict()}
        print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
        return 1
    world = WorldStateStore()
    if args.fixture == "success":
        populate_success_fixture(world, success=True)
    dispatcher = PickTubeOfflineDispatcher(fixture={"name": args.fixture})
    run_root = args.artifact_dir
    invocation = {"count": 0}
    def factory(compiled_graph, **kwargs):
        invocation["count"] += 1
        values = dict(kwargs)
        values["artifact_dir"] = run_root / f"attempt_{invocation['count']}"
        return GraphExecutor(compiled_graph, **values)
    policies = build_first_party_recovery_policy_registry(task_registry)
    compiler = TaskGraphCompiler(task_registry, manifest=manifest)
    resilient = RecoveryOrchestrator(selector=RecoverySelector(policies), compiler=compiler, executor_factory=factory, max_iterations=8)
    result = resilient.run(report.compiled_graph, executor_kwargs={"registry": task_registry, "manifest": manifest, "dispatcher": dispatcher, "mode": args.mode, "world_state": world, "from_artifacts_root": args.from_artifacts_root, "resume": args.resume}, original_envelope=envelope.to_dict(), root_goal_spec=goal.to_dict())
    payload = {**result.to_dict(), "legacy": legacy_output_mapper(result), "execution_scope": result.final_execution_result.execution_scope.value, "physical_execution_performed": False, "physical_goal_verified": False, "graph_id": result.final_execution_result.graph_id, "plan_hash": result.final_execution_result.plan_hash}
    print(json.dumps(payload, ensure_ascii=False, sort_keys=True))
    if args.output_json is not None:
        if args.output_json.is_absolute() or ".." in args.output_json.parts:
            raise ValueError("--output-json must be artifact-relative")
        result.final_execution_result.artifacts
        (args.artifact_dir / args.output_json).parent.mkdir(parents=True, exist_ok=True)
        (args.artifact_dir / args.output_json).write_text(json.dumps(payload, ensure_ascii=False, sort_keys=True) + "\n", encoding="utf-8")
    return 0 if result.outcome.value in {"GOAL_VERIFIED", "GRAPH_COMPLETED_UNVERIFIED", "PLAN_COMPLETED"} else 2 if result.outcome.value == "NEEDS_HUMAN" else 1


if __name__ == "__main__":
    raise SystemExit(main())
