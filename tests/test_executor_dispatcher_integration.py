from __future__ import annotations

from pathlib import Path

from agentic_skills_harness.execution import GraphExecutor
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


def test_executor_uses_dispatcher_and_completes_dry_run(tmp_path):
    r = registry()
    manifest = load_manifest(r.repo_root / "skill_manifest.json")
    report = TaskGraphCompiler(r, manifest=manifest).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json"))
    result = GraphExecutor(report.compiled_graph, registry=r, manifest=manifest, artifact_dir=tmp_path).run()
    assert result.outcome.value == "PLAN_COMPLETED"
    assert result.physical_execution_performed is False
    assert result.execution_scope.value == "NONE"
    assert (tmp_path / "events.jsonl").exists()
    assert (tmp_path / "checkpoint.json").exists()


def test_successful_node_is_not_reexecuted_on_resume(tmp_path):
    r = registry()
    manifest = load_manifest(r.repo_root / "skill_manifest.json")
    report = TaskGraphCompiler(r, manifest=manifest).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json"))
    first = GraphExecutor(report.compiled_graph, registry=r, manifest=manifest, artifact_dir=tmp_path).run()
    second = GraphExecutor(report.compiled_graph, registry=r, manifest=manifest, artifact_dir=tmp_path, resume=True).run()
    assert first.node_records[0].attempt_count == second.node_records[0].attempt_count == 1
    assert second.outcome.value == "PLAN_COMPLETED"
