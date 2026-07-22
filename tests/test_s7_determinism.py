from __future__ import annotations

from agentic_skills_harness.execution import GraphExecutor
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


def test_short_determinism_contract(tmp_path):
    r = registry()
    manifest = load_manifest(r.repo_root / "skill_manifest.json")
    report = TaskGraphCompiler(r, manifest=manifest).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json"))
    values = []
    for index in range(5):
        result = GraphExecutor(report.compiled_graph, registry=r, manifest=manifest, artifact_dir=tmp_path / str(index)).run()
        values.append((tuple(item.node_id for item in result.node_records), result.outcome.value, result.budget_usage.tool_calls))
    assert len(set(values)) == 1
