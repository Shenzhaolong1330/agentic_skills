from __future__ import annotations

from agentic_skills_harness.execution import CancellationToken, GraphExecutor
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


def test_cancelled_run_is_bounded_and_does_not_dispatch(tmp_path):
    r = registry()
    manifest = load_manifest(r.repo_root / "skill_manifest.json")
    report = TaskGraphCompiler(r, manifest=manifest).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json"))
    token = CancellationToken()
    token.cancel("test cancellation")
    result = GraphExecutor(report.compiled_graph, registry=r, manifest=manifest, artifact_dir=tmp_path, cancellation=token).run()
    assert result.outcome.value == "CANCELLED"
    assert result.budget_usage.tool_calls == 0
