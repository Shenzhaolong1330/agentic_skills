from __future__ import annotations

from dataclasses import replace

import pytest

from agentic_skills_harness.execution import GraphExecutor
from agentic_skills_harness.execution.models import ExecutionOutcome
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


def test_executor_constructor_rejects_raw_graph():
    r = registry()
    manifest = load_manifest(r.repo_root / "skill_manifest.json")
    with pytest.raises(TypeError):
        GraphExecutor(example("observe_object.graph.json"), registry=r, manifest=manifest)  # type: ignore[arg-type]


def test_executor_rejects_live_compiled_graph_without_dispatch(tmp_path):
    r = registry()
    manifest = load_manifest(r.repo_root / "skill_manifest.json")
    report = TaskGraphCompiler(r, manifest=manifest).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json"))
    graph = replace(report.compiled_graph, target_mode="live")
    result = GraphExecutor(graph, registry=r, manifest=manifest, mode="live", artifact_dir=tmp_path).run()
    assert result.outcome is ExecutionOutcome.FAILED
    assert result.budget_usage.tool_calls == 0
