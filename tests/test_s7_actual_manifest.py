from __future__ import annotations

from agentic_skills_harness.execution.preflight import ExecutionPreflightValidator
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


def test_actual_manifest_and_registry_preflight_are_current():
    r = registry()
    manifest = load_manifest(r.repo_root / "skill_manifest.json")
    report = TaskGraphCompiler(r, manifest=manifest).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json"))
    result = ExecutionPreflightValidator(report.compiled_graph, registry=r, manifest=manifest, mode="dry_run").validate()
    assert result.ok
