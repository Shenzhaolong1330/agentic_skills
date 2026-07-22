from __future__ import annotations

from dataclasses import replace

from agentic_skills_harness.execution.preflight import ExecutionPreflightValidator
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


def compiled():
    root = registry().repo_root
    manifest = load_manifest(root / "skill_manifest.json")
    report = TaskGraphCompiler(registry(), manifest=manifest).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json"))
    assert report.ok
    return report.compiled_graph, registry(), manifest


def test_preflight_validates_compiled_graph_and_current_registry():
    graph, r, manifest = compiled()
    result = ExecutionPreflightValidator(graph, registry=r, manifest=manifest, mode="dry_run").validate()
    assert result.ok, result.to_dict()


def test_preflight_rejects_live_and_plan_drift():
    graph, r, manifest = compiled()
    live = replace(graph, target_mode="live")
    result = ExecutionPreflightValidator(live, registry=r, manifest=manifest, mode="live").validate()
    codes = {str(item.code) for item in result.errors}
    assert "CAPABILITY_UNSUPPORTED" in codes
    assert "CHECKPOINT_MISMATCH" in codes


def test_compiled_graph_decode_roundtrip():
    graph, _, _ = compiled()
    decoded = type(graph).from_dict(graph.to_dict())
    assert decoded.to_dict() == graph.to_dict()
