from __future__ import annotations

from dataclasses import replace

from agentic_skills_harness.contracts.results import ObservationResult
from agentic_skills_harness.execution import GraphExecutor
from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.planning.canonical import digest
from agentic_skills_harness.planning import TaskGraphCompiler
from tests.planning_helpers import example, registry


class Clock:
    def __init__(self):
        self.value = 0.0

    def __call__(self):
        return self.value


class SlowDispatcher:
    def __init__(self, clock):
        self.clock = clock
        self.calls = 0

    def dispatch(self, request, context):
        self.calls += 1
        self.clock.value += 1.0
        return ObservationResult(request.capability_id, True, {}, "2026-01-01T00:00:00Z", 10, "fake", (), (), {})


def test_compiled_node_timeout_is_enforced_before_success(tmp_path):
    r = registry()
    manifest = load_manifest(r.repo_root / "skill_manifest.json")
    report = TaskGraphCompiler(r, manifest=manifest).compile(example("observe_object.goal.json"), example("dry_run.envelope.json"), example("observe_object.graph.json"))
    original = report.compiled_graph
    node = replace(original.compiled_nodes[0], timeout_s=0.5)
    un_hashed = replace(original, compiled_nodes=(node,), plan_hash="")
    graph = replace(un_hashed, plan_hash=digest(un_hashed._hash_payload()))
    clock = Clock()
    dispatcher = SlowDispatcher(clock)
    result = GraphExecutor(graph, registry=r, manifest=manifest, artifact_dir=tmp_path, dispatcher=dispatcher, monotonic_clock=clock).run()
    assert dispatcher.calls == 1
    assert result.outcome.value == "TIMEOUT"
