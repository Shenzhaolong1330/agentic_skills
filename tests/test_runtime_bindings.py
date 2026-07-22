from __future__ import annotations

import pytest

from agentic_skills_harness.execution.bindings import RuntimeBindingResolver
from agentic_skills_harness.execution.errors import ExecutionError
from agentic_skills_harness.planning.compiler import CompiledNode
from agentic_skills_harness.world.models import EntityRef, FactStatus, WorldFact
from agentic_skills_harness.world.store import WorldStateStore


def node(bindings, arguments=None):
    return CompiledNode("n", "COMPUTE", None, None, "none", "STATIC", None, (), dict(arguments or {}), tuple(bindings), (), (), None, None, {"max_attempts": 1, "retry_on_error_codes": [], "backoff_policy": {"kind": "none"}, "parameter_adjustment_policy": None}, 1, None, None)


def test_literal_and_node_output_bindings_use_safe_json_pointer():
    resolver = RuntimeBindingResolver()
    literal = {"binding_id": "b1", "target_argument_path": "/value", "source_type": "LITERAL", "literal": 3}
    arguments, _ = resolver.resolve(node([literal]), outputs={}, world=WorldStateStore(), now="2026-01-01T00:00:00+00:00")
    assert arguments == {"value": 3}
    output = {"binding_id": "b2", "target_argument_path": "/selected", "source_type": "NODE_OUTPUT", "source_node_id": "source", "source_output_path": "/value"}
    arguments, _ = resolver.resolve(node([output]), outputs={"source": {"value": "ok"}}, world=WorldStateStore(), now="2026-01-01T00:00:00+00:00")
    assert arguments["selected"] == "ok"


def test_world_fact_binding_requires_fresh_non_invalidated_fact():
    world = WorldStateStore()
    world.add_fact(WorldFact("f", EntityRef("obj", "fixture"), "object.pose", {}, {"x": 1}, FactStatus.VERIFIED, 0.95, "base", "2026-01-01T00:00:00+00:00", None, "observe", "1.0.0", (), None, 0, {}, 100000))
    binding = {"binding_id": "b", "target_argument_path": "/pose", "source_type": "WORLD_FACT", "world_fact_selector": {"fact_id": "f", "require_verified": True, "fresh": True}}
    args, _ = RuntimeBindingResolver().resolve(node([binding]), outputs={}, world=world, now="2026-01-01T00:00:00+00:00")
    assert args["pose"] == {"x": 1}
    world.invalidate(fact_ids=("f",), reason="test")
    with pytest.raises(ExecutionError):
        RuntimeBindingResolver().resolve(node([binding]), outputs={}, world=world, now="2026-01-01T00:00:00+00:00")


@pytest.mark.parametrize("field", ["executable", "argv", "env", "backend", "adapter"])
def test_runtime_binding_rejects_execution_fields(field):
    resolver = RuntimeBindingResolver()
    binding = {"binding_id": "b", "target_argument_path": "/value", "source_type": "LITERAL", "literal": {field: "bad"}}
    with pytest.raises(ExecutionError):
        resolver.resolve(node([binding]), outputs={}, world=WorldStateStore(), now="2026-01-01T00:00:00+00:00")
