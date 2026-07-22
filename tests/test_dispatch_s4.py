from __future__ import annotations

import json
from pathlib import Path

import pytest

from agentic_skills_harness.capability import CapabilityContract
from agentic_skills_harness.dispatch import AdapterRegistry, CapabilityDispatcher, DispatchContext, DispatchRequest, FakeBackend, FixedEntrypointAdapter
from agentic_skills_harness.dispatch.models import InvocationPlan
from agentic_skills_harness.dispatch.trace import DispatchTraceWriter
from agentic_skills_harness.registry import CapabilityRegistry
from agentic_skills_harness.types import SkillMode


ROOT = Path(__file__).resolve().parents[1]


def _registry(capability_id="state.reset_realsense", support="supported"):
    from agentic_skills_harness.manifest import load_manifest
    manifest = load_manifest(ROOT / "skill_manifest.json")
    original = next(item for item in CapabilityRegistry.from_manifest(manifest, repo_root=ROOT).list(include_internal=True, include_legacy=True) if item.capability_id == capability_id)
    return CapabilityRegistry([CapabilityContract(**{**original.__dict__, "dispatch_support": support})], repo_root=ROOT), original


def _dispatcher(tmp_path, support="supported", backend=None, capability_id="state.reset_realsense", trace=False):
    registry, original = _registry(capability_id, support)
    adapters = AdapterRegistry(registry)
    adapters.register(capability_id, FixedEntrypointAdapter(ROOT))
    writer = DispatchTraceWriter(tmp_path) if trace else None
    return CapabilityDispatcher(registry, adapter_registry=adapters, backend=backend or FakeBackend(), trace_writer=writer), original


def _action_output(capability_id):
    now = "2026-01-01T00:00:00Z"
    return {"capability_id": capability_id, "status": "SUCCEEDED", "command_accepted": True, "command_executed": True, "planned_only": False, "controller_target_reached": None, "effect_observed": None, "goal_verified": False, "verification": None, "outputs": {}, "errors": [], "warnings": [], "metrics": {}, "artifacts": {}, "started_at": now, "ended_at": now}


def test_dispatch_request_rejects_bindings_and_metadata_escape():
    bad = [
        {"capability_id": "x.y", "arguments": {"argv": ["rm", "-rf"]}},
        {"capability_id": "x.y", "arguments": {"executable": "/bin/sh"}},
        {"capability_id": "x.y", "arguments": {"backend": "fake"}},
        {"capability_id": "x.y", "arguments": {"env": {"A": "B"}}},
        {"capability_id": "x.y", "arguments": {"script": "x"}},
        {"capability_id": "x.y", "metadata": {"adapter_id": "unsafe"}},
        {"capability_id": "x.y", "arguments": {"value": "$(touch p)"}},
        {"capability_id": "x.y", "arguments": {"value": "a;b"}},
    ]
    for payload in bad:
        with pytest.raises(ValueError):
            DispatchRequest.from_dict(payload)


def test_live_gate_denial_does_not_call_backend(tmp_path):
    backend = FakeBackend(stdout=json.dumps(_action_output("state.reset_realsense")))
    dispatcher, _ = _dispatcher(tmp_path, backend=backend)
    result = dispatcher.dispatch(DispatchRequest("state.reset_realsense", {}), DispatchContext(SkillMode.LIVE, hardware_allowed=False, execute=True, artifact_dir=tmp_path))
    assert result.status.value == "DENIED"
    assert backend.calls == 0


def test_live_fake_backend_executes_once_but_does_not_verify_physical_effect(tmp_path):
    backend = FakeBackend(stdout=json.dumps(_action_output("state.reset_realsense")))
    dispatcher, _ = _dispatcher(tmp_path, backend=backend)
    result = dispatcher.dispatch(DispatchRequest("state.reset_realsense", {}), DispatchContext(SkillMode.LIVE, hardware_allowed=True, execute=True, artifact_dir=tmp_path))
    assert result.status.value == "SUCCEEDED"
    assert result.command_executed is True
    assert result.controller_target_reached is None
    assert result.effect_observed is None
    assert result.goal_verified is False
    assert backend.calls == 1


def test_mock_and_dry_run_never_call_backend(tmp_path):
    backend = FakeBackend(stdout=json.dumps(_action_output("state.reset_realsense")))
    dispatcher, _ = _dispatcher(tmp_path, backend=backend)
    for mode in (SkillMode.MOCK, SkillMode.DRY_RUN):
        result = dispatcher.dispatch(DispatchRequest("state.reset_realsense", {}), DispatchContext(mode, hardware_allowed=True, execute=True, artifact_dir=tmp_path))
        assert result.planned_only is True
    assert backend.calls == 0


def test_internal_capability_is_not_publicly_dispatchable(tmp_path):
    dispatcher, _ = _dispatcher(tmp_path, capability_id="internal.task.locate_object")
    result = dispatcher.dispatch(DispatchRequest("internal.task.locate_object", {}), DispatchContext(SkillMode.LIVE, hardware_allowed=True, artifact_dir=tmp_path))
    assert result.errors[0].code.value == "AUTHORIZATION_DENIED"


def test_input_and_output_schema_fail_before_success(tmp_path):
    dispatcher, _ = _dispatcher(tmp_path)
    invalid_input = dispatcher.dispatch(DispatchRequest("state.reset_realsense", {"extra": True}), DispatchContext(SkillMode.LIVE, hardware_allowed=True, execute=True, artifact_dir=tmp_path))
    assert invalid_input.errors[0].code.value == "SCHEMA_VALIDATION_FAILED"
    backend = FakeBackend(stdout="not json")
    dispatcher, _ = _dispatcher(tmp_path, backend=backend)
    invalid_output = dispatcher.dispatch(DispatchRequest("state.reset_realsense", {}), DispatchContext(SkillMode.LIVE, hardware_allowed=True, execute=True, artifact_dir=tmp_path))
    assert invalid_output.status.value == "FAILED"
    assert invalid_output.goal_verified is False


def test_path_escape_is_rejected_before_backend(tmp_path):
    backend = FakeBackend(stdout=json.dumps(_action_output("state.reset_realsense")))
    dispatcher, _ = _dispatcher(tmp_path, backend=backend)
    result = dispatcher.dispatch(DispatchRequest("state.reset_realsense", {}, artifact_refs=("../../outside.json",)), DispatchContext(SkillMode.LIVE, hardware_allowed=True, execute=True, artifact_dir=tmp_path))
    assert result.errors[0].code.value == "INVALID_INPUT"
    assert backend.calls == 0


def test_from_artifacts_reads_only_allowed_json_and_never_executes(tmp_path):
    artifact = tmp_path / "result.json"
    artifact.write_text(json.dumps(_action_output("state.reset_realsense")), encoding="utf-8")
    backend = FakeBackend(stdout=json.dumps(_action_output("state.reset_realsense")))
    dispatcher, _ = _dispatcher(tmp_path, backend=backend)
    result = dispatcher.dispatch(DispatchRequest("state.reset_realsense", {}, artifact_refs=("result.json",)), DispatchContext(SkillMode.FROM_ARTIFACTS, artifact_dir=tmp_path))
    assert result.status.value == "SUCCEEDED"
    assert result.command_executed is False
    assert result.goal_verified is False
    assert backend.calls == 0


def test_dispatch_trace_contains_public_plan_without_host_binding(tmp_path):
    backend = FakeBackend(stdout=json.dumps(_action_output("state.reset_realsense")))
    dispatcher, _ = _dispatcher(tmp_path, backend=backend, trace=True)
    dispatcher.dispatch(DispatchRequest("state.reset_realsense", {}), DispatchContext(SkillMode.LIVE, hardware_allowed=True, execute=True, artifact_dir=tmp_path))
    trace = json.loads((tmp_path / "dispatch_trace.json").read_text(encoding="utf-8"))[0]
    assert trace["backend_called"] is True
    assert "executable" not in trace["invocation_plan"]
    assert str(ROOT) not in json.dumps(trace)
