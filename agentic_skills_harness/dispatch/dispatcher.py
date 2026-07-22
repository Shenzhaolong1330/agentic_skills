from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Mapping

from ..capability import CapabilityContract
from ..contracts.enums import ActionStatus, CapabilityKind, ErrorCode
from ..contracts.errors import ErrorInfo
from ..contracts.results import ActionResult, ObservationResult
from ..contracts.serialization import ContractValidationError, utc_now_iso
from ..hardware_gate import HardwareGate
from ..schema_validation import validate_json
from ..types import SkillContext, SkillMode
from ..world.invalidation import InvalidationEngine
from ..world.store import WorldStateStore
from .adapter import FixedEntrypointAdapter, UnsupportedAdapter
from .adapter_registry import AdapterNotFoundError, AdapterRegistry
from .backends import BackendResult, ExecutionBackend, FakeBackend, NoExecutionBackend, SubprocessBackend
from .error_mapping import make_error, map_legacy_text, map_structured_errors
from .models import DispatchContext, DispatchRequest, InvocationPlan
from .output_parser import OutputParseError, parse_json_output
from .path_policy import SafePathPolicy, UnsafePathError
from .trace import DispatchTraceWriter


class DispatchError(ValueError):
    pass


class CapabilityDispatcher:
    def __init__(self, capability_registry: Any, *, adapter_registry: AdapterRegistry | None = None, backend: ExecutionBackend | None = None, gate: HardwareGate | None = None, trace_writer: DispatchTraceWriter | None = None, world_state: WorldStateStore | None = None, invalidation_engine: InvalidationEngine | None = None) -> None:
        self.registry = capability_registry
        if adapter_registry is None:
            adapters = {}
            for capability in capability_registry.list(include_internal=True, include_legacy=True):
                if capability.dispatch_support != "unsupported":
                    adapters[capability.capability_id] = FixedEntrypointAdapter(capability_registry.repo_root)
                else:
                    adapters[capability.capability_id] = UnsupportedAdapter()
            adapter_registry = AdapterRegistry(capability_registry, adapters)
        self.adapters = adapter_registry
        self.backend = backend or SubprocessBackend()
        self.gate = gate or HardwareGate({})
        self.trace_writer = trace_writer
        self.invalidation_engine = invalidation_engine or (InvalidationEngine(world_state) if world_state is not None else None)

    @staticmethod
    def _context(context: DispatchContext | SkillContext | Mapping[str, Any] | None) -> DispatchContext:
        if context is None:
            return DispatchContext()
        if isinstance(context, DispatchContext):
            return context
        if isinstance(context, SkillContext):
            return DispatchContext.from_skill_context(context)
        return DispatchContext(**dict(context))

    @staticmethod
    def _generic_result(capability_id: str, error: ErrorInfo, *, observation: bool = False, started_at: str | None = None) -> ActionResult | ObservationResult:
        now = started_at or utc_now_iso()
        if observation:
            return ObservationResult(capability_id=capability_id or "unknown", ok=False, observations={}, captured_at=now, freshness_ttl_s=None, source="dispatcher", errors=(error,), warnings=(), artifacts={})
        status = ActionStatus.DENIED if error.code in {ErrorCode.AUTHORIZATION_DENIED, ErrorCode.CAPABILITY_NOT_FOUND, ErrorCode.CAPABILITY_UNSUPPORTED, ErrorCode.INVALID_INPUT, ErrorCode.SCHEMA_VALIDATION_FAILED} else ActionStatus.FAILED
        return ActionResult(capability_id=capability_id or "unknown", status=status, command_accepted=False, command_executed=False, planned_only=False, controller_target_reached=None, effect_observed=None, goal_verified=False, verification=None, outputs={}, errors=(error,), warnings=(), metrics={}, artifacts={}, started_at=now, ended_at=now)

    def _trace(self, request: DispatchRequest, capability: CapabilityContract | None, context: DispatchContext, plan: InvocationPlan | None, backend_called: bool, result: Any, started_at: str, ended_at: str) -> None:
        if not self.trace_writer:
            return
        event = {"request": {"request_id": request.request_id, "capability_id": request.capability_id, "argument_keys": sorted(request.arguments), "artifact_ref_count": len(request.artifact_refs)}, "capability": None if capability is None else {"capability_id": capability.capability_id, "version": capability.capability_version}, "mode": context.mode.value, "gate_decision": None if plan is None else dict(plan.gate_decision), "adapter_id": None if plan is None else plan.adapter_id, "invocation_plan": None if plan is None else plan.to_public_dict(), "backend_called": backend_called, "returncode": getattr(result, "metrics", {}).get("returncode") if result else None, "result": result.to_dict() if hasattr(result, "to_dict") else None, "started_at": started_at, "ended_at": ended_at}
        self.trace_writer.write(event)

    def dispatch(self, request: DispatchRequest | Mapping[str, Any], context: DispatchContext | SkillContext | Mapping[str, Any] | None = None) -> ActionResult | ObservationResult:
        started_at = utc_now_iso()
        try:
            request = request if isinstance(request, DispatchRequest) else DispatchRequest.from_dict(request)
        except Exception as exc:
            error = make_error(ErrorCode.INVALID_INPUT, str(exc), source="dispatch_request")
            return self._generic_result("invalid.request", error, started_at=started_at)
        context_value = self._context(context)
        capability = self.registry.get(request.capability_id)
        if capability is None:
            result = self._generic_result(request.capability_id, make_error(ErrorCode.CAPABILITY_NOT_FOUND, "unknown capability", details={"capability_id": request.capability_id}))
            self._trace(request, None, context_value, None, False, result, started_at, utc_now_iso())
            return result
        is_observation = capability.kind == CapabilityKind.OBSERVATION
        if capability.visibility == "legacy" or capability.visibility == "internal" and not context_value.allow_internal:
            result = self._generic_result(capability.capability_id, make_error(ErrorCode.AUTHORIZATION_DENIED, "capability visibility is not exposed to this dispatcher", details={"visibility": capability.visibility}), observation=is_observation, started_at=started_at)
            self._trace(request, capability, context_value, None, False, result, started_at, utc_now_iso())
            return result
        try:
            errors = validate_json(dict(request.arguments), capability.input_schema_ref, self.registry.repo_root)
        except Exception as exc:
            errors = [{"message": str(exc), "path": []}]
        if errors:
            result = self._generic_result(capability.capability_id, make_error(ErrorCode.SCHEMA_VALIDATION_FAILED, "input schema validation failed", details={"errors": errors}), observation=is_observation, started_at=started_at)
            self._trace(request, capability, context_value, None, False, result, started_at, utc_now_iso())
            return result
        path_policy = SafePathPolicy(context_value.artifact_dir, context_value.fixture_roots)
        try:
            artifact_paths = path_policy.validate_references(request.artifact_refs)
            artifact_dir = path_policy.output_dir(context_value.artifact_dir)
        except (UnsafePathError, OSError) as exc:
            result = self._generic_result(capability.capability_id, make_error(ErrorCode.INVALID_INPUT, str(exc), details={"classification": "artifact_path_policy"}), observation=is_observation, started_at=started_at)
            self._trace(request, capability, context_value, None, False, result, started_at, utc_now_iso())
            return result
        try:
            adapter = self.adapters.require(capability.capability_id)
        except AdapterNotFoundError:
            result = self._generic_result(capability.capability_id, make_error(ErrorCode.CAPABILITY_UNSUPPORTED, "no first-party adapter is registered"), observation=is_observation, started_at=started_at)
            self._trace(request, capability, context_value, None, False, result, started_at, utc_now_iso())
            return result
        if isinstance(adapter, UnsupportedAdapter) or not adapter.supports(capability):
            result = self._generic_result(capability.capability_id, make_error(ErrorCode.CAPABILITY_UNSUPPORTED, "capability adapter is unsupported"), observation=is_observation, started_at=started_at)
            self._trace(request, capability, context_value, None, False, result, started_at, utc_now_iso())
            return result
        try:
            plan = adapter.build_plan(request, capability, context_value)
        except Exception as exc:
            result = self._generic_result(capability.capability_id, make_error(ErrorCode.INVALID_INPUT, str(exc), source="adapter"), observation=is_observation, started_at=started_at)
            self._trace(request, capability, context_value, None, False, result, started_at, utc_now_iso())
            return result
        gate = self.gate.evaluate(context_value, capability.to_dict(), recovery=context_value.recovery)
        # A non-live gate decision means “do not touch hardware”. Artifact
        # replay is still allowed to read a controlled result, so only the
        # explicit non-executing modes and manifest plan-only contract force a
        # planned result here.
        plan = replace(plan, gate_decision=gate.to_dict(), planned_only=bool(context_value.mode in (SkillMode.MOCK, SkillMode.DRY_RUN) or capability.dispatch_support == "plan_only"))
        if not gate.allowed and not gate.planned_only:
            result = self._generic_result(capability.capability_id, make_error(ErrorCode.AUTHORIZATION_DENIED, gate.reason, details=gate.to_dict()), observation=is_observation, started_at=started_at)
            self._trace(request, capability, context_value, plan, False, result, started_at, utc_now_iso())
            return result
        if plan.planned_only or context_value.mode in (SkillMode.MOCK, SkillMode.DRY_RUN):
            if is_observation:
                result = ObservationResult(capability.capability_id, False, {"planned_only": True, "invocation_plan": plan.to_public_dict()}, started_at, None, "dispatcher", (), (), {"planned_only": True})
            else:
                result = ActionResult(capability.capability_id, ActionStatus.PLANNED_ONLY, True, False, True, None, None, False, None, {"invocation_plan": plan.to_public_dict()}, (), (), {}, {"planned_only": True}, started_at, utc_now_iso())
            self._trace(request, capability, context_value, plan, False, result, started_at, utc_now_iso())
            return result
        replayed = context_value.mode.value == "from_artifacts"
        if replayed:
            if not artifact_paths:
                result = self._generic_result(capability.capability_id, make_error(ErrorCode.INVALID_INPUT, "from_artifacts requires an artifact reference"), observation=is_observation, started_at=started_at)
                self._trace(request, capability, context_value, plan, False, result, started_at, utc_now_iso())
                return result
            try:
                payload = __import__("json").loads(artifact_paths[0].read_text(encoding="utf-8"))
                if not isinstance(payload, dict):
                    raise ValueError("artifact output must be an object")
                backend_result = BackendResult(0, __import__("json").dumps(payload), "", False)
            except Exception as exc:
                result = self._generic_result(capability.capability_id, make_error(ErrorCode.SCHEMA_VALIDATION_FAILED, str(exc), source="artifact"), observation=is_observation, started_at=started_at)
                self._trace(request, capability, context_value, plan, False, result, started_at, utc_now_iso())
                return result
        else:
            if not gate.allowed:
                result = self._generic_result(capability.capability_id, make_error(ErrorCode.AUTHORIZATION_DENIED, gate.reason, details=gate.to_dict()), observation=is_observation, started_at=started_at)
                self._trace(request, capability, context_value, plan, False, result, started_at, utc_now_iso())
                return result
            if not isinstance(self.backend, (FakeBackend, NoExecutionBackend, SubprocessBackend)) and context_value.mode.value != "live":
                result = self._generic_result(capability.capability_id, make_error(ErrorCode.CAPABILITY_UNSUPPORTED, "non-live custom backends are not permitted"), observation=is_observation, started_at=started_at)
                self._trace(request, capability, context_value, plan, False, result, started_at, utc_now_iso())
                return result
            backend_result = self.backend.execute(plan, context_value)
        result = self._result_from_backend(request, capability, plan, backend_result, is_observation, started_at, executed=not replayed)
        if isinstance(result, ActionResult) and result.command_executed and self.invalidation_engine is not None:
            report = self.invalidation_engine.invalidate_for_capability(capability, result, arguments=dict(request.arguments))
            if report.invalidated_fact_ids:
                result = replace(result, artifacts={**result.artifacts, "invalidated_fact_ids": list(report.invalidated_fact_ids), "invalidation_event": report.event})
        self._trace(request, capability, context_value, plan, context_value.mode.value == "live" and context_value.mode.value != "from_artifacts", result, started_at, utc_now_iso())
        return result

    def _result_from_backend(self, request: DispatchRequest, capability: CapabilityContract, plan: InvocationPlan, backend: BackendResult, observation: bool, started_at: str, *, executed: bool = True) -> ActionResult | ObservationResult:
        try:
            payload = parse_json_output(backend.stdout)
        except OutputParseError as exc:
            error = make_error(ErrorCode.SCHEMA_VALIDATION_FAILED, str(exc), source="output_parser")
            if observation:
                return ObservationResult(capability.capability_id, False, {}, started_at, None, "capability_dispatch", (error,), (), {})
            return ActionResult(capability.capability_id, ActionStatus.FAILED, True, executed, False, None, None, False, None, {}, (error,), (), {}, {}, started_at, utc_now_iso())
        output_errors = map_structured_errors(payload)
        if not output_errors:
            output_errors = map_legacy_text(backend.stderr, backend.stdout, capability.error_codes)
        if backend.timed_out:
            code = ErrorCode.MOTION_TIMEOUT if ErrorCode.MOTION_TIMEOUT in capability.error_codes else ErrorCode.CAPABILITY_EXECUTION_FAILED
            output_errors = (make_error(code, "capability backend timed out", details={"timeout_s": plan.timeout_s}),)
        elif backend.returncode not in (None, 0):
            output_errors = output_errors or (make_error(ErrorCode.CAPABILITY_EXECUTION_FAILED, "capability backend returned a non-zero status", details={"returncode": backend.returncode}, source="backend"),)
        try:
            schema_errors = validate_json(payload, capability.output_schema_ref, self.registry.repo_root)
        except Exception as exc:
            schema_errors = [{"message": str(exc), "path": []}]
        if schema_errors:
            output_errors = output_errors + (make_error(ErrorCode.SCHEMA_VALIDATION_FAILED, "output schema validation failed", details={"errors": schema_errors}, source="output_schema"),)
        ok = backend.returncode == 0 and not backend.timed_out and not output_errors and not schema_errors
        if observation:
            captured_at = payload.get("captured_at", started_at)
            return ObservationResult(capability.capability_id, ok, payload.get("observations", payload), captured_at, payload.get("freshness_ttl_s"), "capability_dispatch", output_errors, (), {"stdout_summary": backend.stdout[:256]})
        status = ActionStatus.SUCCEEDED if ok else ActionStatus.TIMEOUT if backend.timed_out else ActionStatus.FAILED
        return ActionResult(capability.capability_id, status, True, executed, False, None, None, False, None, payload, output_errors, (), {"returncode": backend.returncode}, {}, started_at, utc_now_iso())
