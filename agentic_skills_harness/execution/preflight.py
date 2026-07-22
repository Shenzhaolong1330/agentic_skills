from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any, Mapping

from ..contracts.enums import ErrorCode
from ..planning.canonical import digest
from ..planning.compiler import CompiledTaskGraph
from ..schema_validation import validate_json
from .errors import ExecutionError


@dataclass
class PreflightReport:
    errors: list[ExecutionError] = field(default_factory=list)
    warnings: list[str] = field(default_factory=list)

    @property
    def ok(self) -> bool:
        return not self.errors

    def to_dict(self) -> dict[str, Any]:
        return {"ok": self.ok, "errors": [{"code": str(item.code), "message": item.message, "details": dict(item.details)} for item in self.errors], "warnings": list(self.warnings)}


class ExecutionPreflightValidator:
    SUPPORTED_MODES = {"mock", "dry_run", "from_artifacts"}

    def __init__(self, compiled_graph: CompiledTaskGraph, *, registry: Any, manifest: Mapping[str, Any], mode: str | None = None) -> None:
        self.graph = compiled_graph
        self.registry = registry
        self.manifest = dict(manifest)
        self.mode = mode or compiled_graph.target_mode

    def validate(self) -> PreflightReport:
        report = PreflightReport()
        graph = self.graph
        if not isinstance(graph, CompiledTaskGraph):
            report.errors.append(ExecutionError(ErrorCode.INVALID_INPUT, "Executor accepts only CompiledTaskGraph"))
            return report
        if self.mode not in self.SUPPORTED_MODES:
            report.errors.append(ExecutionError(ErrorCode.CAPABILITY_UNSUPPORTED, "S7 supports only mock, dry_run and from_artifacts modes", {"mode": self.mode}))
        if graph.target_mode not in self.SUPPORTED_MODES or graph.target_mode == "live":
            report.errors.append(ExecutionError(ErrorCode.CAPABILITY_UNSUPPORTED, "live compiled graphs are rejected by S7", {"target_mode": graph.target_mode}))
        if self.mode != graph.target_mode:
            report.errors.append(ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "runtime mode must match compiled target mode", {"compiled": graph.target_mode, "runtime": self.mode}))
        try:
            schema_errors = validate_json(graph.to_dict(), "schemas/planning/compiled_task_graph.schema.json", self.registry.repo_root)
            if schema_errors:
                report.errors.append(ExecutionError(ErrorCode.SCHEMA_VALIDATION_FAILED, "compiled graph schema validation failed", {"errors": schema_errors[:16]}))
        except Exception as exc:
            report.errors.append(ExecutionError(ErrorCode.SCHEMA_VALIDATION_FAILED, f"compiled graph schema validation unavailable: {exc}"))
        try:
            expected_plan_hash = digest(graph._hash_payload())
            if graph.plan_hash != expected_plan_hash:
                report.errors.append(ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "compiled graph plan_hash mismatch", {"expected": expected_plan_hash, "actual": graph.plan_hash}))
        except Exception as exc:
            report.errors.append(ExecutionError(ErrorCode.SCHEMA_VALIDATION_FAILED, f"compiled graph hash cannot be calculated: {exc}"))
        if str(graph.manifest_version) != str(self.manifest.get("version")):
            report.errors.append(ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "manifest version drift detected", {"compiled": graph.manifest_version, "current": self.manifest.get("version")}))
        if graph.manifest_digest != digest(self.manifest):
            report.errors.append(ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "manifest digest drift detected"))
        capability_index = [{"capability_id": item.capability_id, "capability_version": item.capability_version, "input_schema_ref": item.input_schema_ref, "output_schema_ref": item.output_schema_ref} for item in self.registry.list(include_internal=True, include_legacy=True)]
        if graph.capability_index_digest != digest(capability_index):
            report.errors.append(ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "capability index digest drift detected"))
        node_ids = [node.node_id for node in graph.compiled_nodes]
        node_set = set(node_ids)
        if len(node_ids) != len(node_set):
            report.errors.append(ExecutionError(ErrorCode.SCHEMA_VALIDATION_FAILED, "compiled graph contains duplicate node IDs"))
        if not set(graph.terminal_nodes).issubset(node_set):
            report.errors.append(ExecutionError(ErrorCode.SCHEMA_VALIDATION_FAILED, "compiled graph terminal node reference is invalid"))
        for edge in graph.compiled_edges:
            if edge.source_node_id not in node_set or edge.target_node_id not in node_set:
                report.errors.append(ExecutionError(ErrorCode.SCHEMA_VALIDATION_FAILED, "compiled graph edge reference is invalid", {"edge_id": edge.edge_id}))
        for key, value in graph.budgets.items():
            if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)) or float(value) < 0:
                report.errors.append(ExecutionError(ErrorCode.SCHEMA_VALIDATION_FAILED, "compiled graph budgets must be finite non-negative numbers", {"budget": key}))
        for node in graph.compiled_nodes:
            if node.metadata.get("requires_concurrent") is True or node.metadata.get("requires_concurrency") is True:
                report.errors.append(ExecutionError(ErrorCode.PARALLEL_EXECUTION_NOT_SUPPORTED_IN_S7, "compiled node requires unsupported concurrent execution", {"node_id": node.node_id}))
            if node.capability_id is None:
                continue
            capability = self.registry.get(node.capability_id)
            if capability is None:
                report.errors.append(ExecutionError(ErrorCode.CAPABILITY_NOT_FOUND, "compiled capability is no longer registered", {"node_id": node.node_id, "capability_id": node.capability_id}))
                continue
            if capability.capability_version != node.capability_version:
                report.errors.append(ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "capability version drift detected", {"capability_id": node.capability_id, "compiled": node.capability_version, "current": capability.capability_version}))
            if capability.dispatch_support != node.dispatch_status:
                report.errors.append(ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "capability dispatch disposition drift detected", {"capability_id": node.capability_id, "compiled": node.dispatch_status, "current": capability.dispatch_support}))
            mode_status = capability.mode_support.get(self.mode, "unsupported")
            if mode_status not in {"supported", "planned_only"}:
                report.errors.append(ExecutionError(ErrorCode.CAPABILITY_UNSUPPORTED, "capability is not allowed in this offline mode", {"capability_id": node.capability_id, "mode": self.mode, "status": mode_status}))
            try:
                input_digest = digest(self.registry.resolve_input_schema(capability.capability_id))
                output_digest = digest(self.registry.resolve_output_schema(capability.capability_id))
                if node.input_schema_digest != input_digest or node.output_schema_digest != output_digest:
                    report.errors.append(ExecutionError(ErrorCode.CHECKPOINT_MISMATCH, "capability schema digest drift detected", {"capability_id": node.capability_id}))
            except Exception as exc:
                report.errors.append(ExecutionError(ErrorCode.SCHEMA_VALIDATION_FAILED, f"capability schemas unavailable: {exc}", {"capability_id": node.capability_id}))
        return report

    def validate_or_raise(self) -> None:
        report = self.validate()
        if not report.ok:
            first = report.errors[0]
            raise first
