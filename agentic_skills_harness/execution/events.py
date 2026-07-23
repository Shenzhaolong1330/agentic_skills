from __future__ import annotations

import hashlib
import json
import os
from enum import Enum
from pathlib import Path
import re
import uuid
from typing import Any, Mapping

from ..contracts.serialization import stable_dumps, utc_now_iso
from .errors import EventLogError
from ..contracts.enums import ErrorCode


class EventType(str, Enum):
    TASK_CREATED = "TASK_CREATED"
    EXECUTION_PREFLIGHT_PASSED = "EXECUTION_PREFLIGHT_PASSED"
    EXECUTION_PREFLIGHT_FAILED = "EXECUTION_PREFLIGHT_FAILED"
    CHECKPOINT_RESTORED = "CHECKPOINT_RESTORED"
    NODE_READY = "NODE_READY"
    NODE_STARTED = "NODE_STARTED"
    RESOURCE_ACQUIRED = "RESOURCE_ACQUIRED"
    INPUTS_RESOLVED = "INPUTS_RESOLVED"
    PRECONDITION_EVALUATED = "PRECONDITION_EVALUATED"
    DISPATCH_REQUESTED = "DISPATCH_REQUESTED"
    DISPATCH_COMPLETED = "DISPATCH_COMPLETED"
    PREDICATE_EVALUATED = "PREDICATE_EVALUATED"
    VERIFICATION_COMPLETED = "VERIFICATION_COMPLETED"
    STATE_INVALIDATED = "STATE_INVALIDATED"
    WORLD_STATE_UPDATED = "WORLD_STATE_UPDATED"
    NODE_SUCCEEDED = "NODE_SUCCEEDED"
    NODE_FAILED = "NODE_FAILED"
    NODE_RETRY_SCHEDULED = "NODE_RETRY_SCHEDULED"
    RESOURCE_RELEASED = "RESOURCE_RELEASED"
    EDGE_TRAVERSED = "EDGE_TRAVERSED"
    BUDGET_UPDATED = "BUDGET_UPDATED"
    NO_PROGRESS_DETECTED = "NO_PROGRESS_DETECTED"
    HUMAN_ACTION_REQUESTED = "HUMAN_ACTION_REQUESTED"
    CHECKPOINT_WRITTEN = "CHECKPOINT_WRITTEN"
    TASK_CANCELLED = "TASK_CANCELLED"
    TASK_TERMINATED = "TASK_TERMINATED"
    RECOVERY_CONTEXT_CREATED = "RECOVERY_CONTEXT_CREATED"
    RECOVERY_CANDIDATES_EVALUATED = "RECOVERY_CANDIDATES_EVALUATED"
    RECOVERY_SELECTED = "RECOVERY_SELECTED"
    RECOVERY_REJECTED = "RECOVERY_REJECTED"
    RECOVERY_SUBGRAPH_COMPILED = "RECOVERY_SUBGRAPH_COMPILED"
    RECOVERY_STARTED = "RECOVERY_STARTED"
    RECOVERY_COMPLETED = "RECOVERY_COMPLETED"
    RECOVERY_FAILED = "RECOVERY_FAILED"
    RECOVERY_EXHAUSTED = "RECOVERY_EXHAUSTED"
    REPLAN_REQUESTED = "REPLAN_REQUESTED"
    REPLAN_COMPILED = "REPLAN_COMPILED"
    REPLAN_REJECTED = "REPLAN_REJECTED"
    PLAN_REPLACED = "PLAN_REPLACED"
    PLAN_LINEAGE_UPDATED = "PLAN_LINEAGE_UPDATED"


_SAFE = re.compile(r"^[A-Za-z0-9_.-]+$")


def safe_component(value: str) -> str:
    if not isinstance(value, str) or not value or value in {".", ".."} or not _SAFE.fullmatch(value):
        raise ValueError("unsafe artifact path component")
    return value


class ArtifactStore:
    def __init__(self, root: str | Path) -> None:
        self.root = Path(root).expanduser()
        if self.root.exists() and self.root.is_symlink():
            raise ValueError("artifact root must not be a symlink")
        self.root.mkdir(parents=True, exist_ok=True)
        if self.root.resolve() != self.root:
            self.root = self.root.resolve()

    def path(self, relative: str | Path) -> Path:
        value = Path(relative)
        if value.is_absolute() or ".." in value.parts:
            raise ValueError("artifact reference escapes artifact root")
        target = (self.root / value).resolve()
        if self.root not in target.parents and target != self.root:
            raise ValueError("artifact reference escapes artifact root")
        return target

    def write_json(self, relative: str | Path, value: Any) -> str:
        target = self.path(relative)
        if target.exists() and target.is_symlink():
            raise ValueError("artifact target must not be a symlink")
        target.parent.mkdir(parents=True, exist_ok=True)
        target.write_text(stable_dumps(value) + "\n", encoding="utf-8")
        return target.relative_to(self.root).as_posix()

    def read_json(self, relative: str | Path) -> Any:
        return json.loads(self.path(relative).read_text(encoding="utf-8"))


class EventStore:
    def __init__(self, artifact_dir: str | Path, *, run_id: str, graph_id: str, clock: Any | None = None, fsync: bool = True) -> None:
        self.artifacts = ArtifactStore(artifact_dir)
        self.path = self.artifacts.path("events.jsonl")
        self.run_id = run_id
        self.graph_id = graph_id
        self.clock = clock or utc_now_iso
        self.fsync = fsync
        self._events: list[dict[str, Any]] = []
        if self.path.exists() and self.path.is_symlink():
            raise EventLogError(ErrorCode.EVENT_LOG_CORRUPT, "event log must not be a symlink")
        if self.path.exists():
            self._events = self.read_verify()

    @property
    def sequence(self) -> int:
        return len(self._events)

    @property
    def last_digest(self) -> str | None:
        return self._events[-1]["event_digest"] if self._events else None

    def append(self, event_type: EventType | str, *, node_id: str | None = None, attempt_id: str | None = None, payload: Mapping[str, Any] | None = None, causal_event_id: str | None = None) -> dict[str, Any]:
        value = event_type.value if isinstance(event_type, EventType) else str(event_type)
        if value not in {item.value for item in EventType}:
            raise EventLogError(ErrorCode.EVENT_LOG_CORRUPT, f"unknown event type: {value}")
        event = {"event_id": f"evt_{uuid.uuid4().hex}", "sequence": self.sequence + 1, "run_id": self.run_id, "graph_id": self.graph_id, "node_id": node_id, "attempt_id": attempt_id, "event_type": value, "timestamp": self.clock() if callable(self.clock) else str(self.clock), "causal_event_id": causal_event_id, "payload": dict(payload or {}), "previous_event_digest": self.last_digest}
        event["event_digest"] = hashlib.sha256(stable_dumps(event).encode("utf-8")).hexdigest()
        with self.path.open("a", encoding="utf-8") as handle:
            handle.write(stable_dumps(event) + "\n")
            handle.flush()
            if self.fsync:
                os.fsync(handle.fileno())
        self._events.append(event)
        return dict(event)

    def read_verify(self) -> list[dict[str, Any]]:
        result: list[dict[str, Any]] = []
        previous: str | None = None
        try:
            lines = self.path.read_text(encoding="utf-8").splitlines()
            for index, line in enumerate(lines, 1):
                if not line.strip():
                    raise ValueError(f"blank event line {index}")
                event = json.loads(line)
                if not isinstance(event, dict) or event.get("sequence") != index:
                    raise ValueError(f"event sequence mismatch at line {index}")
                if event.get("run_id") != self.run_id or event.get("graph_id") != self.graph_id:
                    raise ValueError(f"event identity mismatch at line {index}")
                if event.get("event_type") not in {item.value for item in EventType}:
                    raise ValueError(f"unknown event type at line {index}")
                if event.get("previous_event_digest") != previous:
                    raise ValueError(f"event previous digest mismatch at line {index}")
                digest = event.get("event_digest")
                unsigned = dict(event)
                unsigned.pop("event_digest", None)
                if not isinstance(digest, str) or hashlib.sha256(stable_dumps(unsigned).encode("utf-8")).hexdigest() != digest:
                    raise ValueError(f"event digest mismatch at line {index}")
                previous = digest
                result.append(event)
        except Exception as exc:
            raise EventLogError(ErrorCode.EVENT_LOG_CORRUPT, f"event log verification failed: {exc}") from exc
        self._events = result
        return result

    def events(self) -> tuple[dict[str, Any], ...]:
        return tuple(dict(item) for item in self._events)
