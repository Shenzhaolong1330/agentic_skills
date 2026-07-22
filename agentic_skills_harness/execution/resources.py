from __future__ import annotations

from dataclasses import dataclass
from threading import RLock
from typing import Iterable

from ..contracts.enums import ResourceMode
from ..contracts.resources import ResourceRequirement
from .errors import ExecutionError
from ..contracts.enums import ErrorCode


@dataclass(frozen=True)
class _HeldResource:
    owner: str
    mode: ResourceMode
    count: int = 1


class ResourceLockManager:
    """In-memory deterministic shared/exclusive resource manager."""

    def __init__(self) -> None:
        self._locks: dict[str, list[_HeldResource]] = {}
        self._lock = RLock()

    @staticmethod
    def _normalize(requirements: Iterable[ResourceRequirement | dict]) -> tuple[ResourceRequirement, ...]:
        values = tuple(item if isinstance(item, ResourceRequirement) else ResourceRequirement.from_dict(item) for item in requirements)
        by_id: dict[str, ResourceRequirement] = {}
        for item in values:
            current = by_id.get(item.resource_id)
            if current is None or item.mode == ResourceMode.EXCLUSIVE:
                by_id[item.resource_id] = item
        return tuple(sorted(by_id.values(), key=lambda item: item.resource_id))

    def _can_acquire(self, item: ResourceRequirement, owner: str) -> bool:
        held = self._locks.get(item.resource_id, [])
        if not held:
            return True
        if all(lock.owner == owner for lock in held):
            return True
        if item.mode == ResourceMode.SHARED and all(lock.mode == ResourceMode.SHARED for lock in held):
            return True
        return False

    def acquire(self, requirements: Iterable[ResourceRequirement | dict], *, owner: str) -> tuple[str, ...]:
        if not owner or not isinstance(owner, str):
            raise ExecutionError(ErrorCode.RESOURCE_ACQUISITION_FAILED, "resource owner must be a non-empty string")
        normalized = self._normalize(requirements)
        with self._lock:
            if any(not self._can_acquire(item, owner) for item in normalized):
                raise ExecutionError(ErrorCode.RESOURCE_ACQUISITION_FAILED, "one or more resources are already held", {"resources": [item.resource_id for item in normalized]})
            acquired: list[str] = []
            for item in normalized:
                held = self._locks.setdefault(item.resource_id, [])
                existing = next((index for index, value in enumerate(held) if value.owner == owner and value.mode == item.mode), None)
                if existing is None:
                    held.append(_HeldResource(owner, item.mode))
                else:
                    held[existing] = _HeldResource(owner, item.mode, held[existing].count + 1)
                acquired.append(item.resource_id)
            return tuple(acquired)

    def release(self, requirements: Iterable[ResourceRequirement | dict], *, owner: str) -> tuple[str, ...]:
        normalized = self._normalize(requirements)
        released: list[str] = []
        with self._lock:
            for item in normalized:
                held = self._locks.get(item.resource_id, [])
                updated: list[_HeldResource] = []
                for lock in held:
                    if lock.owner == owner and lock.mode == item.mode:
                        if lock.count > 1:
                            updated.append(_HeldResource(lock.owner, lock.mode, lock.count - 1))
                        released.append(item.resource_id)
                    else:
                        updated.append(lock)
                if updated:
                    self._locks[item.resource_id] = updated
                else:
                    self._locks.pop(item.resource_id, None)
        return tuple(released)

    def release_owner(self, owner: str) -> tuple[str, ...]:
        with self._lock:
            values = tuple(self._locks)
            for resource_id in values:
                self._locks[resource_id] = [item for item in self._locks[resource_id] if item.owner != owner]
                if not self._locks[resource_id]:
                    self._locks.pop(resource_id, None)
            return tuple(sorted(set(values) - set(self._locks)))

    def clear_for_resume(self) -> None:
        with self._lock:
            self._locks.clear()

    def snapshot(self) -> dict[str, list[dict[str, str | int]]]:
        with self._lock:
            return {key: [{"owner": item.owner, "mode": item.mode.value, "count": item.count} for item in value] for key, value in sorted(self._locks.items())}

    @property
    def is_empty(self) -> bool:
        return not bool(self._locks)
