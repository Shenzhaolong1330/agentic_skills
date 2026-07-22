from __future__ import annotations

import pytest

from agentic_skills_harness.contracts.resources import ResourceRequirement
from agentic_skills_harness.execution.errors import ExecutionError
from agentic_skills_harness.execution.resources import ResourceLockManager


def requirement(resource_id, mode):
    return ResourceRequirement(resource_id, mode, "test resource")


def test_shared_shared_and_exclusive_conflicts():
    manager = ResourceLockManager()
    shared = requirement("r", "shared")
    exclusive = requirement("r", "exclusive")
    manager.acquire([shared], owner="a")
    manager.acquire([shared], owner="b")
    with pytest.raises(ExecutionError):
        manager.acquire([exclusive], owner="c")
    manager.release([shared], owner="a")
    manager.release([shared], owner="b")
    manager.acquire([exclusive], owner="c")
    assert not manager.is_empty
    manager.release([exclusive], owner="c")
    assert manager.is_empty


def test_canonical_multi_resource_acquisition_rolls_back_on_conflict():
    manager = ResourceLockManager()
    manager.acquire([requirement("z", "exclusive")], owner="held")
    with pytest.raises(ExecutionError):
        manager.acquire([requirement("a", "exclusive"), requirement("z", "exclusive")], owner="new")
    assert manager.snapshot() == {"z": [{"owner": "held", "mode": "exclusive", "count": 1}]}


def test_reentrant_owner_and_resume_clear():
    manager = ResourceLockManager()
    item = requirement("r", "exclusive")
    manager.acquire([item], owner="a")
    manager.acquire([item], owner="a")
    manager.release([item], owner="a")
    assert not manager.is_empty
    manager.clear_for_resume()
    assert manager.is_empty
