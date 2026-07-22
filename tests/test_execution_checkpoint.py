from __future__ import annotations

import json

import pytest

from agentic_skills_harness.execution.checkpoint import CheckpointStore
from agentic_skills_harness.execution.errors import CheckpointError


def payload():
    return {"checkpoint_version": "1", "run_id": "r", "graph_id": "g", "terminal_state": "SUCCEEDED"}


def test_checkpoint_atomic_digest_roundtrip(tmp_path):
    store = CheckpointStore(tmp_path, fsync=False)
    store.write(payload())
    assert store.load()["run_id"] == "r"


def test_checkpoint_digest_mismatch_is_rejected(tmp_path):
    store = CheckpointStore(tmp_path, fsync=False)
    store.write(payload())
    path = tmp_path / "checkpoint.json"
    value = json.loads(path.read_text())
    value["graph_id"] = "tampered"
    path.write_text(json.dumps(value))
    with pytest.raises(CheckpointError):
        store.load()


def test_checkpoint_symlink_is_rejected(tmp_path):
    store = CheckpointStore(tmp_path, fsync=False)
    store.write(payload())
    original = tmp_path / "checkpoint.json"
    target = tmp_path / "target.json"
    target.write_text(original.read_text())
    original.unlink()
    original.symlink_to(target)
    with pytest.raises(CheckpointError):
        store.load()
