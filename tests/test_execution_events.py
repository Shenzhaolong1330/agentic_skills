from __future__ import annotations

import json

import pytest

from agentic_skills_harness.execution.errors import EventLogError
from agentic_skills_harness.execution.events import EventStore, EventType


def test_event_sequence_and_hash_chain(tmp_path):
    store = EventStore(tmp_path, run_id="r", graph_id="g", fsync=False)
    first = store.append(EventType.TASK_CREATED, payload={"safe": True})
    second = store.append(EventType.NODE_STARTED, node_id="n", payload={"value": 1}, causal_event_id=first["event_id"])
    assert second["sequence"] == 2
    assert second["previous_event_digest"] == first["event_digest"]
    assert store.read_verify()[-1]["event_digest"] == second["event_digest"]


@pytest.mark.parametrize("mutation", ["sequence", "digest", "truncate"])
def test_corrupt_event_log_fails_closed(tmp_path, mutation):
    store = EventStore(tmp_path, run_id="r", graph_id="g", fsync=False)
    store.append(EventType.TASK_CREATED, payload={})
    path = tmp_path / "events.jsonl"
    value = json.loads(path.read_text())
    if mutation == "sequence":
        value["sequence"] = 4
        path.write_text(json.dumps(value) + "\n")
    elif mutation == "digest":
        value["event_digest"] = "bad"
        path.write_text(json.dumps(value) + "\n")
    else:
        path.write_text("{\n")
    with pytest.raises(EventLogError):
        EventStore(tmp_path, run_id="r", graph_id="g", fsync=False)
