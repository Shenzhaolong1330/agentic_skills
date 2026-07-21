from __future__ import annotations

from dataclasses import FrozenInstanceError
from pathlib import Path

import pytest

from agentic_skills_harness.capability import CapabilityContract
from agentic_skills_harness.contracts import CapabilityKind, RiskClass
from agentic_skills_harness.manifest import load_capability_registry
from agentic_skills_harness.registry import CapabilityNotFoundError


ROOT = Path(__file__).resolve().parents[1]


def registry():
    return load_capability_registry(ROOT / "skill_manifest.json")


def test_lookup_filter_and_stable_order():
    value = registry()
    assert value.require("motion.move_to_pose").kind is CapabilityKind.ACTION
    assert value.get("missing.capability") is None
    with pytest.raises(CapabilityNotFoundError):
        value.require("missing.capability")
    assert [item.capability_id for item in value.list()] == sorted(item.capability_id for item in value.list())
    assert all(item.visibility == "public" for item in value.list())
    assert value.list(kind="observation")
    assert {item.capability_id for item in value.list(risk_class=RiskClass.MOTION)} >= {"motion.move_to_pose", "motion.go_home"}
    assert value.list(include_internal=True, visibility="internal")
    assert not any(item.visibility == "internal" for item in value.list())


def test_contract_is_frozen_and_schema_refs_are_local():
    capability = registry().require("motion.move_to_pose")
    with pytest.raises(FrozenInstanceError):
        capability.notes = "changed"
    assert registry().resolve_input_schema("motion.move_to_pose")["title"] == "Move-to-pose input"


def test_registry_source_has_no_execution_or_hardware_primitives():
    source = (ROOT / "agentic_skills_harness/registry.py").read_text(encoding="utf-8")
    for forbidden in ("subprocess", "Popen", "os.system", "shell=True", "pyrealsense2", "zerorpc", "rospy", "rclpy"):
        assert forbidden not in source
