from __future__ import annotations

from copy import deepcopy
import json
from pathlib import Path

import pytest

from agentic_skills_harness.manifest import load_manifest, validate_manifest_v02
from agentic_skills_harness.capability import CapabilityContract


ROOT = Path(__file__).resolve().parents[1]


def test_actual_manifest_v02_and_ids():
    manifest = load_manifest(ROOT / "skill_manifest.json")
    capabilities = validate_manifest_v02(manifest, repo_root=ROOT)
    assert manifest["version"] == "0.2.0"
    assert len(capabilities) == len({item.capability_id for item in capabilities})
    assert all(item.capability_version == "1.0.0" for item in capabilities)


def test_manifest_entrypoint_fixture_inventory():
    valid = sorted((ROOT / "tests/fixtures/manifests").glob("valid_*.json"))
    invalid = sorted((ROOT / "tests/fixtures/manifests").glob("invalid_*.json"))
    assert len(valid) >= 12
    assert len(invalid) >= 30
    assert all(CapabilityContract.from_dict(json.loads(path.read_text(encoding="utf-8"))) for path in valid)
    rejected = 0
    for path in invalid:
        try:
            CapabilityContract.from_dict(json.loads(path.read_text(encoding="utf-8")))
        except (ValueError, KeyError, TypeError):
            rejected += 1
    assert rejected == len(invalid)


def test_manifest_does_not_require_a_specific_task_skill():
    manifest = load_manifest(ROOT / "skill_manifest.json")
    reduced = deepcopy(manifest)
    reduced["skills"] = [item for item in reduced["skills"] if item["layer"] != "task"]
    assert validate_manifest_v02(reduced, repo_root=ROOT)


@pytest.mark.parametrize("mutate", [
    lambda m: m["skills"][0]["entrypoints"][0].update({"capability_id": m["skills"][1]["entrypoints"][0]["capability_id"]}),
    lambda m: m["skills"][0]["entrypoints"][0].update({"input_schema_ref": "https://example.invalid/schema.json"}),
    lambda m: m["skills"][0]["entrypoints"][0].update({"path": "../outside.py"}),
    lambda m: m["skills"][1]["entrypoints"][0].update({"moves_robot": True, "requires_hardware": False}),
    lambda m: m["skills"][1]["entrypoints"][0].update({"verifier": {"type":"none","capability_id":None,"required_for_physical_success":True,"physical_verification_limited":False,"notes":"bad"}}),
])
def test_manifest_semantic_errors_are_rejected(mutate):
    manifest = deepcopy(load_manifest(ROOT / "skill_manifest.json"))
    mutate(manifest)
    with pytest.raises((ValueError, KeyError)):
        validate_manifest_v02(manifest, repo_root=ROOT)
