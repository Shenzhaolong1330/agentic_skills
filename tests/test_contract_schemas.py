from __future__ import annotations

from pathlib import Path
import json

import pytest

from agentic_skills_harness.schema_validation import validate_json


ROOT = Path(__file__).resolve().parents[1]
VALID = ROOT / "tests/fixtures/contracts/valid"
INVALID = ROOT / "tests/fixtures/contracts/invalid"
SCHEMAS = {
    "action": "schemas/action_result.schema.json",
    "observation": "schemas/observation_result.schema.json",
    "verification": "schemas/verification_result.schema.json",
    "budget": "schemas/execution_budget.schema.json",
    "resource": "schemas/resource_requirement.schema.json",
    "invalidation": "schemas/state_invalidation.schema.json",
    "error": "schemas/error_info.schema.json",
}


def read(name: str):
    return json.loads((VALID / name).read_text(encoding="utf-8"))


@pytest.mark.parametrize(("filename", "schema"), [
    ("action_planned.json", "action"), ("action_denied.json", "action"), ("action_timeout.json", "action"), ("action_failed.json", "action"),
    ("observation.json", "observation"), ("observation_no_ttl.json", "observation"), ("verification_false.json", "verification"), ("verification_true.json", "verification"),
    ("budget.json", "budget"), ("resource.json", "resource"), ("invalidation.json", "invalidation"), ("error.json", "error"),
])
def test_valid_contract_fixtures(filename, schema):
    assert validate_json(read(filename), SCHEMAS[schema], ROOT) == []


@pytest.mark.parametrize("filename", sorted(path.name for path in INVALID.glob("*.json")))
def test_invalid_contract_fixtures_are_rejected(filename):
    # Every invalid fixture is intentionally invalid against the strict action contract.
    instance = json.loads((INVALID / filename).read_text(encoding="utf-8"))
    assert validate_json(instance, SCHEMAS["action"], ROOT)


def test_all_new_schemas_are_draft_2020_12_and_have_ids():
    for path in ROOT.glob("schemas/*.schema.json"):
        data = json.loads(path.read_text(encoding="utf-8"))
        assert data["$schema"] == "https://json-schema.org/draft/2020-12/schema"
        assert data["$id"]
