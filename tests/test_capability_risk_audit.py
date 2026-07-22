from __future__ import annotations

import json
from pathlib import Path

from scripts.audit_capability_contracts import audit


ROOT = Path(__file__).resolve().parents[1]


def test_audit_covers_every_manifest_capability_without_errors(tmp_path):
    manifest_path = tmp_path / "skill_manifest.json"
    manifest_path.write_text((ROOT / "skill_manifest.json").read_text(encoding="utf-8"), encoding="utf-8")
    _, report = audit(ROOT, manifest_path)
    assert report["status"] == "PASS"
    assert report["capability_count"] == report["audited_capability_count"] == 18
    assert report["errors"] == []
    assert report["unresolved_ambiguities"] == 0
    assert report["reset_invalidation_missing"] == 0
    assert report["physical_verifier_missing"] == 0


def test_audit_makes_self_verifier_and_reset_contracts_safe(tmp_path):
    data = json.loads((ROOT / "skill_manifest.json").read_text(encoding="utf-8"))
    entry = next(entry for skill in data["skills"] for entry in skill["entrypoints"] if entry["capability_id"] == "motion.move_to_pose")
    entry["verifier"] = {"type": "capability", "capability_id": "motion.move_to_pose", "required_for_physical_success": True, "physical_verification_limited": False, "notes": "bad"}
    manifest_path = tmp_path / "skill_manifest.json"
    manifest_path.write_text(json.dumps(data), encoding="utf-8")
    _, report = audit(ROOT, manifest_path)
    fixed = json.loads(manifest_path.read_text(encoding="utf-8"))
    fixed_entry = next(entry for skill in fixed["skills"] for entry in skill["entrypoints"] if entry["capability_id"] == "motion.move_to_pose")
    assert report["status"] == "PASS"
    assert fixed_entry["verifier"]["type"] == "task_specific"
    assert fixed_entry["verifier"]["physical_verification_limited"] is True
