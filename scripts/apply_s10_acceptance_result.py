#!/usr/bin/env python3
"""Validate a real operator artifact and emit a reviewable readiness patch."""

from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from agentic_skills_harness.live.evidence import AcceptanceEvidence
from agentic_skills_harness.live.readiness import ReadinessRegistry

LEVEL_RANK = {name: index for index, name in enumerate(("H0_CONFIG", "H1_READ_ONLY", "H2_GRIPPER_EMPTY", "H3_MOTION_P2P", "H4_MOTION_RELATIVE", "H5_STOP", "H6_GRASP_VERIFICATION", "H7_RECOVERY", "H8_RESET_HOME"))}


def build_patch(result: dict[str, Any], registry: ReadinessRegistry) -> dict[str, Any]:
    if result.get("real_hardware") is not True: raise ValueError("only explicit operator hardware artifacts may be promoted")
    evidence = AcceptanceEvidence.from_dict(result)
    if not evidence.usable(): raise ValueError("acceptance evidence is failed, revoked, or expired")
    readiness = registry.require(evidence.capability_id)
    if LEVEL_RANK[evidence.acceptance_level] < LEVEL_RANK[readiness.required_acceptance_level]: raise ValueError("acceptance_level_insufficient")
    if evidence.capability_version != readiness.capability_version: raise ValueError("capability_version_mismatch")
    if evidence.adapter_digest != readiness.adapter_digest: raise ValueError("adapter_digest_mismatch")
    if evidence.input_schema_digest != readiness.input_schema_digest or evidence.output_schema_digest != readiness.output_schema_digest: raise ValueError("schema_digest_mismatch")
    if evidence.workspace_digest != result.get("workspace_digest"): raise ValueError("workspace_mismatch")
    state = "VALIDATED_READ_ONLY" if evidence.acceptance_level == "H1_READ_ONLY" else "VALIDATED_RECOVERY" if evidence.acceptance_level in {"H7_RECOVERY", "H8_RESET_HOME"} else "VALIDATED_ACTION"
    return {"capability_id": evidence.capability_id, "patch": {"readiness_state": state, "accepted_hardware_fingerprints": [evidence.hardware_fingerprint_digest], "accepted_calibration_hashes": [evidence.calibration_hash]}, "evidence_digest": evidence.digest, "auto_promoted": False, "requires_operator_review": True}


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--result", type=Path, required=True)
    parser.add_argument("--readiness", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args(argv)
    try:
        result = json.loads(args.result.read_text(encoding="utf-8"))
        registry = ReadinessRegistry.from_dict(json.loads(args.readiness.read_text(encoding="utf-8")))
        patch = build_patch(result, registry)
    except Exception as exc:
        print(json.dumps({"ok": False, "error": str(exc)}, ensure_ascii=False))
        return 1
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(patch, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps({"ok": True, "output": str(args.output), "auto_promoted": False}))
    return 0


if __name__ == "__main__": raise SystemExit(main())
