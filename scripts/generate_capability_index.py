#!/usr/bin/env python3
"""Generate the deterministic public capability index."""

from __future__ import annotations

import argparse
from pathlib import Path
import sys

ROOT = Path(__file__).resolve().parents[1]
OUTPUT = ROOT / "CAPABILITY_INDEX.md"
sys.path.insert(0, str(ROOT))

from agentic_skills_harness.manifest import load_manifest
from agentic_skills_harness.registry import CapabilityRegistry


def render(registry: CapabilityRegistry) -> str:
    lines = [
        "# Capability Index",
        "",
        "> Generated from `skill_manifest.json` v0.2.0. Only `visibility=public` entries are listed.",
        "> This is metadata for routing and review; the Registry never executes a capability.",
        "",
    ]
    for capability in registry.list():
        verifier = capability.verifier
        lines.extend([
            f"## `{capability.capability_id}`",
            "",
            f"- version: `{capability.capability_version}`",
            f"- kind: `{capability.kind.value}`",
            f"- description: {capability.description}",
            f"- input contract: `{capability.input_schema_ref}`",
            f"- output contract: `{capability.output_schema_ref}`",
            f"- requires hardware: `{str(capability.requires_hardware).lower()}`",
            f"- risk class: `{capability.risk_class.value}`",
            f"- physical side effects: {', '.join(capability.physical_side_effects) if capability.physical_side_effects else 'none declared'}",
            f"- resources: {', '.join(f'{item.resource_id} ({item.mode.value})' for item in capability.resources)}",
            f"- verifier: `{verifier.type}`; physical verification limited: `{str(verifier.physical_verification_limited).lower()}`",
            f"- allowed as recovery: `{str(capability.allowed_as_recovery).lower()}`",
            "",
        ])
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--check", action="store_true", help="compare the deterministic output without writing")
    args = parser.parse_args()
    manifest = load_manifest(ROOT / "skill_manifest.json")
    registry = CapabilityRegistry.from_manifest(manifest, repo_root=ROOT)
    rendered = render(registry)
    if args.check:
        actual = OUTPUT.read_text(encoding="utf-8") if OUTPUT.exists() else ""
        if actual != rendered:
            print("CAPABILITY_INDEX.md is stale")
            return 1
        print("CAPABILITY_INDEX.md is up to date")
        return 0
    OUTPUT.write_text(rendered, encoding="utf-8")
    print(f"wrote {OUTPUT}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
