from __future__ import annotations

import json
from pathlib import Path
from typing import Any


def load_manifest(path: str | Path) -> dict[str, Any]:
    data = json.loads(Path(path).read_text(encoding="utf-8"))
    validate_minimal_manifest(data)
    return data


def validate_minimal_manifest(manifest: dict[str, Any]) -> None:
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be a JSON object")
    if "skills" not in manifest or not isinstance(manifest["skills"], list):
        raise ValueError("manifest.skills must be a list")
    names = {skill.get("name") for skill in manifest["skills"] if isinstance(skill, dict)}
    required = {
        "atomic-state-dual-franka-reset",
        "procedure-robot-reset-home",
        "task-pick-tube-insert-rack",
    }
    missing = sorted(required - names)
    if missing:
        raise ValueError(f"manifest missing required skills: {missing}")
    gate = manifest.get("hardware_gate")
    if not isinstance(gate, dict):
        raise ValueError("manifest.hardware_gate must be an object")
    if gate.get("default_hardware_allowed") is not False:
        raise ValueError("manifest.hardware_gate.default_hardware_allowed must be false")
    if gate.get("requires_hardware_allowed") is not True:
        raise ValueError("manifest.hardware_gate.requires_hardware_allowed must be true")
    if gate.get("requires_execute_for_side_effects") is not True:
        raise ValueError("manifest.hardware_gate.requires_execute_for_side_effects must be true")


def find_skill(manifest: dict[str, Any], name: str) -> dict[str, Any]:
    for skill in manifest.get("skills", []):
        if skill.get("name") == name:
            return skill
    raise KeyError(f"skill not found in manifest: {name}")


def find_entrypoint(manifest: dict[str, Any], skill_name: str, entrypoint_name: str) -> dict[str, Any]:
    skill = find_skill(manifest, skill_name)
    for entrypoint in skill.get("entrypoints", []):
        if entrypoint.get("name") == entrypoint_name:
            return entrypoint
    raise KeyError(f"entrypoint not found: {skill_name}.{entrypoint_name}")


def is_recovery_allowed(manifest: dict[str, Any], skill_name: str, entrypoint_name: str) -> bool:
    return bool(find_entrypoint(manifest, skill_name, entrypoint_name).get("allowed_as_recovery"))
