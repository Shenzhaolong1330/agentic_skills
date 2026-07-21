from __future__ import annotations

import json
from pathlib import Path, PurePosixPath
import re
from typing import Any

from .capability import CapabilityContract
from .schema_validation import assert_valid_json, load_local_schema


_SCHEMA_REF_RE = re.compile(r"^(?![A-Za-z][A-Za-z0-9+.-]*://)(?!file:)(?!/)(?![A-Za-z]:[\\/])(?!\\)(?!.*(?:^|/)\.\.(?:/|$)).+$")


def load_manifest(path: str | Path) -> dict[str, Any]:
    manifest_path = Path(path).resolve()
    data = json.loads(manifest_path.read_text(encoding="utf-8"))
    validate_minimal_manifest(data)
    if data.get("version") == "0.2.0":
        validate_manifest_v02(data, repo_root=manifest_path.parent)
    return data


def validate_minimal_manifest(manifest: dict[str, Any]) -> None:
    if not isinstance(manifest, dict):
        raise ValueError("manifest must be a JSON object")
    if "skills" not in manifest or not isinstance(manifest["skills"], list):
        raise ValueError("manifest.skills must be a list")
    gate = manifest.get("hardware_gate")
    if not isinstance(gate, dict):
        raise ValueError("manifest.hardware_gate must be an object")
    if gate.get("default_hardware_allowed") is not False:
        raise ValueError("manifest.hardware_gate.default_hardware_allowed must be false")
    if gate.get("requires_hardware_allowed") is not True:
        raise ValueError("manifest.hardware_gate.requires_hardware_allowed must be true")
    if gate.get("requires_execute_for_side_effects") is not True:
        raise ValueError("manifest.hardware_gate.requires_execute_for_side_effects must be true")


def _safe_relative_path(value: str, field: str) -> None:
    if not isinstance(value, str) or not value.strip() or "\\" in value or Path(value).is_absolute() or value.startswith("/"):
        raise ValueError(f"{field} must be a repository-relative POSIX path")
    if ".." in PurePosixPath(value).parts:
        raise ValueError(f"{field} must not escape the repository")


def _validate_entrypoint(entrypoint: dict[str, Any], *, skill_name: str, skill_layer: str, repo_root: Path, all_ids: set[str]) -> CapabilityContract:
    capability = CapabilityContract.from_dict(entrypoint, skill_name=skill_name, skill_layer=skill_layer)
    _safe_relative_path(capability.path, f"{skill_name}.{capability.name}.path")
    if not _SCHEMA_REF_RE.fullmatch(capability.input_schema_ref) or not _SCHEMA_REF_RE.fullmatch(capability.output_schema_ref):
        raise ValueError(f"{capability.capability_id}: unsafe schema reference")
    load_local_schema(capability.input_schema_ref, repo_root)
    load_local_schema(capability.output_schema_ref, repo_root)
    if capability.verifier.type == "capability":
        if capability.verifier.capability_id not in all_ids:
            raise ValueError(f"{capability.capability_id}: dangling verifier capability reference")
    elif capability.verifier.capability_id is not None:
        raise ValueError(f"{capability.capability_id}: only capability verifiers may reference a capability")
    is_physical = bool(capability.physical_side_effects or capability.moves_robot or capability.controls_gripper)
    if is_physical and not capability.verifier.required_for_physical_success:
        raise ValueError(f"{capability.capability_id}: physical capability verifier must be required")
    if capability.moves_robot and capability.execute_flag is None:
        raise ValueError(f"{capability.capability_id}: robot motion requires execute_flag")
    if capability.controls_gripper and capability.execute_flag is None:
        raise ValueError(f"{capability.capability_id}: gripper control requires execute_flag")
    if capability.opens_camera and not is_physical and capability.risk_class != capability.risk_class.READ_ONLY_HARDWARE:
        raise ValueError(f"{capability.capability_id}: read-only camera capability must use READ_ONLY_HARDWARE")
    if capability.visibility == "public" and (not capability.description.strip() or not capability.resources or not capability.verifier.notes.strip()):
        raise ValueError(f"{capability.capability_id}: public metadata is incomplete")
    return capability


def validate_manifest_v02(manifest: dict[str, Any], *, repo_root: str | Path | None = None) -> tuple[CapabilityContract, ...]:
    validate_minimal_manifest(manifest)
    root = Path(repo_root or Path.cwd()).resolve()
    assert_valid_json(manifest, "schemas/capability_manifest.schema.json", root)
    ids: list[str] = []
    for skill in manifest["skills"]:
        if not isinstance(skill, dict):
            raise ValueError("manifest.skills entries must be objects")
        for entrypoint in skill["entrypoints"]:
            ids.append(entrypoint["capability_id"])
    duplicate_ids = sorted({item for item in ids if ids.count(item) > 1})
    if duplicate_ids:
        raise ValueError(f"duplicate capability_id values: {duplicate_ids}")
    all_ids = set(ids)
    capabilities = []
    for skill in manifest["skills"]:
        _safe_relative_path(skill["path"], f"{skill['name']}.path")
        for entrypoint in skill["entrypoints"]:
            capabilities.append(_validate_entrypoint(entrypoint, skill_name=skill["name"], skill_layer=skill["layer"], repo_root=root, all_ids=all_ids))
    return tuple(sorted(capabilities, key=lambda item: item.capability_id))


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


def find_capability(manifest: dict[str, Any], capability_id: str) -> dict[str, Any]:
    for skill in manifest.get("skills", []):
        for entrypoint in skill.get("entrypoints", []):
            if entrypoint.get("capability_id") == capability_id:
                return entrypoint
    raise KeyError(f"capability not found: {capability_id}")


def is_recovery_allowed(manifest: dict[str, Any], skill_name: str, entrypoint_name: str) -> bool:
    return bool(find_entrypoint(manifest, skill_name, entrypoint_name).get("allowed_as_recovery"))


def load_capability_registry(path: str | Path = "skill_manifest.json"):
    from .registry import CapabilityRegistry

    manifest_path = Path(path).resolve()
    return CapabilityRegistry.from_manifest(load_manifest(manifest_path), repo_root=manifest_path.parent)
