from __future__ import annotations

from dataclasses import replace
from pathlib import Path
from typing import Any, Iterable

from .capability import CapabilityContract
from .contracts.enums import CapabilityKind, RiskClass
from .schema_validation import load_local_schema


class CapabilityNotFoundError(KeyError):
    def __init__(self, capability_id: str) -> None:
        self.capability_id = capability_id
        super().__init__(f"capability not found: {capability_id}")


class CapabilityRegistry:
    """Read-only capability metadata and local schema registry."""

    def __init__(self, capabilities: Iterable[CapabilityContract], *, repo_root: str | Path) -> None:
        self._repo_root = Path(repo_root).resolve()
        values = sorted(tuple(capabilities), key=lambda item: item.capability_id)
        ids = [item.capability_id for item in values]
        if len(ids) != len(set(ids)):
            duplicates = sorted({item for item in ids if ids.count(item) > 1})
            raise ValueError(f"duplicate capability IDs: {duplicates}")
        self._capabilities = tuple(values)
        self._by_id = {item.capability_id: item for item in self._capabilities}

    @classmethod
    def from_manifest(cls, manifest: dict[str, Any] | str | Path, *, repo_root: str | Path | None = None) -> "CapabilityRegistry":
        from .manifest import validate_manifest_v02

        if isinstance(manifest, (str, Path)):
            manifest_path = Path(manifest).resolve()
            from .manifest import load_manifest

            manifest = load_manifest(manifest_path)
            repo_root = repo_root or manifest_path.parent
        if repo_root is None:
            raise ValueError("repo_root is required when loading a manifest object")
        validate_manifest_v02(manifest, repo_root=repo_root)
        values = []
        for skill in manifest["skills"]:
            for entrypoint in skill["entrypoints"]:
                values.append(CapabilityContract.from_dict(entrypoint, skill_name=skill["name"], skill_layer=skill.get("layer", "")))
        return cls(values, repo_root=repo_root)

    @classmethod
    def from_manifest_path(cls, manifest_path: str | Path) -> "CapabilityRegistry":
        from .manifest import load_manifest

        path = Path(manifest_path).resolve()
        return cls.from_manifest(load_manifest(path), repo_root=path.parent)

    def get(self, capability_id: str) -> CapabilityContract | None:
        return self._by_id.get(capability_id)

    def require(self, capability_id: str) -> CapabilityContract:
        result = self.get(capability_id)
        if result is None:
            raise CapabilityNotFoundError(capability_id)
        return result

    def list(self, *, kind: CapabilityKind | str | None = None, visibility: str | None = None, requires_hardware: bool | None = None, risk_class: RiskClass | str | None = None, allowed_as_recovery: bool | None = None, opens_camera: bool | None = None, moves_robot: bool | None = None, controls_gripper: bool | None = None, include_internal: bool = False, include_legacy: bool = False) -> tuple[CapabilityContract, ...]:
        if visibility is None and not include_internal and not include_legacy:
            visibility = "public"
        result = self._capabilities
        if visibility is not None:
            result = tuple(item for item in result if item.visibility == visibility)
        elif not include_internal and not include_legacy:
            result = tuple(item for item in result if item.visibility == "public")
        elif not include_internal:
            result = tuple(item for item in result if item.visibility != "internal")
        if not include_legacy:
            result = tuple(item for item in result if item.visibility != "legacy")
        if kind is not None:
            kind_value = kind.value if isinstance(kind, CapabilityKind) else kind
            result = tuple(item for item in result if item.kind.value == kind_value)
        if risk_class is not None:
            risk_value = risk_class.value if isinstance(risk_class, RiskClass) else risk_class
            result = tuple(item for item in result if item.risk_class.value == risk_value)
        for name, expected in (("requires_hardware", requires_hardware), ("allowed_as_recovery", allowed_as_recovery), ("opens_camera", opens_camera), ("moves_robot", moves_robot), ("controls_gripper", controls_gripper)):
            if expected is not None:
                result = tuple(item for item in result if getattr(item, name) is expected)
        return tuple(sorted(result, key=lambda item: item.capability_id))

    def filter(self, **kwargs: Any) -> tuple[CapabilityContract, ...]:
        return self.list(**kwargs)

    def resolve_input_schema(self, capability_id: str) -> Any:
        return load_local_schema(self.require(capability_id).input_schema_ref, self._repo_root)

    def resolve_output_schema(self, capability_id: str) -> Any:
        return load_local_schema(self.require(capability_id).output_schema_ref, self._repo_root)

    def validate(self) -> list[str]:
        errors: list[str] = []
        for item in self._capabilities:
            try:
                load_local_schema(item.input_schema_ref, self._repo_root)
                load_local_schema(item.output_schema_ref, self._repo_root)
            except Exception as exc:
                errors.append(f"{item.capability_id}: {exc}")
        return errors

    @property
    def repo_root(self) -> Path:
        return self._repo_root

    def __len__(self) -> int:
        return len(self._capabilities)
