from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path, PurePosixPath
from typing import Iterable


@dataclass(frozen=True)
class PresetSpec:
    preset_id: str
    repo_relative_config_path: str
    supported_capability_id: str
    description: str

    def __post_init__(self) -> None:
        path = PurePosixPath(self.repo_relative_config_path)
        if path.is_absolute() or ".." in path.parts or "://" in self.repo_relative_config_path:
            raise ValueError("preset path must be a safe repository-relative path")
        if not self.preset_id or not self.supported_capability_id or not self.description:
            raise ValueError("preset fields must be non-empty")

    def to_dict(self) -> dict[str, str]:
        return {
            "preset_id": self.preset_id,
            "repo_relative_config_path": self.repo_relative_config_path,
            "supported_capability_id": self.supported_capability_id,
            "description": self.description,
        }


class PresetCatalog:
    """Closed first-party preset catalogue; IDs are the only public selector."""

    def __init__(self, presets: Iterable[PresetSpec] = ()) -> None:
        values = tuple(presets)
        ids = [item.preset_id for item in values]
        if len(ids) != len(set(ids)):
            raise ValueError("duplicate preset ID")
        self._presets = {item.preset_id: item for item in values}

    def get(self, preset_id: str) -> PresetSpec | None:
        return self._presets.get(preset_id)

    def require(self, preset_id: str, capability_id: str | None = None) -> PresetSpec:
        value = self.get(preset_id)
        if value is None:
            raise ValueError(f"unknown preset: {preset_id}")
        if capability_id is not None and value.supported_capability_id != capability_id:
            raise ValueError(f"preset {preset_id} is not valid for {capability_id}")
        return value

    def resolve(self, preset_id: str, repo_root: str | Path, capability_id: str | None = None) -> Path:
        value = self.require(preset_id, capability_id)
        root = Path(repo_root).resolve()
        path = (root / value.repo_relative_config_path).resolve()
        path.relative_to(root)
        return path

    def to_dict(self) -> list[dict[str, str]]:
        return [self._presets[key].to_dict() for key in sorted(self._presets)]


def default_preset_catalog() -> PresetCatalog:
    return PresetCatalog(
        (
            PresetSpec(
                "handover.transition.v1",
                "procedure_skills/dual_franka_handover_transition/config/transition.json",
                "procedure.handover_transition",
                "First-party safe handover transition pose catalogue.",
            ),
        )
    )
