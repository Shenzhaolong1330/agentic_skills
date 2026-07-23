from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


class WorkspaceViolation(ValueError):
    pass


@dataclass(frozen=True)
class Box:
    min_xyz: tuple[float, float, float]
    max_xyz: tuple[float, float, float]

    def contains(self, xyz: list[float] | tuple[float, ...]) -> bool:
        return len(xyz) == 3 and all(self.min_xyz[i] <= float(xyz[i]) <= self.max_xyz[i] for i in range(3))


@dataclass(frozen=True)
class WorkspaceConfig:
    robot_base_frame: str = "base"
    left: Box = Box((-0.8, -0.8, 0.0), (0.8, 0.8, 1.2))
    right: Box = Box((-0.8, -0.8, 0.0), (0.8, 0.8, 1.2))
    shared: Box = Box((-0.6, -0.6, 0.05), (0.6, 0.6, 1.0))
    forbidden: tuple[Box, ...] = ()
    safe_acceptance_poses: Mapping[str, Mapping[str, Any]] = None  # type: ignore[assignment]

    def __post_init__(self) -> None:
        if not self.robot_base_frame: raise ValueError("robot_base_frame is required")
        object.__setattr__(self, "safe_acceptance_poses", dict(self.safe_acceptance_poses or {}))

    @classmethod
    def from_dict(cls, data: Mapping[str, Any]) -> "WorkspaceConfig":
        allowed = {"robot_base_frame", "left", "right", "shared", "forbidden", "safe_acceptance_poses"}
        if set(data) - allowed: raise ValueError(f"unknown workspace fields: {sorted(set(data)-allowed)}")
        def box(value: Mapping[str, Any] | None, default: Box) -> Box:
            if value is None: return default
            return Box(tuple(float(x) for x in value["min_xyz"]), tuple(float(x) for x in value["max_xyz"]))
        default = cls()
        forbidden = tuple(box(item, default.shared) for item in data.get("forbidden", []))
        return cls(data.get("robot_base_frame", "base"), box(data.get("left"), default.left), box(data.get("right"), default.right), box(data.get("shared"), default.shared), forbidden, data.get("safe_acceptance_poses", {}))

    def validate_pose(self, *, frame: str, xyz_m: list[float] | tuple[float, ...], side: str) -> None:
        if frame != self.robot_base_frame: raise WorkspaceViolation("robot_base_frame_required")
        if len(xyz_m) != 3 or not all(math.isfinite(float(value)) for value in xyz_m): raise WorkspaceViolation("non_finite_pose")
        box = self.shared if side == "both" else (self.left if side == "left" else self.right if side == "right" else None)
        if box is None or not box.contains(xyz_m): raise WorkspaceViolation("workspace_violation")
        if any(item.contains(xyz_m) for item in self.forbidden): raise WorkspaceViolation("forbidden_zone")

    def resolve_acceptance_pose(self, pose_id: str, *, side: str) -> Mapping[str, Any]:
        if pose_id not in self.safe_acceptance_poses: raise WorkspaceViolation("unknown_acceptance_pose_id")
        pose = self.safe_acceptance_poses[pose_id]
        self.validate_pose(frame=str(pose.get("frame")), xyz_m=pose.get("xyz_m", ()), side=side)
        return pose
