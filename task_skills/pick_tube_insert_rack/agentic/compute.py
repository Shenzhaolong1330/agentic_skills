from __future__ import annotations

from typing import Any, Mapping


def extract_tube_geometry(observation: Mapping[str, Any]) -> dict[str, Any]:
    return {"tube_id": str(observation.get("tube_id", "tube_1")), "geometry_source": "fixture", "confidence": float(observation.get("confidence", 1.0))}


def select_arm(gripper_observation: Mapping[str, Any] | None = None) -> str:
    values = dict(gripper_observation or {})
    return "left" if float(values.get("left_closed_fraction", 1.0)) >= float(values.get("right_closed_fraction", 0.0)) else "right"


def build_grasp_plan(geometry: Mapping[str, Any], *, arm: str = "left") -> dict[str, Any]:
    return {"tube_id": geometry.get("tube_id", "tube_1"), "arm": arm, "plan_kind": "offline_pregrasp_grasp"}


def select_hole(observation: Mapping[str, Any] | None = None) -> dict[str, Any]:
    values = dict(observation or {})
    return {"hole_id": str(values.get("hole_id", "rack_hole_1")), "confidence": float(values.get("confidence", 1.0)), "selection": "highest_confidence"}


def project_task_facts(*, tube_id: str = "tube_1", hole_id: str = "rack_hole_1") -> dict[str, Any]:
    return {"tube_id": tube_id, "hole_id": hole_id, "facts_are_evidence_only": True}


__all__ = ["build_grasp_plan", "extract_tube_geometry", "project_task_facts", "select_arm", "select_hole"]
