from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Mapping
from datetime import datetime, timezone


PRESETS = {"head": "config/presets/head_camera.yaml", "wrist_left": "config/presets/wrist_left.yaml", "wrist_right": "config/presets/wrist_right.yaml"}


def observe_scene(locator: Any, *, preset_id: str, hardware_allowed: bool, calibration_hash: str, expected_calibration_hash: str) -> dict[str, Any]:
    if not hardware_allowed: return {"ok": False, "status": "AUTHORIZATION_DENIED", "reason": "hardware_allowed_required"}
    if preset_id not in PRESETS: return {"ok": False, "status": "INVALID_INPUT", "reason": "unknown_preset_id"}
    if calibration_hash != expected_calibration_hash: return {"ok": False, "status": "FRAME_OR_CALIBRATION_INVALID", "reason": "calibration_hash_mismatch"}
    method = getattr(locator, "locate", None) or getattr(locator, "observe", None)
    if not callable(method): return {"ok": False, "status": "CAPABILITY_UNSUPPORTED", "reason": "locator_api_unavailable"}
    result = method(preset_id=preset_id)
    return {"ok": True, "frame": result.get("frame", "UNKNOWN"), "timestamp": result.get("timestamp", datetime.now(timezone.utc).isoformat()), "confidence": result.get("confidence"), "calibration_hash": calibration_hash, "artifact_refs": result.get("artifact_refs", []), "freshness": result.get("freshness", "fresh"), "observation": result}


@dataclass(frozen=True)
class SceneObservationAdapter:
    locator: Any
    preset_id: str

    def observe(self, *, hardware_allowed: bool, calibration_hash: str, expected_calibration_hash: str) -> dict[str, Any]:
        return observe_scene(self.locator, preset_id=self.preset_id, hardware_allowed=hardware_allowed, calibration_hash=calibration_hash, expected_calibration_hash=expected_calibration_hash)
