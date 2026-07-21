#!/usr/bin/env python3
"""Cached rack-slot wrist correction with same-frame VLM+SAM fallback."""
from __future__ import annotations

import math
from pathlib import Path
import time
from typing import Any, Mapping

import cv2
import numpy as np

from rack_grid import pixel_ray_plane_base


class CachedPerceptionError(ValueError):
    def __init__(self, message: str, report: Mapping[str, Any] | None = None) -> None:
        super().__init__(message)
        self.report = dict(report or {})


def locate_hole_with_vlm_sam_frame(
    *,
    image_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: Mapping[str, Any],
    config: Any,
    sam_detector: Any,
    camera_timestamp_ms: float,
    raw_response_path: str | Path | None = None,
) -> tuple[dict[str, Any], dict[str, float]]:
    """Run the legacy empty-hole VLM+SAM logic on an already captured frame."""
    from object_locator.geometry import DepthEstimatorConfig, estimate_position_from_depth
    from object_locator.models import CameraIntrinsics
    from object_locator.openrouter_vlm import OpenRouterVLMClient

    timings: dict[str, float] = {}
    client = OpenRouterVLMClient.from_env(model=config.openrouter.model, timeout_s=config.openrouter.timeout_s)
    started = time.monotonic()
    detection = client.detect_object(
        image_bgr,
        config.target.name,
        target_description=config.target.description,
        jpeg_quality=config.openrouter.jpeg_quality,
        use_json_schema=config.openrouter.use_json_schema,
        temperature=config.openrouter.temperature,
        max_tokens=config.openrouter.max_tokens,
        retry_without_json_schema=config.openrouter.retry_without_json_schema,
        raw_response_path=None if raw_response_path is None else str(raw_response_path),
    )
    timings["vlm"] = time.monotonic() - started
    if not detection.found:
        raise ValueError("legacy VLM found no confirmed empty rack hole")
    started = time.monotonic()
    refined = sam_detector.refine_detection_with_sam(image_bgr, detection)
    timings["sam"] = time.monotonic() - started
    source = config.depth
    started = time.monotonic()
    camera_intrinsics = CameraIntrinsics(**intrinsics)
    position = estimate_position_from_depth(
        np.asarray(depth_m, dtype=np.float32),
        camera_intrinsics,
        refined.bbox,
        DepthEstimatorConfig(
            min_depth_m=source.min_depth_m,
            max_depth_m=source.max_depth_m,
            inner_ratio=source.inner_ratio,
            min_samples=source.min_samples,
            fallback_min_samples=source.fallback_min_samples,
            max_expand_ratio=source.max_expand_ratio,
            expand_steps=source.expand_steps,
            strategy=source.strategy,
        ),
    )
    timings["depth"] = time.monotonic() - started
    return {
        "run_id": f"persistent-frame-{time.time_ns()}",
        "target": config.target.name,
        "found": True,
        "source": "persistent-realsense-frame+legacy-vlm+sam-cache",
        "detection": refined.to_dict(),
        "position": position.to_dict(),
        "position_anchor": "bbox",
        "intrinsics": camera_intrinsics.to_dict(),
        "timestamp_ms": float(camera_timestamp_ms),
    }, timings


def project_base_to_pixel(point_base: np.ndarray, base_T_camera: np.ndarray, intrinsics: Mapping[str, Any]) -> np.ndarray:
    camera_T_base = np.linalg.inv(np.asarray(base_T_camera, dtype=float).reshape(4, 4))
    point_camera = (camera_T_base @ np.append(np.asarray(point_base, dtype=float).reshape(3), 1.0))[:3]
    if point_camera[2] <= 0:
        raise ValueError("cached slot is behind the wrist camera")
    return np.array([
        float(intrinsics["fx"]) * point_camera[0] / point_camera[2] + float(intrinsics["ppx"]),
        float(intrinsics["fy"]) * point_camera[1] / point_camera[2] + float(intrinsics["ppy"]),
    ])


def _local_candidate(image_bgr: np.ndarray, predicted_px: np.ndarray, roi_radius_px: int = 55) -> dict[str, Any]:
    height, width = image_bgr.shape[:2]
    u, v = np.rint(predicted_px).astype(int)
    x1, x2 = max(0, u - roi_radius_px), min(width, u + roi_radius_px + 1)
    y1, y2 = max(0, v - roi_radius_px), min(height, v + roi_radius_px + 1)
    if x2 - x1 < 20 or y2 - y1 < 20:
        raise ValueError("projected slot ROI falls outside wrist image")
    roi = image_bgr[y1:y2, x1:x2]
    gray = cv2.GaussianBlur(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY), (5, 5), 1.0)
    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1.2, 10, param1=60, param2=12, minRadius=4, maxRadius=22)
    candidates: list[tuple[np.ndarray, float]] = []
    if circles is not None:
        for cx, cy, radius in circles[0]:
            point = np.array([cx + x1, cy + y1], dtype=float)
            candidates.append((point, float(radius)))
    if not candidates:
        raise ValueError("no circular hole candidate in cached slot ROI")
    point, radius = min(candidates, key=lambda item: float(np.linalg.norm(item[0] - predicted_px)))
    yy, xx = np.ogrid[:height, :width]
    disk = (xx - point[0]) ** 2 + (yy - point[1]) ** 2 <= max(4.0, radius * 0.75) ** 2
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)[disk]
    bgr = image_bgr[disk]
    red_fraction = float(np.mean(((hsv[:, 0] <= 10) | (hsv[:, 0] >= 170)) & (hsv[:, 1] >= 90)))
    dark_fraction = float(np.mean(np.mean(bgr, axis=1) < 75))
    # Hough has already imposed circularity. Treat weak/distracting appearance
    # conservatively and force the VLM path.
    if red_fraction >= 0.02:
        raise ValueError(f"cached slot appears occupied: red_fraction={red_fraction:.3f}")
    if dark_fraction < 0.35:
        raise ValueError(f"cached slot occupancy uncertain: dark_fraction={dark_fraction:.3f}")
    return {
        "center_px": point,
        "radius_px": radius,
        "bbox": {"x_min": point[0] - radius, "y_min": point[1] - radius, "x_max": point[0] + radius, "y_max": point[1] + radius},
        "roundness": 1.0,
        "red_fraction": red_fraction,
        "dark_fraction": dark_fraction,
        "roi": [x1, y1, x2, y2],
    }


def _depth_quality(depth_m: np.ndarray, center_px: np.ndarray, radius_px: int = 5) -> dict[str, Any]:
    u, v = np.rint(center_px).astype(int)
    y1, y2 = max(0, v - radius_px), min(depth_m.shape[0], v + radius_px + 1)
    x1, x2 = max(0, u - radius_px), min(depth_m.shape[1], u + radius_px + 1)
    values = depth_m[y1:y2, x1:x2]
    valid = values[np.isfinite(values) & (values > 0.05) & (values < 3.0)]
    fraction = float(len(valid) / max(1, values.size))
    return {"ok": len(valid) >= 8 and fraction >= 0.15, "valid_fraction": fraction, "sample_count": int(len(valid))}


def _same_frame_vlm_fallback(
    image_bgr: np.ndarray,
    hole_config: Any,
    detector: Any,
    *,
    raw_response_path: str | Path | None = None,
) -> tuple[dict[str, Any], dict[str, float]]:
    from object_locator.openrouter_vlm import OpenRouterVLMClient

    timings: dict[str, float] = {}
    client = OpenRouterVLMClient.from_env(model=hole_config.openrouter.model, timeout_s=hole_config.openrouter.timeout_s)
    started = time.monotonic()
    detection = client.detect_object(
        image_bgr,
        hole_config.target.name,
        target_description=hole_config.target.description,
        jpeg_quality=hole_config.openrouter.jpeg_quality,
        use_json_schema=hole_config.openrouter.use_json_schema,
        temperature=hole_config.openrouter.temperature,
        max_tokens=hole_config.openrouter.max_tokens,
        retry_without_json_schema=hole_config.openrouter.retry_without_json_schema,
        raw_response_path=None if raw_response_path is None else str(raw_response_path),
    )
    timings["vlm"] = time.monotonic() - started
    if not detection.found:
        raise ValueError("same-frame VLM found no confirmed empty hole")
    started = time.monotonic()
    refined = detector.refine_detection_with_sam(image_bgr, detection)
    timings["sam"] = time.monotonic() - started
    bbox = refined.bbox
    return {
        "center_px": np.array([(bbox.x_min + bbox.x_max) * 0.5, (bbox.y_min + bbox.y_max) * 0.5]),
        "bbox": bbox.to_dict(),
        "detection": refined.to_dict(),
    }, timings


def resolve_cached_slot_from_frame(
    *,
    image_bgr: np.ndarray,
    depth_m: np.ndarray,
    intrinsics: Mapping[str, Any],
    base_T_camera: np.ndarray,
    cached_position_base_m: np.ndarray,
    rack_plane_z_base_m: float,
    hole_config: Any,
    sam_detector: Any,
    max_correction_m: float = 0.015,
    raw_response_path: str | Path | None = None,
) -> tuple[np.ndarray, dict[str, Any]]:
    started_total = time.monotonic()
    predicted_base = np.asarray(cached_position_base_m, dtype=float).reshape(3)
    predicted_px = project_base_to_pixel(predicted_base, base_T_camera, intrinsics)
    report: dict[str, Any] = {
        "mode": "cached-grid",
        "predicted_center_px": predicted_px.tolist(),
        "predicted_position_base_m": predicted_base.tolist(),
        "fallback_used": False,
        "timings_sec": {},
    }
    try:
        local_started = time.monotonic()
        candidate = _local_candidate(image_bgr, predicted_px)
        report["timings_sec"]["local_detection"] = time.monotonic() - local_started
        from object_locator.models import BoundingBox, DetectionResult

        sam_started = time.monotonic()
        bbox = BoundingBox(**candidate["bbox"])
        refined = sam_detector.refine_detection_with_sam(
            image_bgr,
            DetectionResult(
                found=True,
                label="cached rack hole",
                confidence=0.8,
                bbox=bbox,
                notes="local circular candidate",
                source="cached-grid-local-circle",
            ),
        )
        report["timings_sec"]["sam"] = time.monotonic() - sam_started
        refined_bbox = refined.bbox
        candidate["center_px"] = np.array(
            [(refined_bbox.x_min + refined_bbox.x_max) * 0.5, (refined_bbox.y_min + refined_bbox.y_max) * 0.5],
            dtype=float,
        )
        candidate["bbox"] = refined_bbox.to_dict()
        if refined.mask is not None:
            contours, _hierarchy = cv2.findContours(refined.mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                contour = max(contours, key=cv2.contourArea)
                area = float(cv2.contourArea(contour))
                perimeter = float(cv2.arcLength(contour, True))
                roundness = 4.0 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0.0
                candidate["roundness"] = roundness
                if roundness < 0.45:
                    raise ValueError(f"local hole SAM contour roundness too low: {roundness:.3f}")
        depth_quality = _depth_quality(depth_m, candidate["center_px"])
        if not depth_quality["ok"]:
            raise ValueError(f"local hole depth unreliable: {depth_quality}")
        report["local"] = {key: (value.tolist() if isinstance(value, np.ndarray) else value) for key, value in candidate.items()}
        report["local"]["depth_quality"] = depth_quality
        source = "local-circle"
    except Exception as local_error:
        report["fallback_used"] = True
        report["fallback_reason"] = str(local_error)
        candidate, fallback_timings = _same_frame_vlm_fallback(
            image_bgr,
            hole_config,
            sam_detector,
            raw_response_path=raw_response_path,
        )
        report["timings_sec"].update(fallback_timings)
        report["fallback"] = {key: (value.tolist() if isinstance(value, np.ndarray) else value) for key, value in candidate.items()}
        source = "same-frame-vlm+sam"

    corrected = pixel_ray_plane_base(candidate["center_px"], intrinsics, base_T_camera, rack_plane_z_base_m)
    correction_m = float(np.linalg.norm(corrected[:2] - predicted_base[:2]))
    report.update({
        "source": source,
        "corrected_center_px": np.asarray(candidate["center_px"]).tolist(),
        "corrected_position_base_m": corrected.tolist(),
        "correction_xy_m": correction_m,
        "max_correction_xy_m": float(max_correction_m),
    })
    if not math.isfinite(correction_m) or correction_m > float(max_correction_m):
        report["rejected_reason"] = "correction_gate_exceeded"
        report["timings_sec"]["total"] = time.monotonic() - started_total
        raise CachedPerceptionError(
            f"cached slot correction {correction_m:.4f}m exceeds {max_correction_m:.4f}m gate",
            report,
        )
    report["timings_sec"]["total"] = time.monotonic() - started_total
    return corrected, report
