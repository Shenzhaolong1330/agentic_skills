#!/usr/bin/env python3
"""Capture one head-camera frame and inventory every loose test tube in it.

This is a task-level adapter.  It deliberately leaves the atomic object-locator
unchanged, while reusing its camera, VLM/SAM, depth, and calibration
implementations.  Each emitted tube JSON is compatible with the existing
locate_then_grasp_by_tail_side.py and grasp_right_arm_xyz.py scripts.
"""
from __future__ import annotations

import argparse
from dataclasses import replace
from datetime import datetime
import json
import os
from pathlib import Path
import re
import sys
from typing import Any

import cv2
import numpy as np


OBJECT_LOCATOR_ROOT = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
DEFAULT_CONFIG = OBJECT_LOCATOR_ROOT / "config_test_tube_cleanup_leftmost_vlm_head.yaml"
DEFAULT_RACK_CONFIG = OBJECT_LOCATOR_ROOT / "config_rack_center_vlm.yaml"
sys.path.insert(0, str(OBJECT_LOCATOR_ROOT / "src"))

from object_locator.config import load_config  # noqa: E402
from object_locator.geometry import (  # noqa: E402
    DepthEstimatorConfig,
    OrientationEstimatorConfig,
    estimate_orientation,
    estimate_position_from_depth,
)
from object_locator.grounded_sam_detector import (  # noqa: E402
    GroundedSamConfig,
    GroundedSamDetector,
    GroundingCandidate,
    bbox_from_mask,
    estimate_sample_bottle_keypoints,
    _normalize_grounding_prompt,
    _to_numpy,
    _move_batch_to_device,
)
from object_locator.models import BoundingBox, CameraIntrinsics, DetectionResult, PixelPoint  # noqa: E402
from object_locator.openrouter_vlm import (  # noqa: E402
    OpenRouterError,
    OpenRouterVLMClient,
    _detection_from_json,
    _encode_bgr_as_data_url,
    _extract_message_content,
)
from object_locator.realsense_camera import FrameBundle, RealSenseCamera  # noqa: E402
from object_locator.transforms import (  # noqa: E402
    points_base_from_calibration,
    position_base_from_calibration,
)
from rack_grid import build_rack_grid, draw_overlay, save_grid  # noqa: E402
from sam_cache_service import CachedSamRefiner, SOCKET_ENV  # noqa: E402
from wrist_camera_service import WristCameraClient  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--rack-config", type=Path, default=DEFAULT_RACK_CONFIG)
    parser.add_argument("--output-dir", type=Path, required=True)
    parser.add_argument("--inventory-json", type=Path, required=True)
    parser.add_argument("--rack-result-json", type=Path, required=True)
    parser.add_argument("--rack-grid-json", type=Path, default=None)
    parser.add_argument("--rack-grid-overlay", type=Path, default=None)
    parser.add_argument(
        "--camera-service-socket",
        type=Path,
        default=None,
        help="Read the initial head RGB-D frame from the persistent task camera service.",
    )
    parser.add_argument(
        "--build-rack-grid",
        action="store_true",
        help="Experimental: fit/cache the 3x4 rack grid. Disabled by default; legacy wrist VLM does not need it.",
    )
    parser.add_argument(
        "--detector",
        choices=("vlm", "grounded_sam"),
        default="vlm",
        help="Multi-object proposal backend. VLM is the safe default for excluding the rack.",
    )
    parser.add_argument("--box-threshold", type=float, default=None)
    parser.add_argument("--text-threshold", type=float, default=None)
    parser.add_argument("--nms-iou-threshold", type=float, default=0.45)
    parser.add_argument("--min-confidence", type=float, default=0.25)
    parser.add_argument(
        "--min-axis-aspect-ratio",
        type=float,
        default=1.5,
        help="Reject non-elongated boxes such as rack holes or upright tube caps.",
    )
    parser.add_argument("--max-tubes", type=int, default=24)
    parser.add_argument(
        "--allow-rejected-candidates",
        action="store_true",
        help="Proceed with valid tubes if another detected candidate cannot be localized safely.",
    )
    parser.add_argument("--save-rgb", type=Path, default=None)
    parser.add_argument("--save-depth", type=Path, default=None)
    parser.add_argument("--save-vlm-response", type=Path, default=None)
    parser.add_argument("--save-rack-vlm-response", type=Path, default=None)
    parser.add_argument(
        "--reset-realsense",
        action=argparse.BooleanOptionalAction,
        default=True,
        help="Hardware-reset the selected head camera before capture (default: enabled).",
    )
    parser.add_argument("--compact", action="store_true")
    return parser


def progress(message: str) -> None:
    print(f"[tube-inventory] {message}", file=sys.stderr, flush=True)


def parse_multi_vlm_content(content: Any) -> list[dict[str, Any]]:
    """Accept strict-schema output and common schema-free Gemini variants."""
    if isinstance(content, list):
        parsed: Any = content
    else:
        text = str(content or "").strip()
        if text.startswith("```"):
            lines = text.splitlines()
            if lines and lines[0].lstrip().startswith("```"):
                lines = lines[1:]
            if lines and lines[-1].strip() == "```":
                lines = lines[:-1]
            text = "\n".join(lines).strip()
        try:
            parsed = json.loads(text)
        except json.JSONDecodeError as first_error:
            # Gemini occasionally inserts a dangling quote between array items:
            #   {...}, "\n  {...}
            repaired = re.sub(r'}\s*,\s*"\s*(?=\{)', '},\n{', text)
            try:
                parsed = json.loads(repaired, strict=False)
            except json.JSONDecodeError:
                objects = _extract_balanced_json_objects(text)
                if objects:
                    return objects
                raise ValueError("multi-object VLM response does not contain recoverable JSON") from first_error
    raw_tubes = parsed.get("tubes") if isinstance(parsed, dict) else parsed
    if not isinstance(raw_tubes, list):
        raise ValueError("multi-object VLM response does not contain a tubes array")
    return [item for item in raw_tubes if isinstance(item, dict)]


def _extract_balanced_json_objects(text: str) -> list[dict[str, Any]]:
    """Recover complete JSON objects from a damaged top-level array."""
    objects: list[dict[str, Any]] = []
    start: int | None = None
    depth = 0
    in_string = False
    escaped = False
    for index, char in enumerate(text):
        if start is None:
            if char == "{":
                start = index
                depth = 1
                in_string = False
                escaped = False
            continue
        if in_string:
            if escaped:
                escaped = False
            elif char == "\\":
                escaped = True
            elif char == '"':
                in_string = False
            continue
        if char == '"':
            in_string = True
        elif char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                fragment = text[start : index + 1]
                try:
                    parsed = json.loads(fragment, strict=False)
                except json.JSONDecodeError:
                    pass
                else:
                    if isinstance(parsed, dict):
                        objects.append(parsed)
                start = None
    return objects


def _detector_config(config: Any, args: argparse.Namespace) -> GroundedSamConfig:
    source = config.grounded_sam
    return GroundedSamConfig(
        grounding_model=source.grounding_model,
        sam_model=source.sam_model,
        text_prompt=source.text_prompt,
        selection="leftmost",
        box_threshold=source.box_threshold if args.box_threshold is None else args.box_threshold,
        text_threshold=source.text_threshold if args.text_threshold is None else args.text_threshold,
        device=source.device,
        use_sam=source.use_sam,
        refine_bbox_with_mask=source.refine_bbox_with_mask,
        min_box_area_px=source.min_box_area_px,
        max_box_area_ratio=source.max_box_area_ratio,
        min_mask_area_px=source.min_mask_area_px,
        cap_endpoint_rule=source.cap_endpoint_rule,
        cap_dark_threshold=source.cap_dark_threshold,
        cap_min_area_px=source.cap_min_area_px,
    )


def _iou(left: Any, right: Any) -> float:
    x0 = max(left.x_min, right.x_min)
    y0 = max(left.y_min, right.y_min)
    x1 = min(left.x_max, right.x_max)
    y1 = min(left.y_max, right.y_max)
    intersection = max(0.0, x1 - x0) * max(0.0, y1 - y0)
    union = left.area + right.area - intersection
    return 0.0 if union <= 0.0 else intersection / union


def _nms(candidates: list[GroundingCandidate], threshold: float) -> list[GroundingCandidate]:
    kept: list[GroundingCandidate] = []
    for candidate in sorted(candidates, key=lambda item: item.score, reverse=True):
        if all(_iou(candidate.bbox, existing.bbox) <= threshold for existing in kept):
            kept.append(candidate)
    return kept


def detect_all_candidates(
    detector: GroundedSamDetector,
    image_bgr: np.ndarray,
    target: str,
    *,
    min_confidence: float,
    min_axis_aspect_ratio: float,
    nms_iou_threshold: float,
    max_tubes: int,
) -> list[GroundingCandidate]:
    """Run one GroundingDINO inference and retain every filtered candidate."""
    torch = detector._load_torch()
    pil_image = detector._bgr_to_pil(image_bgr)
    processor, model = detector._load_grounding_dino()
    prompt = _normalize_grounding_prompt(detector.config.text_prompt or target)
    inputs = processor(images=pil_image, text=prompt, return_tensors="pt")
    inputs = _move_batch_to_device(inputs, detector._device)
    with torch.no_grad():
        outputs = model(**inputs)
    target_sizes = [pil_image.size[::-1]]
    try:
        results = processor.post_process_grounded_object_detection(
            outputs,
            inputs["input_ids"],
            threshold=detector.config.box_threshold,
            text_threshold=detector.config.text_threshold,
            target_sizes=target_sizes,
        )
    except TypeError:
        results = processor.post_process_grounded_object_detection(
            outputs,
            threshold=detector.config.box_threshold,
            text_threshold=detector.config.text_threshold,
            target_sizes=target_sizes,
        )

    raw = results[0]
    boxes = _to_numpy(raw.get("boxes", []))
    scores = _to_numpy(raw.get("scores", []))
    labels = raw.get("labels", [])
    height, width = image_bgr.shape[:2]
    max_area = height * width * detector.config.max_box_area_ratio
    candidates: list[GroundingCandidate] = []
    for index, raw_box in enumerate(boxes if boxes is not None else []):
        if len(raw_box) < 4:
            continue
        bbox = BoundingBox(*(float(value) for value in raw_box[:4])).clamp(width, height)
        score = float(scores[index]) if scores is not None and index < len(scores) else 0.0
        short_axis = min(bbox.width, bbox.height)
        axis_aspect_ratio = float("inf") if short_axis <= 0.0 else max(bbox.width, bbox.height) / short_axis
        if (
            score < min_confidence
            or axis_aspect_ratio < min_axis_aspect_ratio
            or bbox.area < detector.config.min_box_area_px
            or bbox.area > max_area
            or not bbox.is_valid()
        ):
            continue
        label = str(labels[index]) if index < len(labels) else target
        candidates.append(GroundingCandidate(bbox=bbox, score=score, label=label))
    candidates = _nms(candidates, nms_iou_threshold)
    candidates.sort(key=lambda item: (item.bbox.center[0], item.bbox.center[1]))
    return candidates[:max_tubes]


def detect_all_with_vlm(
    image_bgr: np.ndarray,
    config: Any,
    *,
    min_confidence: float,
    nms_iou_threshold: float,
    max_tubes: int,
    raw_response_path: Path | None,
) -> list[DetectionResult]:
    """Ask once for loose tabletop tube boxes, capped by max_tubes."""
    height, width = image_bgr.shape[:2]
    client = OpenRouterVLMClient.from_env(
        model=config.openrouter.model,
        timeout_s=config.openrouter.timeout_s,
    )
    item_schema = {
        "type": "object",
        "properties": {
            "label": {"type": "string"},
            "confidence": {"type": "number", "minimum": 0, "maximum": 1},
            "box_2d": {
                "type": "object",
                "properties": {
                    "y1": {"type": "number"},
                    "x1": {"type": "number"},
                    "y2": {"type": "number"},
                    "x2": {"type": "number"},
                },
                "required": ["y1", "x1", "y2", "x2"],
                "additionalProperties": False,
            },
            "orientation": {
                "type": "object",
                "properties": {
                    "head_px": {
                        "type": "object",
                        "properties": {"x": {"type": "number"}, "y": {"type": "number"}},
                        "required": ["x", "y"],
                        "additionalProperties": False,
                    },
                    "tail_px": {
                        "type": "object",
                        "properties": {"x": {"type": "number"}, "y": {"type": "number"}},
                        "required": ["x", "y"],
                        "additionalProperties": False,
                    },
                },
                "required": ["head_px", "tail_px"],
                "additionalProperties": False,
            },
            "notes": {"type": "string"},
        },
        "required": ["label", "confidence", "box_2d", "orientation", "notes"],
        "additionalProperties": False,
    }
    schema = {
        "type": "object",
        "properties": {
            "tubes": {"type": "array", "items": item_schema, "maxItems": max_tubes},
        },
        "required": ["tubes"],
        "additionalProperties": False,
    }
    target_instruction = (
        "Find only the leftmost complete loose test tube lying on the bare tabletop. "
        "Return exactly one tube if a valid loose tube is visible; do not inventory the remaining tubes. "
        if max_tubes == 1
        else "Inventory every loose test tube lying on the bare tabletop. "
    )
    ordering_instruction = (
        "Return the selected tube as the only item. "
        if max_tubes == 1
        else "Return tubes in left-to-right order by box center. "
    )
    prompt = (
        f"{target_instruction}The image size is {width}x{height}. "
        "Return one tight box per complete loose tube, including partially transparent body pixels. "
        "Exclude the black test-tube rack, every rack hole, tubes already inserted in the rack, "
        "isolated caps, robot parts, shadows, labels/overlays, and white equipment. "
        "Do not merge nearby tubes. box_2d uses normalized 0-1000 coordinates ordered "
        "y1(top), x1(left), y2(bottom), x2(right). orientation points use original image pixels; "
        "head_px is the black cap/open rim center and tail_px is the opposite closed end. "
        f"{ordering_instruction}If none are visible, return an empty array."
    )
    payload: dict[str, Any] = {
        "model": client.model,
        "messages": [
            {"role": "system", "content": "You are a precise multi-object visual localization engine."},
            {
                "role": "user",
                "content": [
                    {"type": "text", "text": prompt},
                    {
                        "type": "image_url",
                        "image_url": {
                            "url": _encode_bgr_as_data_url(image_bgr, config.openrouter.jpeg_quality)
                        },
                    },
                ],
            },
        ],
        "temperature": config.openrouter.temperature,
        "max_tokens": max(4096, config.openrouter.max_tokens),
        "response_format": {
            "type": "json_schema",
            "json_schema": {"name": "loose_test_tube_inventory", "strict": True, "schema": schema},
        },
    }
    if client.require_parameters:
        payload["provider"] = {"require_parameters": True}
    schema_error = None
    try:
        response = client._post_payload(payload)
    except OpenRouterError as exc:
        if not config.openrouter.retry_without_json_schema:
            raise
        schema_error = str(exc)
        # Some providers accept image+JSON requests but reject array-shaped
        # response_format schemas. The prompt still requires one JSON object.
        payload.pop("response_format", None)
        response = client._post_payload(payload)
    content, _finish_reason = _extract_message_content(response)
    if raw_response_path is not None:
        raw_response_path.parent.mkdir(parents=True, exist_ok=True)
        raw_response_path.write_text(
            json.dumps(
                {
                    "model": client.model,
                    "schema_attempt_error": schema_error,
                    "content": content,
                    "usage": response.get("usage"),
                },
                ensure_ascii=False,
                indent=2,
            ),
            encoding="utf-8",
        )
    raw_tubes = parse_multi_vlm_content(content)
    detections: list[DetectionResult] = []
    for raw in raw_tubes[:max_tubes]:
        if not isinstance(raw, dict):
            continue
        if "orientation" not in raw and ("head_px" in raw or "tail_px" in raw):
            raw = {
                **raw,
                "orientation": {
                    "head_px": raw.get("head_px"),
                    "tail_px": raw.get("tail_px"),
                },
            }
        detection = _detection_from_json(
            {"found": True, **raw}, width, height, fallback_label=config.target.name
        )
        if detection.found and detection.confidence >= min_confidence:
            detections.append(detection)
    # VLMs occasionally repeat a box. Use the same class-agnostic NMS policy.
    kept: list[DetectionResult] = []
    for detection in sorted(detections, key=lambda item: item.confidence, reverse=True):
        if all(_iou(detection.bbox, existing.bbox) <= nms_iou_threshold for existing in kept):
            kept.append(detection)
    kept.sort(key=lambda item: (item.bbox.center[0], item.bbox.center[1]))
    return kept


def refine_candidate(
    detector: GroundedSamDetector,
    image_bgr: np.ndarray,
    candidate: GroundingCandidate,
) -> DetectionResult:
    pil_image = detector._bgr_to_pil(image_bgr)
    mask = None
    sam_score = None
    notes = ["single-frame multi-tube inventory", "GroundingDINO candidate"]
    if detector.config.use_sam:
        try:
            mask, sam_score = detector._segment_with_sam(pil_image, candidate.bbox)
        except Exception as exc:
            notes.append(f"SAM failed: {exc}")
    bbox = candidate.bbox
    head_px = None
    tail_px = None
    if mask is not None and int(mask.sum()) >= detector.config.min_mask_area_px:
        notes.append(f"SAM mask area={int(mask.sum())} px")
        if detector.config.refine_bbox_with_mask:
            refined = bbox_from_mask(mask)
            if refined is not None:
                bbox = refined
        head_px, tail_px = estimate_sample_bottle_keypoints(
            image_bgr,
            mask,
            bbox,
            cap_endpoint_rule=detector.config.cap_endpoint_rule,
            cap_dark_threshold=detector.config.cap_dark_threshold,
            cap_min_area_px=detector.config.cap_min_area_px,
        )
    confidence = candidate.score
    if sam_score is not None:
        confidence = min(1.0, 0.7 * candidate.score + 0.3 * float(sam_score))
    return DetectionResult(
        found=True,
        label=candidate.label,
        confidence=float(confidence),
        bbox=bbox,
        notes="; ".join(notes),
        source="task_multi_grounded_sam",
        head_px=head_px,
        tail_px=tail_px,
        mask=mask,
    )


def restore_validated_vlm_orientation(
    detection: DetectionResult,
    vlm_detection: DetectionResult,
    image_shape_hw: tuple[int, int],
) -> tuple[DetectionResult, str | None]:
    """Keep SAM geometry but recover semantic head/tail from a sane VLM pair."""
    if detection.head_px is not None and detection.tail_px is not None:
        return detection, None
    raw_head = vlm_detection.head_px
    raw_tail = vlm_detection.tail_px
    if raw_head is None or raw_tail is None:
        return detection, "VLM did not provide both head/tail points"

    height, width = image_shape_hw
    bbox = vlm_detection.bbox.clamp(width, height)
    diagonal = float(np.hypot(bbox.width, bbox.height))
    margin = max(4.0, 0.18 * diagonal)

    def valid_coordinate(point: Any) -> bool:
        return bool(
            np.isfinite(point.x)
            and np.isfinite(point.y)
            and 0.0 <= point.x < width
            and 0.0 <= point.y < height
        )

    def near_box(point: Any) -> bool:
        return bool(
            bbox.x_min - margin <= point.x <= bbox.x_max + margin
            and bbox.y_min - margin <= point.y <= bbox.y_max + margin
        )

    # Schema-free VLM responses sometimes emit point arrays as [y, x] even
    # though object-shaped responses use {x, y}. Select an interpretation only
    # when both endpoints geometrically agree with this candidate's box.
    point_pairs = [
        (raw_head, raw_tail, "xy"),
        (
            PixelPoint(raw_head.y, raw_head.x),
            PixelPoint(raw_tail.y, raw_tail.x),
            "yx-array-recovery",
        ),
    ]
    selected_pair = next(
        (
            (candidate_head, candidate_tail, point_order)
            for candidate_head, candidate_tail, point_order in point_pairs
            if valid_coordinate(candidate_head)
            and valid_coordinate(candidate_tail)
            and near_box(candidate_head)
            and near_box(candidate_tail)
        ),
        None,
    )
    if selected_pair is None:
        return detection, "neither xy nor yx VLM head/tail agrees with the tube box"
    head, tail, point_order = selected_pair

    separation = float(np.hypot(head.x - tail.x, head.y - tail.y))
    if separation < max(8.0, 0.42 * diagonal):
        return detection, f"VLM head/tail separation is too small ({separation:.1f}px)"
    center_x, center_y = bbox.center
    opposite_dot = (head.x - center_x) * (tail.x - center_x) + (
        head.y - center_y
    ) * (tail.y - center_y)
    if opposite_dot >= 0.0:
        return detection, "VLM head/tail are not on opposite sides of the tube center"

    if detection.mask is not None and bool(np.any(detection.mask)):
        mask_y, mask_x = np.where(np.asarray(detection.mask, dtype=bool))
        max_mask_distance = max(6.0, 0.18 * diagonal)
        for name, point in (("head", head), ("tail", tail)):
            nearest = float(np.hypot(mask_x - point.x, mask_y - point.y).min())
            if nearest > max_mask_distance:
                return detection, f"VLM {name} is {nearest:.1f}px away from the SAM mask"

    notes = detection.notes
    if notes:
        notes += "; "
    notes += (
        "SAM head/tail unavailable; restored validated semantic head/tail from VLM "
        f"(point_order={point_order})"
    )
    return (
        replace(
            detection,
            notes=notes,
            source=f"{detection.source}+vlm-orientation-fallback",
            head_px=head,
            tail_px=tail,
        ),
        None,
    )


def _depth_config(config: Any) -> DepthEstimatorConfig:
    source = config.depth
    return DepthEstimatorConfig(
        min_depth_m=source.min_depth_m,
        max_depth_m=source.max_depth_m,
        inner_ratio=source.inner_ratio,
        min_samples=source.min_samples,
        fallback_min_samples=source.fallback_min_samples,
        max_expand_ratio=source.max_expand_ratio,
        expand_steps=source.expand_steps,
        strategy=source.strategy,
    )


def build_tube_result(
    *,
    detection: DetectionResult,
    frame: Any,
    config: Any,
    config_path: Path,
    run_id: str,
    index: int,
) -> dict[str, Any]:
    depth_config = _depth_config(config)
    position = estimate_position_from_depth(frame.depth_m, frame.intrinsics, detection.bbox, depth_config)
    orientation = estimate_orientation(
        frame.depth_m,
        frame.intrinsics,
        detection,
        OrientationEstimatorConfig(
            min_depth_m=config.depth.min_depth_m,
            max_depth_m=config.depth.max_depth_m,
        ),
    )
    camera_points: dict[str, Any] = {
        "bbox_center": (position.x_m, position.y_m, position.z_m),
        "head": orientation.head_m if orientation else None,
        "tail": orientation.tail_m if orientation else None,
    }
    if orientation and orientation.tail_to_head_points_m:
        camera_points.update(orientation.tail_to_head_points_m)
    points_base = points_base_from_calibration(
        camera_points,
        enabled=config.calibration.enabled,
        calibration_file=config.calibration.file,
        active_camera=config.calibration.active_camera,
        config_path=config_path,
    )
    if not points_base.get("available"):
        raise ValueError(f"base-frame calibration unavailable: {points_base.get('reason')}")
    bbox_center = points_base.get("bbox_center")
    if not isinstance(bbox_center, dict) or not isinstance(bbox_center.get("base"), dict):
        raise ValueError("required fallback point points_base.bbox_center.base is unavailable")
    return {
        "target": config.target.name,
        "found": True,
        "run_id": run_id,
        "inventory_index": index,
        "inventory_order": "image_bbox_center_x_ascending",
        "detection": detection.to_dict(),
        "position": position.to_dict(),
        "position_anchor": "bbox",
        "points_base": points_base,
        "orientation": orientation.to_dict() if orientation else None,
        "intrinsics": frame.intrinsics.to_dict(),
        "timestamp_ms": frame.timestamp_ms,
        "coordinate_frame": {
            "name": "RealSense color optical frame",
            "units": "meters",
            "x": "right",
            "y": "down",
            "z": "forward from camera",
        },
    }


def locate_rack_from_initial_frame(
    *,
    image_bgr: np.ndarray,
    frame: Any,
    config: Any,
    config_path: Path,
    raw_response_path: Path | None,
) -> dict[str, Any]:
    """Locate the fixed rack from the already captured initial RGB-D frame."""
    client = OpenRouterVLMClient.from_env(
        model=config.openrouter.model,
        timeout_s=config.openrouter.timeout_s,
    )
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
    if not detection.found:
        raise RuntimeError("test-tube rack was not found in the initial head-camera frame")
    position = estimate_position_from_depth(
        frame.depth_m,
        frame.intrinsics,
        detection.bbox,
        _depth_config(config),
    )
    position_base = position_base_from_calibration(
        position,
        enabled=config.calibration.enabled,
        calibration_file=config.calibration.file,
        active_camera=config.calibration.active_camera,
        config_path=config_path,
        position_anchor="bbox",
    )
    if not position_base.get("available"):
        raise RuntimeError(f"rack base-frame calibration unavailable: {position_base.get('reason')}")
    return {
        "target": config.target.name,
        "found": True,
        "source": "initial_shared_rgbd_frame",
        "detection": detection.to_dict(),
        "position": position.to_dict(),
        "position_anchor": "bbox",
        "position_base": position_base,
        "intrinsics": frame.intrinsics.to_dict(),
        "timestamp_ms": frame.timestamp_ms,
    }


def classify_ambiguous_rack_slots_once(
    slot_ids: list[str],
    numbered_overlay_bgr: np.ndarray,
    *,
    config: Any,
) -> dict[str, str]:
    """Classify all ambiguous numbered slots with one VLM request."""
    client = OpenRouterVLMClient.from_env(model=config.openrouter.model, timeout_s=config.openrouter.timeout_s)
    schema = {
        "type": "object",
        "properties": {"slots": {"type": "array", "items": {
            "type": "object",
            "properties": {
                "id": {"type": "string", "enum": slot_ids},
                "state": {"type": "string", "enum": ["empty", "occupied", "ambiguous"]},
            },
            "required": ["id", "state"],
            "additionalProperties": False,
        }}},
        "required": ["slots"],
        "additionalProperties": False,
    }
    payload: dict[str, Any] = {
        "model": client.model,
        "messages": [{"role": "user", "content": [
            {"type": "text", "text": (
                "Classify only these numbered rack slots: " + ", ".join(slot_ids) + ". "
                "empty means a clearly unobstructed dark circular opening; occupied means a tube, cap, or red cap dot; "
                "otherwise ambiguous. Return every requested id exactly once."
            )},
            {"type": "image_url", "image_url": {"url": _encode_bgr_as_data_url(numbered_overlay_bgr, config.openrouter.jpeg_quality)}},
        ]}],
        "temperature": 0.0,
        "max_tokens": 512,
        "response_format": {"type": "json_schema", "json_schema": {"name": "rack_slot_states", "strict": True, "schema": schema}},
    }
    if client.require_parameters:
        payload["provider"] = {"require_parameters": True}
    response = client._post_payload(payload)
    content, _finish_reason = _extract_message_content(response)
    parsed_objects = _extract_balanced_json_objects(content)
    if not parsed_objects:
        raise RuntimeError("rack occupancy VLM returned no JSON object")
    return {str(item["id"]): str(item["state"]) for item in parsed_objects[-1].get("slots", [])}


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    if not 0.0 <= args.nms_iou_threshold <= 1.0:
        raise ValueError("--nms-iou-threshold must be in [0, 1]")
    if args.max_tubes < 1:
        raise ValueError("--max-tubes must be >= 1")
    if args.min_axis_aspect_ratio < 1.0:
        raise ValueError("--min-axis-aspect-ratio must be >= 1")
    config_path = args.config.expanduser().resolve()
    config = load_config(config_path)
    rack_config_path = args.rack_config.expanduser().resolve()
    rack_config = load_config(rack_config_path)
    detector_config = _detector_config(config, args)
    if args.detector == "vlm" and os.environ.get(SOCKET_ENV):
        progress(f"using persistent SAM cache at {os.environ[SOCKET_ENV]}")
        detector = CachedSamRefiner(detector_config)
    else:
        detector = GroundedSamDetector(detector_config)

    if args.camera_service_socket is not None:
        progress(f"requesting fresh head RGB-D from persistent service={args.camera_service_socket}")
        cached = WristCameraClient(args.camera_service_socket, timeout_sec=10.0).frame(
            "head",
            max_age_ms=500.0,
        )
        frame = FrameBundle(
            color_bgr=np.asarray(cached["color_bgr"], dtype=np.uint8),
            depth_m=np.asarray(cached["depth_m"], dtype=np.float32),
            intrinsics=CameraIntrinsics(**cached["intrinsics"]),
            timestamp_ms=float(cached["camera_timestamp_ms"]),
        )
        progress(
            f"persistent head frame received generation={cached['generation']} "
            f"age_ms={cached['age_ms']:.1f}"
        )
    else:
        progress(
            f"opening RealSense serial={config.realsense.serial_number}, "
            f"reset_on_start={args.reset_realsense}, "
            f"stream={config.realsense.width}x{config.realsense.height}@{config.realsense.fps}"
        )
        with RealSenseCamera(
            width=config.realsense.width,
            height=config.realsense.height,
            fps=config.realsense.fps,
            serial_number=config.realsense.serial_number,
            visual_preset=config.realsense.visual_preset,
            reset_on_start=args.reset_realsense,
            reset_wait_s=config.realsense.reset_wait_s,
        ) as camera:
            progress(
                f"camera stream started; warming up {config.realsense.warmup_frames} frame(s) "
                f"with timeout={config.realsense.frame_timeout_ms}ms"
            )
            frame = camera.capture(
                warmup_frames=config.realsense.warmup_frames,
                timeout_ms=config.realsense.frame_timeout_ms,
                retries=config.realsense.capture_retries,
            )
    progress(f"RGB-D capture complete, timestamp_ms={frame.timestamp_ms:.3f}")

    args.output_dir.mkdir(parents=True, exist_ok=True)
    args.inventory_json.parent.mkdir(parents=True, exist_ok=True)
    if args.save_rgb:
        args.save_rgb.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.save_rgb), frame.color_bgr):
            raise RuntimeError(f"failed to save RGB image: {args.save_rgb}")
    if args.save_depth:
        args.save_depth.parent.mkdir(parents=True, exist_ok=True)
        np.save(args.save_depth, frame.depth_m)
    progress("initial RGB-D artifacts saved")

    progress("locating fixed rack from the same initial RGB-D frame")
    rack_result = locate_rack_from_initial_frame(
        image_bgr=frame.color_bgr,
        frame=frame,
        config=rack_config,
        config_path=rack_config_path,
        raw_response_path=args.save_rack_vlm_response,
    )
    args.rack_result_json.parent.mkdir(parents=True, exist_ok=True)
    args.rack_result_json.write_text(
        json.dumps(rack_result, ensure_ascii=False, indent=2),
        encoding="utf-8",
    )
    rack_xyz = rack_result["position_base"]
    progress(
        f"rack cached in base frame: x={rack_xyz['x_m']:+.4f}, "
        f"y={rack_xyz['y_m']:+.4f}, z={rack_xyz['z_m']:+.4f}"
    )

    rack_grid_json: Path | None = None
    rack_grid_overlay: Path | None = None
    if args.build_rack_grid:
        rack_grid_json = args.rack_grid_json or (args.output_dir.parent / "rack_grid.json")
        rack_grid_overlay = args.rack_grid_overlay or (args.output_dir.parent / "rack_grid_overlay.jpg")
        progress("fitting experimental fixed 3x4 rack grid and classifying initial slot occupancy")
        rack_grid = build_rack_grid(
            image_bgr=frame.color_bgr,
            depth_m=frame.depth_m,
            rack_result=rack_result,
            config=rack_config,
            config_path=rack_config_path,
            ambiguous_classifier=lambda ids, overlay: classify_ambiguous_rack_slots_once(
                ids,
                overlay,
                config=rack_config,
            ),
        )
        save_grid(rack_grid_json, rack_grid)
        rack_grid_overlay.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(rack_grid_overlay), draw_overlay(frame.color_bgr, rack_grid["slots"])):
            raise RuntimeError(f"failed to save rack grid overlay: {rack_grid_overlay}")
        progress(
            f"rack grid cached: slots=12 inliers={rack_grid['fit']['inlier_count']} "
            f"rms={rack_grid['fit']['rms_px']:.2f}px"
        )

    if args.detector == "vlm":
        progress("requesting all loose-tube boxes from VLM (one image request)")
        proposals: list[DetectionResult | GroundingCandidate] = detect_all_with_vlm(
            frame.color_bgr,
            config,
            min_confidence=args.min_confidence,
            nms_iou_threshold=args.nms_iou_threshold,
            max_tubes=args.max_tubes,
            raw_response_path=args.save_vlm_response,
        )
        progress(f"VLM returned {len(proposals)} filtered tube candidate(s)")
    else:
        progress("running GroundingDINO multi-candidate fallback")
        proposals = detect_all_candidates(
            detector,
            frame.color_bgr,
            config.target.name,
            min_confidence=args.min_confidence,
            min_axis_aspect_ratio=args.min_axis_aspect_ratio,
            nms_iou_threshold=args.nms_iou_threshold,
            max_tubes=args.max_tubes,
        )
        progress(f"GroundingDINO returned {len(proposals)} filtered candidate(s)")
    if not proposals:
        raise RuntimeError("no loose test-tube candidates found in the initial frame")

    run_id = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    tubes: list[dict[str, Any]] = []
    rejected: list[dict[str, Any]] = []
    for source_index, proposal in enumerate(proposals, start=1):
        progress(f"refining candidate {source_index}/{len(proposals)} with SAM and depth")
        if isinstance(proposal, DetectionResult):
            detection = detector.refine_detection_with_sam(frame.color_bgr, proposal)
            detection, fallback_rejection = restore_validated_vlm_orientation(
                detection,
                proposal,
                frame.color_bgr.shape[:2],
            )
            if fallback_rejection is None and detection.source.endswith(
                "+vlm-orientation-fallback"
            ):
                progress(
                    f"candidate {source_index}: SAM orientation unavailable; "
                    "using validated VLM head/tail"
                )
            elif detection.head_px is None or detection.tail_px is None:
                progress(
                    f"candidate {source_index}: VLM orientation fallback rejected: "
                    f"{fallback_rejection}"
                )
        else:
            detection = refine_candidate(detector, frame.color_bgr, proposal)
        try:
            result = build_tube_result(
                detection=detection,
                frame=frame,
                config=config,
                config_path=config_path,
                run_id=run_id,
                index=len(tubes) + 1,
            )
            orientation_ok = bool(
                result.get("orientation")
                and result["orientation"].get("head_px")
                and result["orientation"].get("tail_px")
            )
            if not orientation_ok:
                raise ValueError("head/tail pixel orientation is unavailable")
        except Exception as exc:
            rejected.append(
                {
                    "source_index": source_index,
                    "bbox": detection.bbox.to_dict(),
                    "confidence": detection.confidence,
                    "reason": str(exc),
                }
            )
            progress(f"candidate {source_index} rejected: {exc}")
            continue
        result["inventory_index"] = len(tubes) + 1
        result_path = args.output_dir / f"tube_{len(tubes) + 1:02d}.json"
        result_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
        center_x, center_y = detection.bbox.center
        preferred = result["points_base"].get("tail_to_head_1_5")
        preferred_available = bool(
            isinstance(preferred, dict) and isinstance(preferred.get("base"), dict)
        )
        tubes.append(
            {
                "index": len(tubes) + 1,
                "result_json": str(result_path.resolve()),
                "bbox_center_px": {"x": center_x, "y": center_y},
                "confidence": detection.confidence,
                "planned_grasp_point": "tail_to_head_1_5" if preferred_available else "bbox_center",
                "degraded_to_bbox_center": not preferred_available,
            }
        )
        progress(
            f"candidate {source_index} accepted as tube {len(tubes)}; "
            f"grasp_point={tubes[-1]['planned_grasp_point']}"
        )

    if not tubes:
        raise RuntimeError(f"all {len(proposals)} candidates were rejected: {rejected}")
    if rejected and not args.allow_rejected_candidates:
        raise RuntimeError(
            f"refusing partial inventory: {len(rejected)} of {len(proposals)} candidates "
            f"could not be localized safely; details={rejected}"
        )
    inventory = {
        "ok": True,
        "run_id": run_id,
        "capture_count": 1,
        "camera_timestamp_ms": frame.timestamp_ms,
        "ordering": "left_to_right_by_initial_image_bbox_center_x",
        "tube_count": len(tubes),
        "candidate_count": len(proposals),
        "tubes": tubes,
        "rejected_candidates": rejected,
        "config": str(config_path),
        "detector": args.detector,
        "initial_rgb": str(args.save_rgb.resolve()) if args.save_rgb else None,
        "initial_depth": str(args.save_depth.resolve()) if args.save_depth else None,
        "vlm_response": str(args.save_vlm_response.resolve()) if args.save_vlm_response else None,
        "rack_result_json": str(args.rack_result_json.resolve()),
        "rack_grid_json": None if rack_grid_json is None else str(rack_grid_json.resolve()),
        "rack_grid_overlay": None if rack_grid_overlay is None else str(rack_grid_overlay.resolve()),
        "rack_vlm_response": (
            str(args.save_rack_vlm_response.resolve()) if args.save_rack_vlm_response else None
        ),
    }
    args.inventory_json.write_text(json.dumps(inventory, ensure_ascii=False, indent=2), encoding="utf-8")
    progress(f"inventory complete: {len(tubes)} tube(s), json={args.inventory_json}")
    print(json.dumps(inventory, ensure_ascii=False, indent=None if args.compact else 2))
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        raise SystemExit(1)
