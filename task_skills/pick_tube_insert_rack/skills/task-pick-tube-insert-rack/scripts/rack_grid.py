#!/usr/bin/env python3
"""Task-local 3x4 rack grid fitting, projection, and slot state helpers."""
from __future__ import annotations

import itertools
import json
import math
from pathlib import Path
from typing import Any, Callable, Mapping

import cv2
import numpy as np
import yaml


ROWS = 3
COLS = 4
VALID_STATES = {"empty", "reserved", "occupied", "ambiguous", "unknown"}


def slot_id(row: int, col: int) -> str:
    return f"r{row + 1}c{col + 1}"


def grid_points(origin: np.ndarray, col_step: np.ndarray, row_step: np.ndarray) -> np.ndarray:
    return np.asarray(
        [origin + col * col_step + row * row_step for row in range(ROWS) for col in range(COLS)],
        dtype=float,
    )


def _nearest_unique_errors(grid: np.ndarray, candidates: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    distances = np.linalg.norm(grid[:, None, :] - candidates[None, :, :], axis=2)
    pairs = sorted(
        ((float(distances[g, c]), g, c) for g in range(len(grid)) for c in range(len(candidates))),
        key=lambda item: item[0],
    )
    errors = np.full(len(grid), np.inf, dtype=float)
    assignment = np.full(len(grid), -1, dtype=int)
    used_candidates: set[int] = set()
    for distance, grid_index, candidate_index in pairs:
        if assignment[grid_index] >= 0 or candidate_index in used_candidates:
            continue
        errors[grid_index] = distance
        assignment[grid_index] = candidate_index
        used_candidates.add(candidate_index)
    return errors, assignment


def fit_grid_ransac(
    candidates_px: np.ndarray,
    *,
    max_error_px: float = 4.0,
    min_inliers: int = 8,
) -> dict[str, Any]:
    """Fit an affine 3x4 grid from unordered circle centers.

    Hypotheses are generated from candidate pairs and discrete row/column
    separations.  The best hypothesis is then least-squares refined using its
    unique inliers.
    """
    points = np.asarray(candidates_px, dtype=float).reshape(-1, 2)
    if len(points) < min_inliers:
        raise ValueError(f"need at least {min_inliers} circle candidates, got {len(points)}")
    threshold = float(max_error_px)
    best: tuple[int, float, np.ndarray, np.ndarray, np.ndarray, np.ndarray] | None = None
    for origin_index, origin in enumerate(points):
        others = [index for index in range(len(points)) if index != origin_index]
        for col_index, row_index in itertools.permutations(others, 2):
            col_delta = points[col_index] - origin
            row_delta = points[row_index] - origin
            for col_gap in range(1, COLS):
                col_step = col_delta / col_gap
                col_norm = float(np.linalg.norm(col_step))
                if not 18.0 <= col_norm <= 60.0 or abs(col_step[0]) < abs(col_step[1]):
                    continue
                if col_step[0] < 0:
                    col_step = -col_step
                for row_gap in range(1, ROWS):
                    row_step = row_delta / row_gap
                    row_norm = float(np.linalg.norm(row_step))
                    if not 18.0 <= row_norm <= 60.0 or abs(row_step[1]) < abs(row_step[0]):
                        continue
                    if row_step[1] < 0:
                        row_step = -row_step
                    cross_2d = float(col_step[0] * row_step[1] - col_step[1] * row_step[0])
                    if abs(cross_2d) < 0.35 * col_norm * row_norm:
                        continue
                    grid = grid_points(origin, col_step, row_step)
                    errors, assignment = _nearest_unique_errors(grid, points)
                    inliers = errors <= threshold
                    count = int(inliers.sum())
                    if count < min_inliers:
                        continue
                    rms = float(np.sqrt(np.mean(errors[inliers] ** 2)))
                    if best is None or (count, -rms) > (best[0], -best[1]):
                        best = (count, rms, origin.copy(), col_step.copy(), row_step.copy(), assignment)
    if best is None:
        raise ValueError(f"could not fit a 3x4 grid with >= {min_inliers} centers and <= {threshold:g}px error")

    _count, _rms, origin, col_step, row_step, assignment = best
    provisional = grid_points(origin, col_step, row_step)
    errors, assignment = _nearest_unique_errors(provisional, points)
    inlier_indices = np.flatnonzero(errors <= threshold)
    design = np.array([[1.0, index % COLS, index // COLS] for index in inlier_indices], dtype=float)
    observed = points[assignment[inlier_indices]]
    coefficients, *_ = np.linalg.lstsq(design, observed, rcond=None)
    origin, col_step, row_step = coefficients
    if not 18.0 <= float(np.linalg.norm(col_step)) <= 60.0 or not 18.0 <= float(np.linalg.norm(row_step)) <= 60.0:
        raise ValueError("refined rack grid pitch is outside the physical 18..60px range")
    fitted = grid_points(origin, col_step, row_step)
    errors, assignment = _nearest_unique_errors(fitted, points)
    inliers = errors <= threshold
    count = int(inliers.sum())
    rms = float(np.sqrt(np.mean(errors[inliers] ** 2)))
    if count < min_inliers or rms > threshold:
        raise ValueError(f"refined grid rejected: inliers={count}, rms={rms:.3f}px")
    return {
        "centers_px": fitted,
        "origin_px": origin,
        "col_step_px": col_step,
        "row_step_px": row_step,
        "inlier_count": count,
        "rms_px": rms,
        "matched_candidate_indices": [int(assignment[i]) if inliers[i] else None for i in range(ROWS * COLS)],
    }


def detect_circle_candidates(image_bgr: np.ndarray, rack_bbox: Mapping[str, Any]) -> np.ndarray:
    height, width = image_bgr.shape[:2]
    x1 = max(0, int(math.floor(float(rack_bbox["x_min"]) - 8)))
    y1 = max(0, int(math.floor(float(rack_bbox["y_min"]) - 8)))
    x2 = min(width, int(math.ceil(float(rack_bbox["x_max"]) + 8)))
    y2 = min(height, int(math.ceil(float(rack_bbox["y_max"]) + 8)))
    gray = cv2.cvtColor(image_bgr[y1:y2, x1:x2], cv2.COLOR_BGR2GRAY)
    gray = cv2.GaussianBlur(gray, (5, 5), 1.2)
    circles = cv2.HoughCircles(
        gray,
        cv2.HOUGH_GRADIENT,
        dp=1.2,
        minDist=12,
        param1=60,
        param2=10,
        minRadius=5,
        maxRadius=18,
    )
    points: list[np.ndarray] = []
    if circles is not None:
        circle_points = circles[0, :, :2].astype(float)
        circle_points[:, 0] += x1
        circle_points[:, 1] += y1
        points.extend(circle_points)

    # Dark lower-row holes often have too little edge contrast for Hough.  Add
    # local dark-center maxima; RANSAC rejects rack edges and duplicate peaks.
    gray_float = gray.astype(np.float32)
    contrast = cv2.GaussianBlur(gray_float, (0, 0), 8.0) - cv2.GaussianBlur(gray_float, (0, 0), 2.0)
    margin = 14
    if contrast.shape[0] > 2 * margin and contrast.shape[1] > 2 * margin:
        contrast[:margin, :] = -np.inf
        contrast[-margin:, :] = -np.inf
        contrast[:, :margin] = -np.inf
        contrast[:, -margin:] = -np.inf
    working = contrast.copy()
    for _ in range(30):
        _minimum, maximum, _min_location, location = cv2.minMaxLoc(working)
        if maximum < 1.5:
            break
        px, py = location
        candidate = np.array([px + x1, py + y1], dtype=float)
        if all(float(np.linalg.norm(candidate - existing)) >= 7.0 for existing in points):
            points.append(candidate)
        cv2.circle(working, (px, py), 9, float("-inf"), -1)

    # The rack bbox supplies a conservative spatial prior. Search a small
    # window around each physical 3x4 cell for the strongest local radial
    # contrast, which recovers low-contrast dark openings in the lower rows.
    bbox_width = float(rack_bbox["x_max"]) - float(rack_bbox["x_min"])
    bbox_height = float(rack_bbox["y_max"]) - float(rack_bbox["y_min"])
    full_gray = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2GRAY).astype(np.float32)
    radial = np.abs(cv2.GaussianBlur(full_gray, (0, 0), 7.0) - cv2.GaussianBlur(full_gray, (0, 0), 2.0))
    for row_fraction in (0.14, 0.38, 0.61):
        for col_fraction in (0.19, 0.39, 0.59, 0.79):
            nominal_x = int(round(float(rack_bbox["x_min"]) + col_fraction * bbox_width))
            nominal_y = int(round(float(rack_bbox["y_min"]) + row_fraction * bbox_height))
            window = radial[max(0, nominal_y - 5): nominal_y + 6, max(0, nominal_x - 5): nominal_x + 6]
            if window.size == 0:
                continue
            _minimum, _maximum, _min_location, location = cv2.minMaxLoc(window)
            candidate = np.array([nominal_x - 5 + location[0], nominal_y - 5 + location[1]], dtype=float)
            if all(float(np.linalg.norm(candidate - existing)) >= 5.0 for existing in points):
                points.append(candidate)
    result = np.asarray(points, dtype=float).reshape(-1, 2)
    interior = (
        (result[:, 0] >= float(rack_bbox["x_min"]) + 10.0)
        & (result[:, 0] <= float(rack_bbox["x_max"]) - 10.0)
        & (result[:, 1] >= float(rack_bbox["y_min"]) + 10.0)
        & (result[:, 1] <= float(rack_bbox["y_max"]) - 8.0)
    )
    return result[interior]


def _base_T_camera(config: Any, config_path: Path) -> np.ndarray:
    from object_locator.transforms import base_T_camera_from_calibration

    calibration_path = (config_path.parent / config.calibration.file).resolve()
    raw = yaml.safe_load(calibration_path.read_text(encoding="utf-8"))
    transform, _metadata = base_T_camera_from_calibration(
        raw,
        active_camera=config.calibration.active_camera,
        calibration_path=calibration_path,
    )
    return np.asarray(transform.matrix, dtype=float)


def pixel_ray_plane_base(
    pixel: np.ndarray,
    intrinsics: Mapping[str, Any],
    base_T_camera: np.ndarray,
    plane_z_base_m: float,
) -> np.ndarray:
    u, v = np.asarray(pixel, dtype=float).reshape(2)
    ray_camera = np.array(
        [(u - float(intrinsics["ppx"])) / float(intrinsics["fx"]),
         (v - float(intrinsics["ppy"])) / float(intrinsics["fy"]), 1.0],
        dtype=float,
    )
    transform = np.asarray(base_T_camera, dtype=float).reshape(4, 4)
    origin = transform[:3, 3]
    ray = transform[:3, :3] @ ray_camera
    scale = (float(plane_z_base_m) - origin[2]) / ray[2]
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError("pixel ray does not intersect rack plane in front of camera")
    return origin + scale * ray


def classify_slot(
    image_bgr: np.ndarray,
    depth_m: np.ndarray,
    center_px: np.ndarray,
    *,
    radius_px: int = 9,
) -> tuple[str, float, dict[str, float]]:
    u, v = np.rint(center_px).astype(int)
    y, x = np.ogrid[: image_bgr.shape[0], : image_bgr.shape[1]]
    mask = (x - u) ** 2 + (y - v) ** 2 <= radius_px**2
    pixels = image_bgr[mask]
    hsv = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2HSV)[mask]
    depth = depth_m[mask]
    red = ((hsv[:, 0] <= 10) | (hsv[:, 0] >= 170)) & (hsv[:, 1] >= 90) & (hsv[:, 2] >= 60)
    red_fraction = float(np.mean(red)) if len(red) else 0.0
    dark_fraction = float(np.mean(np.mean(pixels, axis=1) < 70)) if len(pixels) else 0.0
    valid_depth = depth[np.isfinite(depth) & (depth > 0.05)]
    depth_spread = float(np.percentile(valid_depth, 90) - np.percentile(valid_depth, 10)) if len(valid_depth) >= 8 else math.inf
    metrics = {"red_fraction": red_fraction, "dark_fraction": dark_fraction, "depth_spread_m": depth_spread}
    if red_fraction >= 0.025:
        return "occupied", min(0.99, 0.75 + 4.0 * red_fraction), metrics
    if dark_fraction >= 0.48 and depth_spread <= 0.035:
        return "empty", min(0.98, 0.55 + 0.7 * dark_fraction), metrics
    return "ambiguous", 0.35, metrics


def build_rack_grid(
    *,
    image_bgr: np.ndarray,
    depth_m: np.ndarray,
    rack_result: Mapping[str, Any],
    config: Any,
    config_path: Path,
    ambiguous_classifier: Callable[[list[str], np.ndarray], Mapping[str, str]] | None = None,
) -> dict[str, Any]:
    bbox = rack_result["detection"]["bbox"]
    candidates = detect_circle_candidates(image_bgr, bbox)
    fit = fit_grid_ransac(candidates, max_error_px=4.0, min_inliers=8)
    plane_z = float(rack_result["position_base"]["z_m"])
    transform = _base_T_camera(config, config_path)
    intrinsics = rack_result["intrinsics"]
    slots: list[dict[str, Any]] = []
    ambiguous_ids: list[str] = []
    for index, center in enumerate(fit["centers_px"]):
        row, col = divmod(index, COLS)
        state, confidence, metrics = classify_slot(image_bgr, depth_m, center)
        identifier = slot_id(row, col)
        if state == "ambiguous":
            ambiguous_ids.append(identifier)
        base = pixel_ray_plane_base(center, intrinsics, transform, plane_z)
        slots.append({
            "id": identifier,
            "row": row + 1,
            "col": col + 1,
            "center_px": {"x": float(center[0]), "y": float(center[1])},
            "position_base_m": base.tolist(),
            "state": state,
            "confidence": float(confidence),
            "source": "rgb_depth" if state != "ambiguous" else "rgb_depth_ambiguous",
            "metrics": metrics,
        })
    if ambiguous_ids and ambiguous_classifier is not None:
        decisions = ambiguous_classifier(ambiguous_ids, draw_overlay(image_bgr, slots))
        for slot in slots:
            decision = decisions.get(slot["id"])
            if slot["id"] in ambiguous_ids and decision in {"empty", "occupied"}:
                slot.update({"state": decision, "confidence": 0.7, "source": "numbered_overlay_vlm"})
    return {
        "schema_version": 1,
        "shape": {"rows": ROWS, "cols": COLS},
        "ordering": "head_image_row_major_top_to_bottom_left_to_right",
        "rack_plane_z_base_m": plane_z,
        "fit": {
            "candidate_count": int(len(candidates)),
            "inlier_count": fit["inlier_count"],
            "rms_px": fit["rms_px"],
            "origin_px": fit["origin_px"].tolist(),
            "col_step_px": fit["col_step_px"].tolist(),
            "row_step_px": fit["row_step_px"].tolist(),
            "max_rms_px": 4.0,
        },
        "slots": slots,
    }


def draw_overlay(image_bgr: np.ndarray, slots: list[Mapping[str, Any]]) -> np.ndarray:
    overlay = image_bgr.copy()
    colors = {"empty": (0, 220, 0), "occupied": (0, 0, 255), "ambiguous": (0, 200, 255), "unknown": (128, 128, 128), "reserved": (255, 180, 0)}
    for slot in slots:
        center = slot["center_px"]
        point = (int(round(float(center["x"]))), int(round(float(center["y"]))))
        color = colors.get(str(slot["state"]), (255, 255, 255))
        cv2.circle(overlay, point, 10, color, 2, cv2.LINE_AA)
        cv2.putText(overlay, str(slot["id"]), (point[0] + 8, point[1] - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.42, color, 1, cv2.LINE_AA)
    return overlay


def transition_slot(grid: dict[str, Any], identifier: str, new_state: str) -> dict[str, Any]:
    if new_state not in VALID_STATES:
        raise ValueError(f"invalid slot state: {new_state}")
    allowed = {
        "empty": {"reserved"},
        "reserved": {"occupied", "unknown"},
        "ambiguous": {"unknown"},
        "occupied": set(),
        "unknown": set(),
    }
    for slot in grid["slots"]:
        if slot["id"] != identifier:
            continue
        old_state = str(slot["state"])
        if new_state not in allowed.get(old_state, set()):
            raise ValueError(f"illegal slot transition {identifier}: {old_state} -> {new_state}")
        slot["state"] = new_state
        return slot
    raise KeyError(f"unknown rack slot: {identifier}")


def reserve_next_empty(grid: dict[str, Any]) -> str:
    for slot in grid["slots"]:
        if slot["state"] == "empty":
            transition_slot(grid, slot["id"], "reserved")
            return str(slot["id"])
    raise RuntimeError("rack grid contains no confirmed empty slot")


def save_grid(path: Path, grid: Mapping[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(grid, ensure_ascii=False, indent=2), encoding="utf-8")
