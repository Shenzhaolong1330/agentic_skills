from __future__ import annotations

import json
from pathlib import Path
import sys

import cv2
import numpy as np
import pytest


SCRIPT_DIR = Path(__file__).resolve().parents[1] / "scripts"
OBJECT_LOCATOR_ROOT = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
sys.path[:0] = [str(SCRIPT_DIR), str(OBJECT_LOCATOR_ROOT / "src")]

from rack_grid import (  # noqa: E402
    build_rack_grid,
    fit_grid_ransac,
    reserve_next_empty,
    slot_id,
    transition_slot,
)
import wrist_cached_perception as wrist_perception  # noqa: E402
from wrist_cached_perception import CachedPerceptionError, project_base_to_pixel  # noqa: E402


def test_fit_interpolates_missing_centers_and_orders_row_major() -> None:
    origin = np.array([120.0, 80.0])
    col = np.array([31.0, 1.0])
    row = np.array([-1.5, 27.0])
    full = np.array([origin + c * col + r * row for r in range(3) for c in range(4)])
    observed = np.delete(full, [2, 9, 11], axis=0)
    fitted = fit_grid_ransac(observed, min_inliers=8, max_error_px=4.0)
    assert fitted["inlier_count"] == 9
    assert fitted["rms_px"] < 0.01
    assert np.allclose(fitted["centers_px"], full, atol=0.01)
    assert [slot_id(r, c) for r in range(3) for c in range(4)] == [
        "r1c1", "r1c2", "r1c3", "r1c4",
        "r2c1", "r2c2", "r2c3", "r2c4",
        "r3c1", "r3c2", "r3c3", "r3c4",
    ]


def test_slot_state_machine_never_reuses_unknown() -> None:
    grid = {"slots": [{"id": "r1c1", "state": "empty"}, {"id": "r1c2", "state": "empty"}]}
    assert reserve_next_empty(grid) == "r1c1"
    transition_slot(grid, "r1c1", "unknown")
    assert reserve_next_empty(grid) == "r1c2"
    transition_slot(grid, "r1c2", "occupied")
    with pytest.raises(RuntimeError):
        reserve_next_empty(grid)


def test_project_base_to_pixel_round_trip_center() -> None:
    intrinsics = {"fx": 600.0, "fy": 600.0, "ppx": 320.0, "ppy": 240.0}
    assert np.allclose(project_base_to_pixel(np.array([0.0, 0.0, 1.0]), np.eye(4), intrinsics), [320.0, 240.0])


def test_cached_correction_rejects_more_than_15mm(monkeypatch: pytest.MonkeyPatch) -> None:
    from types import SimpleNamespace
    from object_locator.models import BoundingBox

    class FakeSam:
        def refine_detection_with_sam(self, _image, _detection):
            return SimpleNamespace(bbox=BoundingBox(x_min=332.0, y_min=232.0, x_max=348.0, y_max=248.0), mask=None)

    monkeypatch.setattr(wrist_perception, "_local_candidate", lambda _image, _predicted: {
        "center_px": np.array([340.0, 240.0]), "radius_px": 8.0,
        "bbox": {"x_min": 332.0, "y_min": 232.0, "x_max": 348.0, "y_max": 248.0}, "roundness": 1.0,
        "red_fraction": 0.0, "dark_fraction": 1.0, "roi": [0, 0, 1, 1],
    })
    with pytest.raises(CachedPerceptionError) as error:
        wrist_perception.resolve_cached_slot_from_frame(
            image_bgr=np.zeros((480, 640, 3), dtype=np.uint8),
            depth_m=np.ones((480, 640), dtype=np.float32),
            intrinsics={"fx": 600.0, "fy": 600.0, "ppx": 320.0, "ppy": 240.0},
            base_T_camera=np.eye(4),
            cached_position_base_m=np.array([0.0, 0.0, 1.0]),
            rack_plane_z_base_m=1.0,
            hole_config=None,
            sam_detector=FakeSam(),
            max_correction_m=0.015,
        )
    assert error.value.report["correction_xy_m"] > 0.015


def test_local_failure_uses_same_frame_vlm_fallback(monkeypatch: pytest.MonkeyPatch) -> None:
    monkeypatch.setattr(wrist_perception, "_local_candidate", lambda *_args: (_ for _ in ()).throw(ValueError("uncertain")))
    monkeypatch.setattr(wrist_perception, "_same_frame_vlm_fallback", lambda *_args: (
        {"center_px": np.array([320.0, 240.0]), "bbox": {}, "detection": {}},
        {"vlm": 0.1, "sam": 0.02},
    ))
    corrected, report = wrist_perception.resolve_cached_slot_from_frame(
        image_bgr=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.ones((480, 640), dtype=np.float32),
        intrinsics={"fx": 600.0, "fy": 600.0, "ppx": 320.0, "ppy": 240.0},
        base_T_camera=np.eye(4),
        cached_position_base_m=np.array([0.0, 0.0, 1.0]),
        rack_plane_z_base_m=1.0,
        hole_config=None,
        sam_detector=None,
    )
    assert np.allclose(corrected, [0.0, 0.0, 1.0])
    assert report["fallback_used"] is True
    assert report["source"] == "same-frame-vlm+sam"


def test_legacy_locator_runs_on_supplied_rgbd_frame(monkeypatch: pytest.MonkeyPatch) -> None:
    from types import SimpleNamespace
    from object_locator.models import BoundingBox, DetectionResult
    import object_locator.openrouter_vlm as openrouter_vlm

    detection = DetectionResult(
        found=True,
        label="empty test tube rack hole",
        confidence=0.9,
        bbox=BoundingBox(x_min=300.0, y_min=220.0, x_max=340.0, y_max=260.0),
        source="fake-vlm",
    )

    class FakeClient:
        def detect_object(self, *_args, **_kwargs):
            return detection

    monkeypatch.setattr(openrouter_vlm.OpenRouterVLMClient, "from_env", lambda **_kwargs: FakeClient())
    depth = SimpleNamespace(
        min_depth_m=0.05, max_depth_m=3.0, inner_ratio=0.7, min_samples=8,
        fallback_min_samples=3, max_expand_ratio=2.0, expand_steps=2, strategy="median",
    )
    config = SimpleNamespace(
        target=SimpleNamespace(name="empty test tube rack hole", description="one empty hole"),
        openrouter=SimpleNamespace(
            model="fake", timeout_s=1.0, jpeg_quality=90, use_json_schema=True,
            temperature=0.0, max_tokens=128, retry_without_json_schema=False,
        ),
        depth=depth,
    )
    result, timings = wrist_perception.locate_hole_with_vlm_sam_frame(
        image_bgr=np.zeros((480, 640, 3), dtype=np.uint8),
        depth_m=np.ones((480, 640), dtype=np.float32),
        intrinsics={"width": 640, "height": 480, "fx": 600.0, "fy": 600.0, "ppx": 320.0, "ppy": 240.0},
        config=config,
        sam_detector=SimpleNamespace(refine_detection_with_sam=lambda _image, item: item),
        camera_timestamp_ms=123.0,
    )
    assert result["position_anchor"] == "bbox"
    assert result["position"]["depth_m"] == pytest.approx(1.0)
    assert set(timings) == {"vlm", "sam", "depth"}


def test_latest_head_archive_builds_twelve_slots() -> None:
    run = Path("/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260714_235621")
    if not (run / "initial_tube_inventory_rgb.jpg").exists():
        pytest.skip("latest hardware archive is not present")
    from object_locator.config import load_config

    config_path = OBJECT_LOCATOR_ROOT / "config_rack_center_vlm.yaml"
    grid = build_rack_grid(
        image_bgr=cv2.imread(str(run / "initial_tube_inventory_rgb.jpg")),
        depth_m=np.load(run / "initial_tube_inventory_depth.npy"),
        rack_result=json.loads((run / "rack_detection.json").read_text()),
        config=load_config(config_path),
        config_path=config_path,
    )
    assert len(grid["slots"]) == 12
    assert grid["fit"]["inlier_count"] >= 8
    assert grid["fit"]["rms_px"] <= 4.0
