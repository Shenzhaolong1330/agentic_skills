#!/usr/bin/env python3
"""Run the atomic object-locator while delegating SAM to the task cache."""
from __future__ import annotations

from pathlib import Path
import sys


OBJECT_LOCATOR_ROOT = Path("/home/deepcybo/agentic_skills/atomic_skills/object_locator")
sys.path.insert(0, str(OBJECT_LOCATOR_ROOT / "src"))

from object_locator import cli  # noqa: E402
from object_locator.grounded_sam_detector import GroundedSamConfig  # noqa: E402

from sam_cache_service import CachedSamRefiner  # noqa: E402


def _refine_with_cache(color_bgr, runtime: dict, detection):
    refiner = CachedSamRefiner(
        GroundedSamConfig(
            grounding_model=runtime["grounding_model"],
            sam_model=runtime["sam_model"],
            text_prompt=runtime["grounded_sam_text_prompt"],
            selection=runtime["grounded_sam_selection"],
            box_threshold=runtime["grounded_sam_box_threshold"],
            text_threshold=runtime["grounded_sam_text_threshold"],
            device=runtime["grounded_sam_device"],
            use_sam=True,
            refine_bbox_with_mask=runtime["grounded_sam_refine_bbox_with_mask"],
            min_box_area_px=runtime["grounded_sam_min_box_area_px"],
            max_box_area_ratio=runtime["grounded_sam_max_box_area_ratio"],
            min_mask_area_px=runtime["grounded_sam_min_mask_area_px"],
            cap_endpoint_rule=runtime["grounded_sam_cap_endpoint_rule"],
            cap_dark_threshold=runtime["grounded_sam_cap_dark_threshold"],
            cap_min_area_px=runtime["grounded_sam_cap_min_area_px"],
        )
    )
    return refiner.refine_detection_with_sam(color_bgr, detection)


def main() -> int:
    cli._refine_vlm_detection_with_sam = _refine_with_cache
    return cli.main()


if __name__ == "__main__":
    raise SystemExit(main())
