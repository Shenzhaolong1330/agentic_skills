#!/usr/bin/env python3
"""Bind a wrist object-locator config to the current-run TCP calibration."""

from __future__ import annotations

import argparse
from pathlib import Path
import shutil

import yaml


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-config", type=Path, required=True)
    parser.add_argument("--runtime-calibration-dir", type=Path, required=True)
    parser.add_argument("--output-config", type=Path, required=True)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    source_config = args.source_config.expanduser().resolve()
    runtime_dir = args.runtime_calibration_dir.expanduser().resolve()
    output_config = args.output_config.expanduser().resolve()
    source = yaml.safe_load(source_config.read_text(encoding="utf-8")) or {}
    if not isinstance(source, dict):
        raise ValueError(f"source config must be a YAML object: {source_config}")

    source_calibration = source.get("calibration", {}).get("file")
    if not source_calibration:
        raise ValueError(f"source config has no calibration.file: {source_config}")
    source_calibration_path = Path(str(source_calibration))
    if not source_calibration_path.is_absolute():
        source_calibration_path = source_config.parent / source_calibration_path
    source_calibration_path = source_calibration_path.resolve()

    runtime_calibration = runtime_dir / "calibration"
    runtime_calibration.mkdir(parents=True, exist_ok=True)
    shutil.copy2(source_calibration_path, runtime_calibration / "extrinsics.yaml")
    for filename in ("base_T_flange.yaml", "base_T_left_flange.yaml"):
        path = runtime_calibration / filename
        if not path.exists():
            raise FileNotFoundError(f"current-run calibration file is missing: {path}")

    calibration = source.setdefault("calibration", {})
    if not isinstance(calibration, dict):
        raise ValueError("source config calibration section must be a YAML object")
    calibration["enabled"] = True
    calibration["file"] = str((runtime_calibration / "extrinsics.yaml").resolve())
    output_config.parent.mkdir(parents=True, exist_ok=True)
    output_config.write_text(yaml.safe_dump(source, sort_keys=False), encoding="utf-8")
    print(output_config)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
