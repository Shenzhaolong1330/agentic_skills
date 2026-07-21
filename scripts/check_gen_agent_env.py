#!/usr/bin/env python3
"""Check the reproducible, offline harness environment."""

from __future__ import annotations

import importlib
from importlib.metadata import PackageNotFoundError, version as distribution_version
import json
import os
from pathlib import Path
import sys


REQUIRED = {
    "numpy": "numpy",
    "scipy": "scipy",
    "cv2": "cv2",
    "yaml": "yaml",
    "jsonschema": "jsonschema",
    "pytest": "pytest",
    "packaging": "packaging",
}
DIST_NAMES = {"cv2": "opencv-python-headless", "yaml": "PyYAML"}
REPORT_DIR = Path("/tmp/agentic_skills_gen_agent/S02_S03/environment")
REPORT_PATH = REPORT_DIR / "environment_report.json"


def package_version(module_name: str) -> str | None:
    try:
        module = importlib.import_module(module_name)
    except Exception:
        return None
    try:
        return distribution_version(DIST_NAMES.get(module_name, module_name))
    except PackageNotFoundError:
        return str(getattr(module, "__version__", "unknown"))


def main() -> int:
    packages = {name: package_version(module) for name, module in REQUIRED.items()}
    imports_ok = all(version is not None for version in packages.values())

    jsonschema_ok = False
    if imports_ok:
        try:
            from jsonschema import Draft202012Validator

            Draft202012Validator.check_schema({"$schema": Draft202012Validator.META_SCHEMA["$id"], "type": "object"})
            jsonschema_ok = True
        except Exception:
            jsonschema_ok = False

    repo_root = Path(__file__).resolve().parents[1]
    sys.path.insert(0, str(repo_root))
    try:
        __import__("agentic_skills_harness")
        harness_import_ok = True
    except Exception:
        harness_import_ok = False

    environment_name = os.environ.get("CONDA_DEFAULT_ENV") or os.environ.get("CONDA_PREFIX", "").rsplit("/", 1)[-1]
    report = {
        "environment_name": environment_name,
        "python_executable": sys.executable,
        "python_version": ".".join(map(str, sys.version_info[:3])),
        "packages": packages,
        "imports_ok": imports_ok and jsonschema_ok and harness_import_ok,
        "jsonschema_draft_2020_12": jsonschema_ok,
        "repository_root": str(repo_root),
        "harness_import_ok": harness_import_ok,
        "hardware_packages_intentionally_not_required": [
            "pyrealsense2",
            "rospy/rclpy",
            "torch",
            "transformers",
            "zerorpc",
            "robot vendor SDKs",
        ],
        "status": "PASS" if sys.version_info[:2] == (3, 11) and imports_ok and jsonschema_ok and harness_import_ok else "FAIL",
    }
    REPORT_DIR.mkdir(parents=True, exist_ok=True)
    REPORT_PATH.write_text(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(report, ensure_ascii=False, indent=2, sort_keys=True))
    return 0 if report["status"] == "PASS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
