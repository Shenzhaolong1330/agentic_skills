"""Fixed RealSense and object-locator binding for the S10H read-only probe.

Importing and constructing this module is hardware-free.  Camera processes are
started only by :meth:`FixedS10SceneBackend.capture_h1_observations`, after the
operator runner has supplied its already-authorized ``hardware_allowed`` value.
The executable, scripts, camera serials, locator preset, calibration sources,
working directory, argument layout, and output layout are not caller-selectable.
"""

from __future__ import annotations

from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass
import ast
import hashlib
import json
import math
from pathlib import Path
import subprocess
from typing import Any

import yaml

from agentic_skills_harness.live.fingerprint import digest_json


FIXED_SCENE_BACKEND_ID = "realsense_object_locator.s10.v1"
FIXED_VIEWER_RELATIVE_PATH = Path(
    "atomic_skills/state_realsense_viewer/skills/"
    "atomic-state-realsense-viewer/scripts/view_state_realsense.py"
)
FIXED_REALSENSE_PYTHON = Path("/home/deepcybo/miniconda3/bin/python")
FIXED_OBJECT_LOCATOR_ROOT = Path(
    "/home/deepcybo/agentic_skills/atomic_skills/object_locator"
)
FIXED_OBJECT_LOCATOR_EXECUTABLE = (
    FIXED_OBJECT_LOCATOR_ROOT / ".venv/bin/object-locator"
)
FIXED_OBJECT_LOCATOR_EDITABLE_LINK = (
    FIXED_OBJECT_LOCATOR_ROOT
    / ".venv/lib/python3.10/site-packages/"
    "__editable__.realsense_vlm_object_locator-0.1.0.pth"
)
FIXED_OBJECT_LOCATOR_CONFIG = (
    FIXED_OBJECT_LOCATOR_ROOT / "config_rack_center_grounded_sam.yaml"
)
FIXED_OBJECT_LOCATOR_VLM_CONFIG = (
    FIXED_OBJECT_LOCATOR_ROOT / "config_rack_center_vlm.yaml"
)
FIXED_LOCATOR_MODES = {
    "grounded-sam": {
        "config": FIXED_OBJECT_LOCATOR_CONFIG,
        "detector": "grounded_sam",
    },
    "openrouter-vlm": {
        "config": FIXED_OBJECT_LOCATOR_VLM_CONFIG,
        "detector": "vlm",
    },
}
FIXED_OBJECT_LOCATOR_SOURCE = (
    FIXED_OBJECT_LOCATOR_ROOT / "src/object_locator/cli.py"
)
FIXED_OBJECT_LOCATOR_REALSENSE_SOURCE = (
    FIXED_OBJECT_LOCATOR_ROOT / "src/object_locator/realsense_camera.py"
)
FIXED_OBJECT_LOCATOR_EXTRINSICS = (
    FIXED_OBJECT_LOCATOR_ROOT / "calibration/extrinsics.yaml"
)
FIXED_CALIBRATION_SOURCES = (
    FIXED_OBJECT_LOCATOR_EXTRINSICS,
    Path(
        "/home/deepcybo/Le-nero/dual_arm_teleop/"
        "calibration/all_camera_calibration_4.json"
    ),
    Path(
        "/home/deepcybo/dual_arm_camera_calibration/"
        "calibration/head_eye_result/head_to_base.json"
    ),
)
FIXED_CAMERA_SERIALS = {
    "head": "348522072761",
    "left_wrist": "347622074336",
    "right_wrist": "337322072568",
}
FIXED_CAMERA_SAMPLE_COUNT = 5
FIXED_LOCATOR_SAMPLE_COUNT = 5
FIXED_PROCESS_TIMEOUT_S = 180.0
FIXED_TARGET_NAME = "black test tube rack"


class SceneBindingError(ValueError):
    """The host or local acceptance config does not match the fixed binding."""


class SceneProbeError(RuntimeError):
    """A fixed scene process or its output failed validation."""


@dataclass(frozen=True)
class ProcessResult:
    returncode: int
    stdout: str
    stderr: str


ProcessRunner = Callable[[Sequence[str], Path, float], ProcessResult]


def _sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _sha256_concatenated(paths: Sequence[Path]) -> str:
    digest = hashlib.sha256()
    for path in paths:
        with path.open("rb") as stream:
            for chunk in iter(lambda: stream.read(1024 * 1024), b""):
                digest.update(chunk)
    return "sha256:" + digest.hexdigest()


def _sha256_python_tree(root: Path) -> str:
    digest = hashlib.sha256()
    paths = sorted(root.glob("*.py"))
    if not paths:
        raise SceneBindingError("fixed_scene_locator_source_tree_missing")
    for path in paths:
        digest.update(path.name.encode("utf-8"))
        digest.update(b"\0")
        digest.update(path.read_bytes())
        digest.update(b"\0")
    return digest.hexdigest()


def _as_mapping(value: Any) -> Mapping[str, Any]:
    return value if isinstance(value, Mapping) else {}


def _finite_number(value: Any, name: str) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise SceneProbeError(f"{name}_must_be_finite")
    return result


def _parse_camera_serial_bundle(value: Any) -> dict[str, str]:
    if not isinstance(value, str):
        raise SceneBindingError("fixed_scene_camera_serial_bundle_required")
    parsed: dict[str, str] = {}
    for item in value.split(";"):
        if item.count("=") != 1:
            raise SceneBindingError("fixed_scene_camera_serial_bundle_invalid")
        name, serial = (part.strip() for part in item.split("=", 1))
        if not name or not serial or name in parsed:
            raise SceneBindingError("fixed_scene_camera_serial_bundle_invalid")
        parsed[name] = serial
    if parsed != FIXED_CAMERA_SERIALS:
        raise SceneBindingError("fixed_scene_camera_serial_mismatch")
    return parsed


def _viewer_camera_serials(viewer_path: Path) -> dict[str, str]:
    tree = ast.parse(viewer_path.read_text(encoding="utf-8"))
    for node in tree.body:
        if (
            isinstance(node, ast.AnnAssign)
            and isinstance(node.target, ast.Name)
            and node.target.id == "CAMERAS"
        ):
            cameras = ast.literal_eval(node.value)
            return {
                str(name): str(spec["serial_number"])
                for name, spec in cameras.items()
            }
    raise SceneBindingError("fixed_scene_viewer_camera_map_missing")


def _production_process_runner(
    command: Sequence[str],
    cwd: Path,
    timeout_s: float,
) -> ProcessResult:
    completed = subprocess.run(
        list(command),
        cwd=cwd,
        text=True,
        capture_output=True,
        check=False,
        timeout=timeout_s,
    )
    return ProcessResult(completed.returncode, completed.stdout, completed.stderr)


def _read_json_object(path: Path, name: str) -> dict[str, Any]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SceneProbeError(f"{name}_json_unavailable") from exc
    if not isinstance(value, dict):
        raise SceneProbeError(f"{name}_must_be_object")
    return value


def _artifact_path(value: Any, root: Path, name: str) -> str:
    if not isinstance(value, str) or not value:
        raise SceneProbeError(f"{name}_artifact_path_missing")
    path = Path(value).resolve()
    if not path.is_relative_to(root) or not path.is_file():
        raise SceneProbeError(f"{name}_artifact_path_untrusted")
    return str(path)


def _strictly_increasing(values: Sequence[float]) -> bool:
    return all(second > first for first, second in zip(values, values[1:]))


@dataclass(frozen=True)
class S10SceneBinding:
    """Validated metadata for the one accepted S10 scene implementation."""

    repo_root: Path
    viewer_path: Path
    realsense_python: Path
    locator_root: Path
    locator_executable: Path
    locator_mode: str
    locator_config: Path
    detector_mode: str
    locator_source: Path
    locator_realsense_source: Path
    locator_extrinsics: Path
    artifact_root: Path
    calibration_hash: str
    source_sha256: Mapping[str, str]

    @classmethod
    def from_config(
        cls,
        repo_root: Path,
        config: Mapping[str, Any],
        *,
        locator_mode: str | None = None,
    ) -> "S10SceneBinding":
        repo = repo_root.resolve()
        viewer_candidate = repo / FIXED_VIEWER_RELATIVE_PATH
        viewer = viewer_candidate.resolve()
        if (
            not viewer.is_file()
            or viewer_candidate.is_symlink()
            or not viewer.is_relative_to(repo)
        ):
            raise SceneBindingError("fixed_scene_viewer_missing_or_untrusted")

        acceptance = _as_mapping(config.get("acceptance"))
        selected_mode = locator_mode or acceptance.get(
            "scene_locator_mode",
            "grounded-sam",
        )
        if selected_mode not in FIXED_LOCATOR_MODES:
            raise SceneBindingError("fixed_scene_locator_mode_invalid")
        mode_spec = FIXED_LOCATOR_MODES[str(selected_mode)]
        locator_config_path = Path(mode_spec["config"])
        detector_mode = str(mode_spec["detector"])

        fixed_files = (
            FIXED_REALSENSE_PYTHON,
            FIXED_OBJECT_LOCATOR_EXECUTABLE,
            FIXED_OBJECT_LOCATOR_EDITABLE_LINK,
            *(Path(spec["config"]) for spec in FIXED_LOCATOR_MODES.values()),
            FIXED_OBJECT_LOCATOR_SOURCE,
            FIXED_OBJECT_LOCATOR_REALSENSE_SOURCE,
            *FIXED_CALIBRATION_SOURCES,
        )
        if any(not path.is_file() for path in fixed_files):
            raise SceneBindingError("fixed_scene_external_dependency_missing")

        hardware = _as_mapping(config.get("hardware"))
        configured_serials = _parse_camera_serial_bundle(
            hardware.get("camera_serial")
        )
        if _viewer_camera_serials(viewer) != configured_serials:
            raise SceneBindingError("fixed_scene_viewer_camera_serial_mismatch")

        locator_config = yaml.safe_load(
            locator_config_path.read_text(encoding="utf-8")
        )
        if not isinstance(locator_config, dict):
            raise SceneBindingError("fixed_scene_locator_config_invalid")
        detector = _as_mapping(locator_config.get("detector"))
        realsense = _as_mapping(locator_config.get("realsense"))
        calibration = _as_mapping(locator_config.get("calibration"))
        target = _as_mapping(locator_config.get("target"))
        output = _as_mapping(locator_config.get("output"))
        resolved_calibration = (
            locator_config_path.parent
            / str(calibration.get("file", ""))
        ).resolve()
        locator_checks = (
            detector.get("mode") == detector_mode,
            str(realsense.get("serial_number")) == configured_serials["head"],
            realsense.get("reset_on_start") is False,
            calibration.get("enabled") is True,
            calibration.get("active_camera") == "head",
            resolved_calibration == FIXED_OBJECT_LOCATOR_EXTRINSICS.resolve(),
            target.get("name") == FIXED_TARGET_NAME,
            output.get("json") is True,
        )
        if not all(locator_checks):
            raise SceneBindingError("fixed_scene_locator_preset_mismatch")

        configured_calibration_hash = hardware.get("calibration_hash")
        if (
            not isinstance(configured_calibration_hash, str)
            or acceptance.get("calibration_hash") != configured_calibration_hash
        ):
            raise SceneBindingError("fixed_scene_calibration_binding_mismatch")
        actual_calibration_hash = _sha256_concatenated(
            FIXED_CALIBRATION_SOURCES
        )
        if configured_calibration_hash != actual_calibration_hash:
            raise SceneBindingError("fixed_scene_calibration_source_digest_mismatch")

        artifact_value = acceptance.get("artifact_root")
        if (
            not isinstance(artifact_value, str)
            or not Path(artifact_value).is_absolute()
        ):
            raise SceneBindingError("fixed_scene_artifact_root_required")
        artifact_root = Path(artifact_value).resolve()

        locator_entrypoint = FIXED_OBJECT_LOCATOR_EXECUTABLE.read_text(
            encoding="utf-8"
        )
        if not locator_entrypoint.startswith(
            "#!/home/deepcybo/agentic_skills/atomic_skills/"
            "object_locator/.venv/bin/python\n"
        ):
            raise SceneBindingError("fixed_scene_locator_entrypoint_mismatch")
        editable_source = FIXED_OBJECT_LOCATOR_EDITABLE_LINK.read_text(
            encoding="utf-8"
        ).strip()
        if editable_source != str(FIXED_OBJECT_LOCATOR_ROOT / "src"):
            raise SceneBindingError("fixed_scene_locator_editable_source_mismatch")

        sources = {
            "viewer": _sha256_file(viewer),
            "realsense_python": _sha256_file(FIXED_REALSENSE_PYTHON.resolve()),
            "locator_entrypoint": _sha256_file(FIXED_OBJECT_LOCATOR_EXECUTABLE),
            "locator_editable_link": _sha256_file(
                FIXED_OBJECT_LOCATOR_EDITABLE_LINK
            ),
            "locator_config": _sha256_file(locator_config_path),
            "locator_python_tree": _sha256_python_tree(
                FIXED_OBJECT_LOCATOR_ROOT / "src/object_locator"
            ),
            "locator_cli": _sha256_file(FIXED_OBJECT_LOCATOR_SOURCE),
            "locator_realsense": _sha256_file(
                FIXED_OBJECT_LOCATOR_REALSENSE_SOURCE
            ),
            "locator_extrinsics": _sha256_file(
                FIXED_OBJECT_LOCATOR_EXTRINSICS
            ),
        }
        return cls(
            repo,
            viewer,
            FIXED_REALSENSE_PYTHON,
            FIXED_OBJECT_LOCATOR_ROOT,
            FIXED_OBJECT_LOCATOR_EXECUTABLE,
            str(selected_mode),
            locator_config_path,
            detector_mode,
            FIXED_OBJECT_LOCATOR_SOURCE,
            FIXED_OBJECT_LOCATOR_REALSENSE_SOURCE,
            FIXED_OBJECT_LOCATOR_EXTRINSICS,
            artifact_root,
            configured_calibration_hash,
            sources,
        )

    @property
    def adapter_digest(self) -> str:
        backend_path = Path(__file__).resolve()
        return digest_json(
            {
                "backend_id": FIXED_SCENE_BACKEND_ID,
                "backend_sha256": _sha256_file(backend_path),
                "viewer_relative_path": FIXED_VIEWER_RELATIVE_PATH.as_posix(),
                "sources": dict(self.source_sha256),
                "camera_serial_hash": digest_json(FIXED_CAMERA_SERIALS),
                "locator_mode": self.locator_mode,
                "detector_mode": self.detector_mode,
                "calibration_hash": self.calibration_hash,
                "camera_sample_count": FIXED_CAMERA_SAMPLE_COUNT,
                "locator_sample_count": FIXED_LOCATOR_SAMPLE_COUNT,
                "process_timeout_s": FIXED_PROCESS_TIMEOUT_S,
            }
        )

    def public_dict(self) -> dict[str, Any]:
        return {
            "backend_id": FIXED_SCENE_BACKEND_ID,
            "adapter_digest": self.adapter_digest,
            "viewer_relative_path": FIXED_VIEWER_RELATIVE_PATH.as_posix(),
            "locator_mode": self.locator_mode,
            "locator_preset": self.locator_config.name,
            "locator_target": FIXED_TARGET_NAME,
            "detector": self.detector_mode,
            "supported_locator_modes": sorted(FIXED_LOCATOR_MODES),
            "camera_serial_hash": digest_json(FIXED_CAMERA_SERIALS),
            "calibration_hash": self.calibration_hash,
            "source_sha256": dict(self.source_sha256),
            "camera_sample_count": FIXED_CAMERA_SAMPLE_COUNT,
            "locator_sample_count": FIXED_LOCATOR_SAMPLE_COUNT,
            "real_hardware_validated": False,
        }


class FixedS10SceneBackend:
    """Closed S10H SceneBackend with no arbitrary command/config passthrough."""

    def __init__(
        self,
        binding: S10SceneBinding,
        *,
        process_runner: ProcessRunner | None = None,
    ) -> None:
        self.binding = binding
        self._process_runner = process_runner or _production_process_runner
        self._process_calls = 0
        self._camera_process_calls = 0
        self._locator_calls = 0

    @classmethod
    def from_config(
        cls,
        repo_root: Path,
        config: Mapping[str, Any],
        *,
        locator_mode: str | None = None,
    ) -> "FixedS10SceneBackend":
        return cls(
            S10SceneBinding.from_config(
                repo_root,
                config,
                locator_mode=locator_mode,
            )
        )

    @property
    def process_calls(self) -> int:
        return self._process_calls

    @property
    def camera_process_calls(self) -> int:
        return self._camera_process_calls

    @property
    def locator_calls(self) -> int:
        return self._locator_calls

    def _run(self, command: Sequence[str], cwd: Path, kind: str) -> ProcessResult:
        self._process_calls += 1
        if kind == "camera":
            self._camera_process_calls += 1
        elif kind == "locator":
            self._locator_calls += 1
        result = self._process_runner(command, cwd, FIXED_PROCESS_TIMEOUT_S)
        if result.returncode != 0:
            raise SceneProbeError(f"fixed_scene_{kind}_process_failed")
        return result

    def _validated_artifact_dir(self, artifact_dir: Path) -> Path:
        path = artifact_dir.resolve()
        if not path.is_relative_to(self.binding.artifact_root):
            raise SceneBindingError("fixed_scene_artifact_path_escape")
        path.mkdir(parents=True, exist_ok=True)
        return path

    def _capture_camera_samples(self, root: Path) -> list[dict[str, Any]]:
        samples: list[dict[str, Any]] = []
        for index in range(1, FIXED_CAMERA_SAMPLE_COUNT + 1):
            sample_dir = root / "realsense" / f"sample_{index:02d}"
            command = (
                str(self.binding.realsense_python),
                str(self.binding.viewer_path),
                "--mode",
                "live",
                "--hardware-allowed",
                "--camera",
                "all",
                "--output-dir",
                str(sample_dir),
                "--no-state",
                "--object-locator-src",
                str(self.binding.locator_root / "src"),
                "--capture-retries",
                "1",
                "--compact",
            )
            self._run(command, self.binding.repo_root, "camera")
            summary = _read_json_object(sample_dir / "summary.json", "viewer_summary")
            if summary.get("errors") != {}:
                raise SceneProbeError("fixed_scene_viewer_reported_errors")
            captures = _as_mapping(summary.get("captures"))
            if set(captures) != set(FIXED_CAMERA_SERIALS):
                raise SceneProbeError("fixed_scene_camera_set_mismatch")
            normalized: dict[str, Any] = {"sample": index, "cameras": {}}
            for name, expected_serial in FIXED_CAMERA_SERIALS.items():
                capture = _as_mapping(captures.get(name))
                if str(capture.get("serial_number")) != expected_serial:
                    raise SceneProbeError("fixed_scene_observed_camera_serial_mismatch")
                valid_depth_ratio = _finite_number(
                    capture.get("valid_depth_ratio"),
                    f"{name}_valid_depth_ratio",
                )
                if valid_depth_ratio <= 0.0 or valid_depth_ratio > 1.0:
                    raise SceneProbeError("fixed_scene_valid_depth_ratio_out_of_range")
                normalized["cameras"][name] = {
                    "timestamp_ms": _finite_number(
                        capture.get("timestamp_ms"),
                        f"{name}_timestamp_ms",
                    ),
                    "valid_depth_ratio": valid_depth_ratio,
                    "serial_hash": digest_json(expected_serial),
                    "artifact_refs": [
                        _artifact_path(
                            capture.get(field),
                            sample_dir.resolve(),
                            f"{name}_{field}",
                        )
                        for field in ("rgb_image", "depth_image", "panel_image")
                    ],
                }
            samples.append(normalized)
        return samples

    def _locate_samples(self, root: Path) -> list[dict[str, Any]]:
        samples: list[dict[str, Any]] = []
        for index in range(1, FIXED_LOCATOR_SAMPLE_COUNT + 1):
            sample_dir = root / "object_locator" / f"sample_{index:02d}"
            sample_dir.mkdir(parents=True, exist_ok=True)
            result_path = sample_dir / "result.json"
            panel_path = sample_dir / "panel.jpg"
            rgb_path = sample_dir / "rgb.jpg"
            depth_path = sample_dir / "depth.jpg"
            detector_trace_path = sample_dir / "detector_trace.json"
            command = (
                str(self.binding.locator_executable),
                "--config",
                str(self.binding.locator_config),
                "--result-json",
                str(result_path),
                "--history-dir",
                "",
                "--output",
                str(panel_path),
                "--output-rgb",
                str(rgb_path),
                "--output-depth",
                str(depth_path),
                "--save-vlm-response",
                str(detector_trace_path),
                "--capture-retries",
                "1",
                "--no-reset-realsense",
                "--json",
            )
            self._run(command, self.binding.locator_root, "locator")
            result = _read_json_object(result_path, "locator_result")
            runtime = _as_mapping(result.get("_runtime"))
            observed_realsense = _as_mapping(result.get("realsense"))
            position_base = _as_mapping(result.get("position_base"))
            detection = _as_mapping(result.get("detection"))
            debug_outputs = _as_mapping(result.get("debug_outputs"))
            if result.get("found") is not True:
                raise SceneProbeError("fixed_scene_target_not_found")
            if result.get("target") != FIXED_TARGET_NAME:
                raise SceneProbeError("fixed_scene_target_mismatch")
            if str(observed_realsense.get("serial_number")) != FIXED_CAMERA_SERIALS["head"]:
                raise SceneProbeError("fixed_scene_locator_camera_mismatch")
            if (
                runtime.get("config") != str(self.binding.locator_config)
                or runtime.get("detector_mode") != self.binding.detector_mode
                or runtime.get("calibration_active_camera") != "head"
                or Path(str(runtime.get("calibration_file"))).as_posix()
                != "calibration/extrinsics.yaml"
                or runtime.get("reset_on_start") is not False
            ):
                raise SceneProbeError("fixed_scene_locator_runtime_binding_mismatch")
            if (
                position_base.get("available") is not True
                or position_base.get("frame") != "base"
                or position_base.get("active_camera") != "head"
                or Path(str(position_base.get("calibration_file"))).resolve()
                != self.binding.locator_extrinsics.resolve()
            ):
                raise SceneProbeError("fixed_scene_base_frame_unavailable")
            confidence = _finite_number(
                detection.get("confidence"),
                "locator_confidence",
            )
            if confidence < 0.0 or confidence > 1.0:
                raise SceneProbeError("fixed_scene_locator_confidence_out_of_range")
            base_xyz = [
                _finite_number(position_base.get(name), f"locator_{name}")
                for name in ("x_m", "y_m", "z_m")
            ]
            artifact_refs = [
                _artifact_path(path, sample_dir.resolve(), f"locator_{name}")
                for name, path in debug_outputs.items()
                if isinstance(path, str) and name in {"panel", "rgb", "depth"}
            ]
            if len(artifact_refs) != 3:
                raise SceneProbeError("fixed_scene_locator_artifacts_incomplete")
            samples.append(
                {
                    "sample": index,
                    "timestamp_ms": _finite_number(
                        result.get("timestamp_ms"),
                        "locator_timestamp_ms",
                    ),
                    "target": FIXED_TARGET_NAME,
                    "locator_mode": self.binding.locator_mode,
                    "detector": self.binding.detector_mode,
                    "found": True,
                    "confidence": confidence,
                    "frame": "base",
                    "position_base_m": base_xyz,
                    "calibration_hash": self.binding.calibration_hash,
                    "artifact_refs": artifact_refs,
                }
            )
        return samples

    def capture_h1_observations(
        self,
        artifact_dir: Path,
        *,
        hardware_allowed: bool,
    ) -> dict[str, Any]:
        """Capture the fixed H1 sample set once; failures are never retried."""

        if not hardware_allowed:
            raise PermissionError("hardware_allowed_required")
        root = self._validated_artifact_dir(artifact_dir)
        camera_samples = self._capture_camera_samples(root)
        locator_samples = self._locate_samples(root)
        camera_timestamps = {
            name: [
                float(sample["cameras"][name]["timestamp_ms"])
                for sample in camera_samples
            ]
            for name in FIXED_CAMERA_SERIALS
        }
        locator_timestamps = [
            float(sample["timestamp_ms"]) for sample in locator_samples
        ]
        camera_monotonic = {
            name: _strictly_increasing(values)
            for name, values in camera_timestamps.items()
        }
        locator_monotonic = _strictly_increasing(locator_timestamps)
        stale_count = sum(not value for value in camera_monotonic.values())
        stale_count += int(not locator_monotonic)
        checks = {
            "scene_observation_status": "OK",
            "locator_mode": self.binding.locator_mode,
            "locator_detector": self.binding.detector_mode,
            "locator_preset": self.binding.locator_config.name,
            "camera_sample_count": {
                name: len(camera_samples) for name in FIXED_CAMERA_SERIALS
            },
            "camera_timestamps_monotonic": camera_monotonic,
            "camera_valid_depth_min_ratio": {
                name: min(
                    float(sample["cameras"][name]["valid_depth_ratio"])
                    for sample in camera_samples
                )
                for name in FIXED_CAMERA_SERIALS
            },
            "locator_sample_count": len(locator_samples),
            "locator_success_count": sum(
                sample["found"] is True for sample in locator_samples
            ),
            "locator_timestamps_monotonic": locator_monotonic,
            "locator_frame_missing_count": sum(
                sample["frame"] != "base" for sample in locator_samples
            ),
            "locator_confidence": [
                sample["confidence"] for sample in locator_samples
            ],
            "stale_output_count": stale_count,
            "calibration_hash": self.binding.calibration_hash,
            "camera_samples": camera_samples,
            "locator_samples": locator_samples,
        }
        checks["automatic_passed"] = (
            all(
                count == FIXED_CAMERA_SAMPLE_COUNT
                for count in checks["camera_sample_count"].values()
            )
            and all(camera_monotonic.values())
            and checks["locator_sample_count"] == FIXED_LOCATOR_SAMPLE_COUNT
            and checks["locator_success_count"] == FIXED_LOCATOR_SAMPLE_COUNT
            and locator_monotonic
            and checks["locator_frame_missing_count"] == 0
            and stale_count == 0
        )
        return checks
