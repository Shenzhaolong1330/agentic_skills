from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest
import yaml

from agentic_skills_harness.live.scene_backend import (
    FIXED_CAMERA_SERIALS,
    FIXED_LOCATOR_MODES,
    FIXED_OBJECT_LOCATOR_CONFIG,
    FIXED_OBJECT_LOCATOR_EXECUTABLE,
    FIXED_OBJECT_LOCATOR_VLM_CONFIG,
    FIXED_TARGET_NAME,
    FixedS10SceneBackend,
    ProcessResult,
    S10SceneBinding,
    SceneBindingError,
    SceneProbeError,
)
from agentic_skills_harness.live.hardware_acceptance import (
    run_fixed_hardware_probe,
)
from agentic_skills_harness.live.vendor_backend import (
    DualFrankaVendorBinding,
    FixedDualFrankaVendorBackend,
)
from scripts.audit_live_capability_implementation import audit_fixed_scene_backend


ROOT = Path(__file__).resolve().parents[1]
LOCAL_CONFIG = ROOT / "config/local/s10_hardware_acceptance.yaml"


def scene_config(artifact_root: Path) -> dict:
    config = yaml.safe_load(LOCAL_CONFIG.read_text(encoding="utf-8"))
    config["acceptance"]["artifact_root"] = str(artifact_root.resolve())
    return config


def _option(command: tuple[str, ...], name: str) -> str:
    return command[command.index(name) + 1]


class FakeSceneProcessRunner:
    def __init__(
        self,
        *,
        duplicate_timestamps: bool = False,
        failure_at: int | None = None,
    ) -> None:
        self.calls: list[tuple[tuple[str, ...], Path, float]] = []
        self.duplicate_timestamps = duplicate_timestamps
        self.failure_at = failure_at

    def __call__(
        self,
        command,
        cwd: Path,
        timeout_s: float,
    ) -> ProcessResult:
        args = tuple(command)
        self.calls.append((args, cwd, timeout_s))
        if self.failure_at == len(self.calls):
            return ProcessResult(1, "", "fixed fake failure")
        if args[0] == str(FIXED_OBJECT_LOCATOR_EXECUTABLE):
            self._write_locator_result(args)
        else:
            self._write_viewer_result(args)
        return ProcessResult(0, "{}", "")

    def _timestamp(self, index: int, offset: int = 0) -> float:
        return 1000.0 if self.duplicate_timestamps else 1000.0 + index * 10 + offset

    def _write_viewer_result(self, command: tuple[str, ...]) -> None:
        output_dir = Path(_option(command, "--output-dir"))
        output_dir.mkdir(parents=True, exist_ok=True)
        index = int(output_dir.name.split("_")[-1])
        captures = {}
        for offset, (name, serial) in enumerate(FIXED_CAMERA_SERIALS.items()):
            paths = {
                "rgb_image": output_dir / f"{name}_rgb.jpg",
                "depth_image": output_dir / f"{name}_depth.jpg",
                "panel_image": output_dir / f"{name}_panel.jpg",
            }
            for path in paths.values():
                path.write_bytes(b"fake")
            captures[name] = {
                "camera": name,
                "serial_number": serial,
                "timestamp_ms": self._timestamp(index, offset),
                "valid_depth_ratio": 0.9,
                **{key: str(value) for key, value in paths.items()},
            }
        (output_dir / "summary.json").write_text(
            json.dumps(
                {
                    "output_dir": str(output_dir),
                    "captures": captures,
                    "errors": {},
                }
            ),
            encoding="utf-8",
        )

    def _write_locator_result(self, command: tuple[str, ...]) -> None:
        locator_config = Path(_option(command, "--config"))
        detector_mode = (
            "vlm"
            if locator_config == FIXED_OBJECT_LOCATOR_VLM_CONFIG
            else "grounded_sam"
        )
        result_path = Path(_option(command, "--result-json"))
        sample_dir = result_path.parent
        sample_dir.mkdir(parents=True, exist_ok=True)
        index = int(sample_dir.name.split("_")[-1])
        outputs = {
            "panel": Path(_option(command, "--output")),
            "rgb": Path(_option(command, "--output-rgb")),
            "depth": Path(_option(command, "--output-depth")),
        }
        for path in outputs.values():
            path.write_bytes(b"fake")
        Path(_option(command, "--save-vlm-response")).write_text(
            "{}",
            encoding="utf-8",
        )
        result_path.write_text(
            json.dumps(
                {
                    "target": FIXED_TARGET_NAME,
                    "found": True,
                    "detection": {"confidence": 0.95},
                    "position_base": {
                        "available": True,
                        "frame": "base",
                        "active_camera": "head",
                        "calibration_file": str(
                            FIXED_OBJECT_LOCATOR_CONFIG.parent
                            / "calibration/extrinsics.yaml"
                        ),
                        "x_m": 0.45,
                        "y_m": 0.02,
                        "z_m": 0.12,
                    },
                    "debug_outputs": {
                        key: str(path) for key, path in outputs.items()
                    },
                    "timestamp_ms": self._timestamp(index),
                    "realsense": {
                        "serial_number": FIXED_CAMERA_SERIALS["head"]
                    },
                    "_runtime": {
                        "config": str(locator_config),
                        "detector_mode": detector_mode,
                        "calibration_active_camera": "head",
                        "calibration_file": "calibration/extrinsics.yaml",
                        "reset_on_start": False,
                    },
                }
            ),
            encoding="utf-8",
        )


class FakeReadOnlyVendorClient:
    def ping(self):
        return {"ok": True}

    def get_observation(self):
        return {
            side: {
                "robot_state": {
                    "joint_positions": [0.0] * 7,
                    "eef_pose": {
                        "position": [
                            0.35,
                            0.20 if side == "left_arm" else -0.20,
                            0.45,
                        ],
                        "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                    },
                },
                "gripper": {"position": 0.02},
            }
            for side in ("left_arm", "right_arm")
        }

    def close(self):
        return None


def make_backend(
    tmp_path: Path,
    runner: FakeSceneProcessRunner,
    *,
    locator_mode: str | None = None,
) -> FixedS10SceneBackend:
    binding = S10SceneBinding.from_config(
        ROOT,
        scene_config(tmp_path),
        locator_mode=locator_mode,
    )
    return FixedS10SceneBackend(binding, process_runner=runner)


def test_binding_reuses_fixed_host_assets_and_redacts_camera_serials(tmp_path):
    binding = S10SceneBinding.from_config(ROOT, scene_config(tmp_path))
    vlm_binding = S10SceneBinding.from_config(
        ROOT,
        scene_config(tmp_path),
        locator_mode="openrouter-vlm",
    )
    public = binding.public_dict()
    assert public["backend_id"] == "realsense_object_locator.s10.v1"
    assert public["locator_preset"] == "config_rack_center_grounded_sam.yaml"
    assert public["locator_target"] == FIXED_TARGET_NAME
    assert public["camera_sample_count"] == 5
    assert public["locator_sample_count"] == 5
    assert public["supported_locator_modes"] == [
        "grounded-sam",
        "openrouter-vlm",
    ]
    serialized = json.dumps(public)
    assert all(serial not in serialized for serial in FIXED_CAMERA_SERIALS.values())
    assert "OPENROUTER_API_KEY" not in serialized
    assert binding.adapter_digest != vlm_binding.adapter_digest


def test_binding_rejects_camera_or_calibration_substitution(tmp_path):
    config = scene_config(tmp_path)
    config["hardware"]["camera_serial"] = (
        "head=evil;left_wrist=347622074336;right_wrist=337322072568"
    )
    with pytest.raises(SceneBindingError, match="camera_serial_mismatch"):
        S10SceneBinding.from_config(ROOT, config)

    config = scene_config(tmp_path)
    config["hardware"]["calibration_hash"] = "sha256:" + "0" * 64
    config["acceptance"]["calibration_hash"] = config["hardware"]["calibration_hash"]
    with pytest.raises(SceneBindingError, match="source_digest_mismatch"):
        S10SceneBinding.from_config(ROOT, config)

    with pytest.raises(SceneBindingError, match="locator_mode_invalid"):
        S10SceneBinding.from_config(
            ROOT,
            scene_config(tmp_path),
            locator_mode="arbitrary-config",
        )


def test_hardware_gate_and_artifact_root_reject_before_process(tmp_path):
    runner = FakeSceneProcessRunner()
    backend = make_backend(tmp_path, runner)
    with pytest.raises(PermissionError, match="hardware_allowed_required"):
        backend.capture_h1_observations(
            tmp_path / "run",
            hardware_allowed=False,
        )
    with pytest.raises(SceneBindingError, match="artifact_path_escape"):
        backend.capture_h1_observations(
            tmp_path.parent / "escape",
            hardware_allowed=True,
        )
    assert runner.calls == []
    assert backend.process_calls == 0


@pytest.mark.parametrize(
    ("locator_mode", "expected_config", "expected_detector"),
    (
        ("grounded-sam", FIXED_OBJECT_LOCATOR_CONFIG, "grounded_sam"),
        ("openrouter-vlm", FIXED_OBJECT_LOCATOR_VLM_CONFIG, "vlm"),
    ),
)
def test_fixed_h1_scene_probe_captures_three_cameras_and_locator_five_times(
    tmp_path,
    locator_mode,
    expected_config,
    expected_detector,
):
    runner = FakeSceneProcessRunner()
    backend = make_backend(tmp_path, runner, locator_mode=locator_mode)
    checks = backend.capture_h1_observations(
        tmp_path / "run/observations/scene",
        hardware_allowed=True,
    )
    assert checks["automatic_passed"] is True
    assert checks["camera_sample_count"] == {
        "head": 5,
        "left_wrist": 5,
        "right_wrist": 5,
    }
    assert checks["locator_sample_count"] == 5
    assert checks["locator_success_count"] == 5
    assert checks["locator_frame_missing_count"] == 0
    assert checks["stale_output_count"] == 0
    assert checks["locator_mode"] == locator_mode
    assert checks["locator_detector"] == expected_detector
    assert backend.process_calls == 10
    assert backend.camera_process_calls == 5
    assert backend.locator_calls == 5

    viewer_commands = [call[0] for call in runner.calls[:5]]
    locator_commands = [call[0] for call in runner.calls[5:]]
    assert all("--mode" in command and "live" in command for command in viewer_commands)
    assert all("--hardware-allowed" in command for command in viewer_commands)
    assert all("--no-state" in command for command in viewer_commands)
    assert all("--execute" not in command for command in viewer_commands)
    assert all(
        _option(command, "--config") == str(expected_config)
        for command in locator_commands
    )
    assert all("--serial-number" not in command for command in locator_commands)
    assert all("--target" not in command for command in locator_commands)
    assert all("--no-reset-realsense" in command for command in locator_commands)


def test_stale_scene_timestamps_fail_closed(tmp_path):
    runner = FakeSceneProcessRunner(duplicate_timestamps=True)
    backend = make_backend(tmp_path, runner)
    checks = backend.capture_h1_observations(
        tmp_path / "run",
        hardware_allowed=True,
    )
    assert checks["automatic_passed"] is False
    assert checks["stale_output_count"] == 4


def test_process_failure_is_not_retried(tmp_path):
    runner = FakeSceneProcessRunner(failure_at=1)
    backend = make_backend(tmp_path, runner)
    with pytest.raises(SceneProbeError, match="camera_process_failed"):
        backend.capture_h1_observations(
            tmp_path / "run",
            hardware_allowed=True,
        )
    assert len(runner.calls) == 1
    assert backend.process_calls == 1


def test_h1_runner_requires_and_merges_vendor_and_scene_evidence(tmp_path):
    config = scene_config(tmp_path)
    scene_runner = FakeSceneProcessRunner()
    scene_backend = make_backend(tmp_path, scene_runner)
    vendor_backend = FixedDualFrankaVendorBackend(
        DualFrankaVendorBinding.from_config(ROOT, config),
        client_factory=lambda _binding: FakeReadOnlyVendorClient(),
    )
    result = run_fixed_hardware_probe(
        "read-only",
        vendor_backend,
        config,
        artifact_dir=tmp_path / "run",
        scene_backend=scene_backend,
    )
    assert result.automatic_passed is True
    assert result.checks["robot_sample_count"] == 10
    assert result.checks["gripper_sample_count"] == {"left": 10, "right": 10}
    assert result.checks["camera_sample_count"]["head"] == 5
    assert result.checks["locator_sample_count"] == 5
    assert result.checks["motion_command_calls"] == 0
    assert result.checks["gripper_command_calls"] == 0
    assert vendor_backend.backend_calls == 11
    assert scene_backend.process_calls == 10

    with pytest.raises(ValueError, match="fixed_scene_backend_required_for_h1"):
        run_fixed_hardware_probe(
            "read-only",
            vendor_backend,
            config,
            artifact_dir=tmp_path / "run2",
            scene_backend=None,
        )


def test_static_scene_audit_uses_no_camera_or_locator_runtime():
    report = audit_fixed_scene_backend(ROOT)
    assert report["status"] == "PASS_IMPLEMENTATION_READY_OFFLINE_AUDITED"
    assert report["binding_verified_statically"] is True
    assert report["hardware_calls_during_audit"] == 0
    assert report["real_hardware_validated"] is False
    assert report["operation_support"]["observe.scene"]["supported"] is True
    assert set(
        report["operation_support"]["observe.scene"]["available_modes"]
    ) == set(FIXED_LOCATOR_MODES)


def test_h0_reports_fixed_scene_binding_without_hardware_access():
    result = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/run_s10_hardware_acceptance.py"),
            "audit",
            "--repo-root",
            str(ROOT),
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert result.returncode == 0, result.stdout + result.stderr
    lines = [
        json.loads(line)
        for line in result.stdout.splitlines()
        if line.strip()
    ]
    assert lines[0]["scene_binding"]["status"] == "FIXED_SCENE_CONFIG_VALID"
    assert set(lines[0]["scene_binding"]["available_modes"]) == set(
        FIXED_LOCATOR_MODES
    )
    assert (
        lines[1]["scene_backend"]["status"]
        == "PASS_IMPLEMENTATION_READY_OFFLINE_AUDITED"
    )
    assert lines[1]["scene_backend"]["hardware_calls_during_audit"] == 0

    vlm = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/run_s10_hardware_acceptance.py"),
            "audit",
            "--repo-root",
            str(ROOT),
            "--scene-locator-mode",
            "openrouter-vlm",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert vlm.returncode == 0, vlm.stdout + vlm.stderr
    assert json.loads(vlm.stdout.splitlines()[0])["scene_binding"][
        "selected_mode"
    ] == "openrouter-vlm"

    rejected = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/run_s10_hardware_acceptance.py"),
            "audit",
            "--repo-root",
            str(ROOT),
            "--scene-locator-mode",
            "/tmp/evil.yaml",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert rejected.returncode == 2
    assert "invalid choice" in rejected.stderr
