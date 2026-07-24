from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest

from agentic_skills_harness.live.vendor_backend import (
    DualFrankaVendorBinding,
    FixedDualFrankaVendorBackend,
    VendorBindingError,
)
from agentic_skills_harness.live.fingerprint import digest_json
from agentic_skills_harness.safety.estop_guard import EStopState
from agentic_skills_harness.safety.held_object_guard import HeldObjectState
from agentic_skills_harness.safety.motion_limits import MotionLimits
from agentic_skills_harness.safety.workspace import WorkspaceConfig
from scripts.audit_live_capability_implementation import audit_fixed_vendor_backend


ROOT = Path(__file__).resolve().parents[1]


def fixed_config() -> dict:
    config = {
        "version": 1,
        "hardware": {
            "robot_type": "franka_dual_arm",
            "controller_type": "dual_franka_robotiq_rpc_server@172.16.0.1:4242",
            "rpc_protocol_version": "zerorpc-nero_compatible@sha256:" + "1" * 64,
            "arm_ids": ["left_arm", "right_arm"],
            "gripper_type": "dual_robotiq_2f_85",
            "workspace_digest": "",
            "calibration_hash": "calibration",
            "reset_config_digest": "reset",
        },
        "workspace": {
            "robot_base_frame": "base",
            "safe_acceptance_poses": {
                "left_h3_a": {"frame": "base", "xyz_m": [0.35, 0.20, 0.45]},
                "right_h3_a": {"frame": "base", "xyz_m": [0.35, -0.20, 0.45]},
            },
        },
        "limits": {
            "max_translation_speed_m_s": 0.03,
            "max_rotation_speed_rad_s": 0.15,
            "max_translation_step_m": 0.010,
            "max_rotation_step_rad": 0.035,
            "max_timeout_s": 30.0,
            "max_relative_displacement_m": 0.010,
        },
        "acceptance": {"calibration_hash": "calibration"},
    }
    config["hardware"]["workspace_digest"] = "sha256:" + digest_json(config["workspace"])
    return config


def observation() -> dict:
    return {
        "left_arm": {
            "robot_state": {
                "joint_positions": [0.0] * 7,
                "eef_pose": {
                    "position": [0.35, 0.20, 0.45],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
            },
            "gripper": {"position": 0.02},
        },
        "right_arm": {
            "robot_state": {
                "joint_positions": [0.0] * 7,
                "eef_pose": {
                    "position": [0.35, -0.20, 0.45],
                    "orientation_xyzw": [0.0, 0.0, 0.0, 1.0],
                },
            },
            "gripper": {"position": 0.02},
        },
    }


class FakeVendorClient:
    def __init__(self) -> None:
        self.calls: list[tuple[str, tuple, dict]] = []
        self.closed = False

    def _record(self, name: str, *args, **kwargs):
        self.calls.append((name, args, kwargs))
        return {"ok": True, "method": name}

    def ping(self):
        return self._record("ping")

    def get_observation(self):
        self.calls.append(("get_observation", (), {}))
        return observation()

    def open_gripper(self, side):
        return self._record("open_gripper", side)

    def close_gripper(self, side):
        return self._record("close_gripper", side)

    def reactivate_gripper(self, side):
        return self._record("reactivate_gripper", side)

    def dual_robot_move_to_ee_pose(self, left, right, **kwargs):
        return self._record("dual_robot_move_to_ee_pose", left, right, **kwargs)

    def recover_robot(self, side):
        return self._record("recover_robot", side)

    def go_home(self, side, duration, rate):
        return self._record("go_home", side, duration, rate)

    def close(self):
        self.closed = True


def make_backend():
    binding = DualFrankaVendorBinding.from_config(ROOT, fixed_config())
    clients: list[FakeVendorClient] = []

    def factory(_binding):
        client = FakeVendorClient()
        clients.append(client)
        return client

    return FixedDualFrankaVendorBackend(binding, client_factory=factory), clients


def test_binding_is_fixed_to_bundled_client_and_private_rpc_port():
    binding = DualFrankaVendorBinding.from_config(ROOT, fixed_config())
    assert binding.client_path.is_file()
    assert binding.client_path.is_relative_to(ROOT)
    assert binding.host == "172.16.0.1"
    assert binding.port == 4242
    assert binding.public_dict()["endpoint"] == "private-ipv4:4242"
    assert "172.16.0.1" not in json.dumps(binding.public_dict())


@pytest.mark.parametrize(
    ("field", "value", "reason"),
    (
        ("robot_type", "other", "fixed_backend_robot_type_mismatch"),
        ("gripper_type", "other", "fixed_backend_gripper_type_mismatch"),
        ("arm_ids", ["right_arm", "left_arm"], "fixed_backend_arm_ids_mismatch"),
        (
            "controller_type",
            "dual_franka_robotiq_rpc_server@8.8.8.8:4242",
            "fixed_backend_private_ipv4_required",
        ),
        (
            "controller_type",
            "dual_franka_robotiq_rpc_server@172.16.0.1:9999",
            "fixed_backend_rpc_port_mismatch",
        ),
    ),
)
def test_binding_rejects_non_fixed_hardware(field, value, reason):
    config = fixed_config()
    config["hardware"][field] = value
    with pytest.raises(VendorBindingError, match=reason):
        DualFrankaVendorBinding.from_config(ROOT, config)


def test_construction_and_unsupported_capabilities_open_no_rpc():
    backend, clients = make_backend()
    assert clients == []
    assert backend.safe_stop()["status"] == "CAPABILITY_UNSUPPORTED"
    assert backend.verify_grasp()["verified"] is False
    assert clients == []
    assert backend.backend_calls == 0
    assert backend.rpc_connection_count == 0


def test_read_only_normalizes_real_vendor_observation_without_faking_fields():
    backend, clients = make_backend()
    assert backend.ping()["ok"] is True
    state = backend.read_robot_state()
    assert len(clients) == 1
    assert state["left"]["joint_positions"] == [0.0] * 7
    assert state["right"]["ee_pose"] == [0.35, -0.20, 0.45, 0.0, 0.0, 0.0]
    assert state["left"]["controller_state"] == "UNKNOWN"
    assert state["data_integrity"] == "PARTIAL"
    assert backend.backend_calls == 2
    assert backend.rpc_connection_count == 1


def test_gripper_has_closed_enum_mapping_and_close_never_asserts_holding():
    backend, clients = make_backend()
    backend.command_gripper(side="left", operation="open")
    backend.command_gripper(side="right", operation="close")
    assert [item[0] for item in clients[0].calls] == ["open_gripper", "close_gripper"]
    with pytest.raises(VendorBindingError, match="gripper_side_enum"):
        backend.command_gripper(side="both", operation="open")
    with pytest.raises(VendorBindingError, match="gripper_operation_enum"):
        backend.command_gripper(side="left", operation="drop")


def test_relative_motion_composes_absolute_target_and_uses_only_p2p_vendor_call():
    backend, clients = make_backend()
    backend.move_relative(
        side="left",
        delta_xyz_m=(0.0, 0.0, 0.010),
        delta_rotvec_rad=(0.0, 0.0, 0.0),
        reference_frame="base",
        workspace=WorkspaceConfig.from_dict(fixed_config()["workspace"]),
        limits=MotionLimits.from_dict(fixed_config()["limits"]),
    )
    call_names = [item[0] for item in clients[0].calls]
    assert call_names.count("get_observation") == 2
    assert call_names.count("dual_robot_move_to_ee_pose") == 1
    motion_call = next(item for item in clients[0].calls if item[0] == "dual_robot_move_to_ee_pose")
    assert motion_call[1][0][:3] == pytest.approx([0.35, 0.20, 0.46])
    assert motion_call[2]["delta"] is False
    assert motion_call[2]["smooth"] is True


def test_recovery_and_home_use_distinct_vendor_methods_and_guards():
    backend, clients = make_backend()
    with pytest.raises(PermissionError):
        backend.recover_fault(
            side="both",
            estop_state=EStopState.ESTOP,
            held_object_state=HeldObjectState.NONE_CONFIRMED,
            current_error_known=True,
            allowed_as_recovery=True,
        )
    assert clients == []
    backend.recover_fault(
        side="both",
        estop_state=EStopState.CLEAR,
        held_object_state=HeldObjectState.NONE_CONFIRMED,
        current_error_known=True,
        allowed_as_recovery=True,
    )
    backend.reset_home(
        estop_state=EStopState.CLEAR,
        held_object_state=HeldObjectState.NONE_CONFIRMED,
        allowed_as_recovery=True,
    )
    assert [item[0] for item in clients[0].calls] == ["recover_robot", "go_home"]


def test_backend_closes_only_an_instantiated_fixed_client():
    backend, clients = make_backend()
    backend.close()
    assert clients == []
    backend.ping()
    backend.close()
    assert clients[0].closed is True


def test_static_vendor_audit_passes_without_importing_or_calling_vendor_runtime():
    report = audit_fixed_vendor_backend(ROOT)
    assert report["status"] == "PASS_IMPLEMENTATION_READY_OFFLINE_AUDITED"
    assert report["binding_verified_statically"] is True
    assert report["hardware_calls_during_audit"] == 0
    assert report["real_hardware_validated"] is False
    assert report["operation_support"]["motion.safe_stop"]["supported"] is False
    assert report["operation_support"]["gripper.verify_grasp"]["supported"] is False
    assert report["operation_support"]["robot.recover_fault"]["supported"] is True


def test_h0_cli_has_no_backend_selector_and_remains_hardware_free():
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
    lines = [json.loads(line) for line in result.stdout.splitlines() if line.strip()]
    assert lines[0]["status"] == "CONFIG_VALID"
    assert lines[1]["vendor_backend"]["status"] == "PASS_IMPLEMENTATION_READY_OFFLINE_AUDITED"
    assert lines[1]["vendor_backend"]["hardware_calls_during_audit"] == 0

    rejected = subprocess.run(
        [
            sys.executable,
            str(ROOT / "scripts/run_s10_hardware_acceptance.py"),
            "audit",
            "--repo-root",
            str(ROOT),
            "--backend",
            "evil",
        ],
        cwd=ROOT,
        text=True,
        capture_output=True,
        check=False,
    )
    assert rejected.returncode != 0
    assert "unrecognized arguments: --backend evil" in rejected.stderr
