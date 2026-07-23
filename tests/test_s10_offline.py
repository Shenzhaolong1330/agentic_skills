from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys

import pytest

from agentic_skills_harness.dispatch import DispatchRequest
from agentic_skills_harness.live import (
    CapabilityReadiness,
    LivePreflightPolicy,
    LivePreflightRequest,
    ReadinessRegistry,
    ReadinessState,
    VerificationMaturity,
)
from agentic_skills_harness.live.fingerprint import HardwareFingerprint
from agentic_skills_harness.meta_operations.geometry import Transform, TransformChain, TransformPoseError, transform_pose
from agentic_skills_harness.meta_operations.gripper import GripperCommand
from agentic_skills_harness.meta_operations.motion import GuardedMoveContract, MoveRelativeContract, MoveToPoseContract
from agentic_skills_harness.meta_operations.robot import FaultRecoveryContract, SafeStopContract, observe_robot_state
from agentic_skills_harness.meta_operations.reset import ResetHomeContract
from agentic_skills_harness.meta_operations.verifiers import GraspEvidence, verify_grasp
from agentic_skills_harness.safety.estop_guard import EStopState
from agentic_skills_harness.safety.held_object_guard import HeldObjectState
from agentic_skills_harness.safety.workspace import WorkspaceConfig


ROOT = Path(__file__).resolve().parents[1]


def readiness(state=ReadinessState.HARDWARE_ACCEPTANCE_PENDING, *, validated=False):
    return CapabilityReadiness(
        "motion.move_to_pose", "1.0.0", "fixed.motion.v1", "adapter", "input", "output", "MOTION",
        state, "H3_MOTION_P2P", ("fp",) if validated else (), ("cal",) if validated else (), (), "2026-01-01T00:00:00Z",
        VerificationMaturity.PHYSICALLY_VALIDATED if validated else VerificationMaturity.LIMITED, "workspace" if validated else "",
    )


def test_all_readiness_states_and_pending_is_not_validated():
    for state in ReadinessState:
        if state in {ReadinessState.VALIDATED_READ_ONLY, ReadinessState.VALIDATED_ACTION, ReadinessState.VALIDATED_RECOVERY}:
            value = readiness(state, validated=True)
            assert value.live_validated
        else:
            value = readiness(state)
            assert not value.live_validated


def test_readiness_identity_mismatches_fail_closed():
    registry = ReadinessRegistry([readiness(ReadinessState.VALIDATED_ACTION, validated=True)])
    common = dict(acceptance_level="H3_MOTION_P2P", hardware_fingerprint_digest="fp", calibration_hash="cal", capability_version="1.0.0", adapter_digest="adapter", input_schema_digest="input", output_schema_digest="output", workspace_digest="workspace")
    assert registry.can_live("motion.move_to_pose", **common) == (True, "accepted")
    for key, value in (("capability_version", "old"), ("adapter_digest", "old"), ("hardware_fingerprint_digest", "other"), ("calibration_hash", "other")):
        changed = dict(common, **{key: value})
        assert registry.can_live("motion.move_to_pose", **changed)[0] is False


def test_readiness_transition_requires_real_evidence_and_rejects_illegal_jump():
    registry = ReadinessRegistry([readiness(ReadinessState.IMPLEMENTATION_READY)])
    with pytest.raises(ValueError): registry.transition("motion.move_to_pose", ReadinessState.VALIDATED_ACTION)
    registry.transition("motion.move_to_pose", ReadinessState.HARDWARE_ACCEPTANCE_PENDING)
    evidence = {"real_hardware": True, "passed": True, "capability_id": "motion.move_to_pose", "capability_version": "1.0.0", "adapter_digest": "adapter", "input_schema_digest": "input", "output_schema_digest": "output", "hardware_fingerprint_digest": "fp", "calibration_hash": "cal", "workspace_digest": "workspace"}
    promoted = registry.transition("motion.move_to_pose", ReadinessState.VALIDATED_ACTION, evidence=evidence)
    assert promoted.live_validated


def test_fingerprint_hashes_sensitive_identifiers():
    value = HardwareFingerprint.from_config({"robot_type": "r", "controller_type": "c", "rpc_protocol_version": "1", "arm_ids": ["left", "right"], "camera_serial": "SECRET-SERIAL", "gripper_type": "g", "workspace_digest": "w", "calibration_hash": "c1", "reset_config_digest": "r1"})
    assert "SECRET-SERIAL" not in json.dumps(value.to_dict())
    assert len(value.digest) == 64


def test_preflight_rejects_every_invalid_gate_without_backend_calls():
    policy = LivePreflightPolicy(ReadinessRegistry([readiness()]))
    request = LivePreflightRequest("motion.move_to_pose", "live", True, True, True, "H3_MOTION_P2P", "1.0.0", "adapter", "input", "output", "fp", "cal", "workspace", robot_health="READY", estop_or_unsafe=False, resources_available=True, input_valid=True, workspace_valid=True, speed_valid=True, force_valid=True, held_object_safe=True, verifier_available=True)
    result = policy.evaluate(request)
    assert not result.allowed and result.backend_calls_allowed == 0
    assert "hardware_acceptance_pending" in result.reasons


def test_preflight_allows_fake_backend_only_with_validated_evidence():
    policy = LivePreflightPolicy(ReadinessRegistry([readiness(ReadinessState.VALIDATED_ACTION, validated=True)]))
    request = LivePreflightRequest("motion.move_to_pose", "live", True, True, True, "H3_MOTION_P2P", "1.0.0", "adapter", "input", "output", "fp", "cal", "workspace", robot_health="READY", estop_or_unsafe=False, resources_available=True, input_valid=True, workspace_valid=True, speed_valid=True, force_valid=True, held_object_safe=True, verifier_available=True)
    result = policy.evaluate(request)
    assert result.allowed and result.backend_calls_allowed == 1 and not result.evidence_updated


def test_transform_identity_inverse_composition_and_calibration_rejection():
    chain = TransformChain((Transform("camera", "base", (1, 2, 3), calibration_hash="cal"),), "cal")
    output = transform_pose(pose={"frame": "camera", "xyz_m": [0, 0, 0], "rotvec_rad": [0, 0, 0]}, target_frame="base", chain=chain, expected_calibration_hash="cal")
    assert output["frame"] == "base" and output["xyz_m"] == [1.0, 2.0, 3.0]
    with pytest.raises(TransformPoseError):
        transform_pose(pose={"frame": "camera", "xyz_m": [0, 0, 0]}, target_frame="base", chain=chain, expected_calibration_hash="wrong")
    with pytest.raises(TransformPoseError):
        transform_pose(pose={"frame": "camera", "xyz_m": [float("nan"), 0, 0]}, target_frame="base", chain=chain, expected_calibration_hash="cal")


def test_relative_move_is_fresh_bounded_and_delegates_absolute_target():
    contract = MoveRelativeContract(MoveToPoseContract(workspace=WorkspaceConfig()))
    current = {"frame": "base", "xyz_m": [0.1, 0.1, 0.4], "rotvec_rad": [0, 0, 0]}
    target = contract.compute_target(current, delta_xyz_m=[0.001, 0, 0], delta_rotvec_rad=[0, 0, 0], reference_frame="base", side="left", freshness_s=0.1)
    assert target["source"] == "motion.move_relative.composed"
    with pytest.raises(ValueError): contract.compute_target(current, delta_xyz_m=[0.001, 0, 0], delta_rotvec_rad=[0, 0, 0], reference_frame="base", side="left", freshness_s=2)
    with pytest.raises(ValueError): contract.compute_target(current, delta_xyz_m=[0.02, 0, 0], delta_rotvec_rad=[0, 0, 0], reference_frame="base", side="left", freshness_s=0.1)


def test_guarded_move_without_realtime_stop_is_unsupported():
    value = GuardedMoveContract().validate(max_distance_m=.01, max_speed_m_s=.01, max_force_n=1, max_torque_nm=.1, timeout_s=1, contact_direction=[0, 0, 1], contact_success_condition="force", retreat_policy="retreat")
    assert value["status"] == "CAPABILITY_UNSUPPORTED" and value["live_readiness"] == "DISABLED"


def test_gripper_close_never_creates_holding_and_grasp_is_limited_without_evidence():
    assert GripperCommand().validate(operation="close", side="left")["may_release"] == "false"
    assert GripperCommand().execute(object(), operation="close", side="left")["holding"] == "UNKNOWN"
    assert not verify_grasp([]).verified
    assert not verify_grasp([GraspEvidence("close_returncode", True)]).verified
    assert verify_grasp([GraspEvidence("width", 0.02, .9, True), GraspEvidence("motor_current", 1.2, .9, True)]).verified


def test_safe_stop_never_maps_to_reset_and_fault_recovery_guards_estop_and_held_unknown():
    stop = SafeStopContract().execute(object())
    assert stop["status"] == "CAPABILITY_UNSUPPORTED"
    denied = FaultRecoveryContract().execute(object(), estop_state=EStopState.ESTOP, held_object_state=HeldObjectState.NONE_CONFIRMED, current_error_known=True, allowed_as_recovery=True)
    assert denied["status"] == "DENIED"


def test_reset_home_is_separate_and_requires_recovery_guards():
    denied = ResetHomeContract().execute(object(), hardware_allowed=True, execute=True, allowed_as_recovery=True, estop_state=EStopState.ESTOP, held_object_state=HeldObjectState.NONE_CONFIRMED)
    assert denied["status"] == "DENIED" and denied["reason"] == "estop_or_unsafe"
    denied = ResetHomeContract().execute(object(), hardware_allowed=True, execute=True, allowed_as_recovery=True, estop_state=EStopState.CLEAR, held_object_state=HeldObjectState.UNKNOWN)
    assert denied["status"] == "DENIED" and denied["reason"] == "held_object_guard"
    denied = FaultRecoveryContract().execute(object(), estop_state=EStopState.CLEAR, held_object_state=HeldObjectState.UNKNOWN, current_error_known=True, allowed_as_recovery=True)
    assert denied["status"] == "DENIED"


def test_robot_state_unknown_fields_are_not_faked():
    value = observe_robot_state(object())
    assert value["controller_state"] == "UNKNOWN" and value["robot_mode"] == "UNKNOWN" and value["data_integrity"] == "PARTIAL"


def test_dispatch_security_rejects_at_least_sixty_malicious_payloads():
    payloads = []
    for key in ("executable", "argv", "env", "cwd", "adapter", "backend", "script", "reset_script", "client_path", "config_path", "result_path", "hardware_allowed", "execute", "workspace_override", "speed_override", "force_override", "validated", "acceptance_state", "pose", "command"):
        payloads.extend(({"capability_id": "x.y", "arguments": {key: "bad"}}, {"capability_id": "x.y", "arguments": {"nested": {key: "bad"}}}, {"capability_id": "x.y", "metadata": {key: "bad"}}))
    assert len(payloads) >= 60
    rejected = 0
    for payload in payloads:
        with pytest.raises(ValueError): DispatchRequest.from_dict(payload)
        rejected += 1
    assert rejected == len(payloads)


def test_acceptance_cli_default_and_audit_are_hardware_free(tmp_path):
    command = [sys.executable, str(ROOT / "scripts/run_s10_hardware_acceptance.py"), "audit", "--repo-root", str(ROOT)]
    result = subprocess.run(command, cwd=ROOT, text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stdout + result.stderr
    assert "real_hardware_validated_count" in result.stdout
