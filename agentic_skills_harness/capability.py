from __future__ import annotations

from dataclasses import dataclass
import re
from typing import Any

from .contracts.enums import CapabilityKind, ErrorCode, ResourceMode, RiskClass
from .contracts.resources import ResourceRequirement
from .contracts.serialization import ContractValidationError, reject_unknown, require_bool, require_enum, require_list, require_number, require_string, to_plain


VISIBILITIES = {"public", "internal", "legacy"}
VERIFIER_TYPES = {"output_schema", "capability", "task_specific", "none"}
DISPATCH_SUPPORT = {"supported", "plan_only", "unsupported"}
CAPABILITY_ID_RE = re.compile(r"^[a-z][a-z0-9]*(?:\.[a-z][a-z0-9_-]*)+$")
SEMVER_RE = re.compile(r"^(0|[1-9]\d*)\.(0|[1-9]\d*)\.(0|[1-9]\d*)(?:-[0-9A-Za-z.-]+)?(?:\+[0-9A-Za-z.-]+)?$")


@dataclass(frozen=True)
class VerifierContract:
    type: str
    capability_id: str | None
    required_for_physical_success: bool
    physical_verification_limited: bool
    notes: str

    def __post_init__(self) -> None:
        object.__setattr__(self, "type", require_string(self.type, "verifier.type", non_empty=True))
        if self.type not in VERIFIER_TYPES:
            raise ContractValidationError(f"invalid verifier.type: {self.type!r}")
        if self.capability_id is not None:
            object.__setattr__(self, "capability_id", require_string(self.capability_id, "verifier.capability_id", non_empty=True))
        object.__setattr__(self, "required_for_physical_success", require_bool(self.required_for_physical_success, "verifier.required_for_physical_success"))
        object.__setattr__(self, "physical_verification_limited", require_bool(self.physical_verification_limited, "verifier.physical_verification_limited"))
        object.__setattr__(self, "notes", require_string(self.notes, "verifier.notes", non_empty=True))
        if self.type == "none" and self.required_for_physical_success:
            raise ContractValidationError("none verifier cannot be required for physical success")

    @classmethod
    def from_dict(cls, data: dict[str, Any]) -> "VerifierContract":
        names = ("type", "capability_id", "required_for_physical_success", "physical_verification_limited", "notes")
        reject_unknown(data, names, names)
        return cls(data["type"], data["capability_id"], data["required_for_physical_success"], data["physical_verification_limited"], data["notes"])

    def to_dict(self) -> dict[str, Any]:
        return to_plain(self)


@dataclass(frozen=True)
class CapabilityContract:
    name: str
    capability_id: str
    capability_version: str
    kind: CapabilityKind
    visibility: str
    path: str
    type: str
    input_schema_ref: str
    output_schema_ref: str
    requires_hardware: bool
    opens_camera: bool
    connects_robot_rpc: bool
    moves_robot: bool
    controls_gripper: bool
    physical_side_effects: tuple[str, ...]
    risk_class: RiskClass
    resources: tuple[ResourceRequirement, ...]
    preconditions: tuple[str, ...]
    effects: tuple[str, ...]
    invalidates: tuple[str, ...]
    timeout_s: float
    verifier: VerifierContract
    error_codes: tuple[ErrorCode, ...]
    default_safe_to_run: bool
    allowed_as_recovery: bool
    execute_flag: str | None
    notes: str
    description: str
    skill_name: str = ""
    skill_layer: str = ""
    adapter_id: str | None = None
    dispatch_support: str = "unsupported"
    artifact_policy: dict[str, Any] | None = None

    def __post_init__(self) -> None:
        for name in ("name", "capability_id", "capability_version", "path", "type", "input_schema_ref", "output_schema_ref", "notes", "description"):
            object.__setattr__(self, name, require_string(getattr(self, name), name, non_empty=True))
        if self.type not in {"python", "shell"}:
            raise ContractValidationError(f"unsupported entrypoint type: {self.type!r}")
        if not CAPABILITY_ID_RE.fullmatch(self.capability_id):
            raise ContractValidationError(f"invalid capability_id: {self.capability_id!r}")
        if not SEMVER_RE.fullmatch(self.capability_version):
            raise ContractValidationError(f"invalid capability_version: {self.capability_version!r}")
        if self.visibility not in VISIBILITIES:
            raise ContractValidationError(f"invalid visibility: {self.visibility!r}")
        object.__setattr__(self, "kind", require_enum(self.kind, CapabilityKind, "kind"))
        object.__setattr__(self, "risk_class", require_enum(self.risk_class, RiskClass, "risk_class"))
        for name in ("requires_hardware", "opens_camera", "connects_robot_rpc", "moves_robot", "controls_gripper", "default_safe_to_run", "allowed_as_recovery"):
            object.__setattr__(self, name, require_bool(getattr(self, name), name))
        for name in ("physical_side_effects", "preconditions", "effects", "invalidates"):
            values = getattr(self, name)
            if not isinstance(values, (list, tuple)) or any(not isinstance(value, str) or not value.strip() for value in values):
                raise ContractValidationError(f"{name} must be an array of non-empty strings")
            object.__setattr__(self, name, tuple(values))
        resources = tuple(item if isinstance(item, ResourceRequirement) else ResourceRequirement.from_dict(item) for item in self.resources)
        object.__setattr__(self, "resources", resources)
        if not self.resources:
            raise ContractValidationError("resources must not be empty")
        object.__setattr__(self, "timeout_s", require_number(self.timeout_s, "timeout_s", positive=True))
        object.__setattr__(self, "verifier", self.verifier if isinstance(self.verifier, VerifierContract) else VerifierContract.from_dict(self.verifier))
        codes = tuple(self.error_codes)
        if not codes:
            raise ContractValidationError("error_codes must not be empty")
        object.__setattr__(self, "error_codes", tuple(ErrorCode(code) for code in codes))
        if self.execute_flag is not None:
            object.__setattr__(self, "execute_flag", require_string(self.execute_flag, "execute_flag", non_empty=True))
        if self.adapter_id is not None:
            object.__setattr__(self, "adapter_id", require_string(self.adapter_id, "adapter_id", non_empty=True))
        object.__setattr__(self, "dispatch_support", require_string(self.dispatch_support, "dispatch_support", non_empty=True))
        if self.dispatch_support not in DISPATCH_SUPPORT:
            raise ContractValidationError(f"invalid dispatch_support: {self.dispatch_support!r}")
        if self.artifact_policy is not None:
            if not isinstance(self.artifact_policy, dict):
                raise ContractValidationError("artifact_policy must be an object")
            object.__setattr__(self, "artifact_policy", dict(self.artifact_policy))
        if self.kind == CapabilityKind.RECOVERY and self.risk_class not in (RiskClass.RECOVERY, RiskClass.HIGH_RISK):
            raise ContractValidationError("recovery capability must use RECOVERY or HIGH_RISK risk")
        is_physical = bool(self.physical_side_effects or self.moves_robot or self.controls_gripper)
        if is_physical and self.verifier.type == "none":
            raise ContractValidationError("physical capability requires a verifier contract")
        if self.moves_robot and (not self.requires_hardware or self.risk_class in (RiskClass.NONE, RiskClass.READ_ONLY_HARDWARE)):
            raise ContractValidationError("robot motion requires hardware and a non-read-only risk")
        if self.controls_gripper and (not self.requires_hardware or self.risk_class in (RiskClass.NONE, RiskClass.READ_ONLY_HARDWARE)):
            raise ContractValidationError("gripper control requires hardware and a non-read-only risk")
        if self.visibility == "public" and (not self.description or not self.input_schema_ref or not self.output_schema_ref):
            raise ContractValidationError("public capability metadata is incomplete")

    @classmethod
    def from_dict(cls, data: dict[str, Any], *, skill_name: str = "", skill_layer: str = "") -> "CapabilityContract":
        names = (
            "name", "capability_id", "capability_version", "kind", "visibility", "path", "type", "input_schema_ref", "output_schema_ref",
            "requires_hardware", "opens_camera", "connects_robot_rpc", "moves_robot", "controls_gripper", "physical_side_effects", "risk_class",
            "resources", "preconditions", "effects", "invalidates", "timeout_s", "verifier", "error_codes", "default_safe_to_run", "allowed_as_recovery",
            "execute_flag", "notes", "description", "adapter_id", "dispatch_support", "artifact_policy",
        )
        required_names = tuple(name for name in names if name not in {"adapter_id", "dispatch_support", "artifact_policy"})
        reject_unknown(data, names, required_names)
        return cls(
            name=data["name"], capability_id=data["capability_id"], capability_version=data["capability_version"], kind=data["kind"], visibility=data["visibility"],
            path=data["path"], type=data["type"], input_schema_ref=data["input_schema_ref"], output_schema_ref=data["output_schema_ref"], requires_hardware=data["requires_hardware"],
            opens_camera=data["opens_camera"], connects_robot_rpc=data["connects_robot_rpc"], moves_robot=data["moves_robot"], controls_gripper=data["controls_gripper"],
            physical_side_effects=tuple(data["physical_side_effects"]), risk_class=data["risk_class"], resources=tuple(ResourceRequirement.from_dict(item) for item in require_list(data["resources"], "resources")),
            preconditions=tuple(data["preconditions"]), effects=tuple(data["effects"]), invalidates=tuple(data["invalidates"]), timeout_s=data["timeout_s"], verifier=VerifierContract.from_dict(data["verifier"]),
            error_codes=tuple(ErrorCode(code) for code in require_list(data["error_codes"], "error_codes")), default_safe_to_run=data["default_safe_to_run"], allowed_as_recovery=data["allowed_as_recovery"],
            execute_flag=data["execute_flag"], notes=data["notes"], description=data["description"], skill_name=skill_name, skill_layer=skill_layer,
            adapter_id=data.get("adapter_id"), dispatch_support=data.get("dispatch_support", "unsupported"), artifact_policy=data.get("artifact_policy"),
        )

    def to_dict(self) -> dict[str, Any]:
        result = to_plain(self)
        result.pop("skill_name", None)
        result.pop("skill_layer", None)
        return result
