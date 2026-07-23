from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Iterable


@dataclass(frozen=True)
class GraspEvidence:
    evidence_type: str
    value: Any
    confidence: float | None = None
    independent: bool = False


@dataclass(frozen=True)
class GraspVerification:
    verified: bool
    evidence_types: tuple[str, ...]
    confidence: float | None
    limited: bool
    reason: str

    def to_dict(self) -> dict[str, Any]:
        return {"verified": self.verified, "evidence_types": list(self.evidence_types), "confidence": self.confidence, "limited": self.limited, "reason": self.reason}


def verify_grasp(evidence: Iterable[GraspEvidence], *, object_present_reliable: bool = False) -> GraspVerification:
    values = tuple(evidence)
    names = tuple(item.evidence_type for item in values)
    if not values: return GraspVerification(False, (), None, True, "no_independent_evidence")
    independent = tuple(item for item in values if item.independent and item.value is not False)
    if not independent: return GraspVerification(False, names, None, True, "close_returncode_is_not_grasp_evidence")
    if len(independent) == 1 and independent[0].evidence_type == "object_present" and not object_present_reliable:
        return GraspVerification(False, names, independent[0].confidence, True, "object_present_reliability_limited")
    confidence_values = [item.confidence for item in independent if item.confidence is not None]
    confidence = min(confidence_values) if confidence_values else None
    return GraspVerification(True, names, confidence, False, "independent_evidence_agrees")
