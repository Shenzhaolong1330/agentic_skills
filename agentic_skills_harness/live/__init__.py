"""Offline-first capability readiness and hardware acceptance contracts.

The live package contains evidence and policy data only.  It never opens a
device, starts a process, or grants hardware authorization.
"""

from .acceptance import AcceptanceLevel, AcceptanceObservation, AcceptancePlan, AcceptanceResult, AcceptanceStep
from .evidence import AcceptanceEvidence, EvidenceMismatch, HardwareFingerprint
from .policy import LivePreflightPolicy, LivePreflightRequest, LivePreflightResult
from .readiness import (
    CapabilityReadiness,
    ReadinessError,
    ReadinessRegistry,
    ReadinessState,
    VerificationMaturity,
)

__all__ = [
    "AcceptanceEvidence", "AcceptanceLevel", "AcceptancePlan", "AcceptanceResult",
    "AcceptanceObservation", "AcceptanceStep", "CapabilityReadiness", "EvidenceMismatch", "HardwareFingerprint",
    "LivePreflightPolicy", "LivePreflightRequest", "LivePreflightResult", "ReadinessRegistry",
    "ReadinessError", "ReadinessState", "VerificationMaturity",
]
