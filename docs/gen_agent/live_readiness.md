# Live readiness

Readiness states are `DISABLED`, `CONTRACT_ONLY`, `IMPLEMENTATION_READY`,
`HARDWARE_ACCEPTANCE_PENDING`, `VALIDATED_READ_ONLY`, `VALIDATED_ACTION`,
`VALIDATED_RECOVERY`, and `REJECTED`. S10A may produce only implementation or
pending states. No automated test creates physically validated evidence.

Evidence is bound to capability version, adapter digest, input/output schema
digests, hardware fingerprint, workspace digest, and calibration hash. Version
or identity mismatch fails closed. Evidence is not a per-run token.
