# Live capability implementation audit

`scripts/audit_live_capability_implementation.py` performs a static audit of
the 11 canonical operations and their existing capability IDs, manifests,
SKILL files, fixed bindings, schemas, side effects, verifier declarations,
and known gaps. It reads source and metadata only; it never imports or runs an
entrypoint.

The audit distinguishes implementation preparation from hardware validation.
Existing fixed bindings are at most `IMPLEMENTATION_READY` with
`HARDWARE_ACCEPTANCE_PENDING`; missing real-time guarded motion, safe-stop, or
fault-recovery APIs remain contract-only/disabled. The report always records
`real_hardware_validated_count: 0` during S10A.
