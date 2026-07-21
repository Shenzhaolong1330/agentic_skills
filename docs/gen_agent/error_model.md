# Structured error model (S2)

`ErrorInfo` is the stable external error shape. Standard `ErrorCode` values are
mapped by the read-only catalog to category, severity, retryability,
replanning safety, and human-intervention policy.

Unknown vendor or external codes are normalized to `UNKNOWN_ERROR`; their
original value is retained at `details.original_code`. The default is
conservative: not retryable, not safe to replan, and requiring a human. An
exception object, executable value, or non-JSON detail is rejected.

`VerificationResult` requires a method and evidence when verified. A result
with errors cannot be verified. `ActionResult` enforces plan-only, denied,
timeout, fatal/unsafe-error, and goal-verification invariants. This prevents a
successful child process from being reported as physical success.

The catalog is data, not a recovery policy executor. No automatic retry,
recovery, or verifier is run by these contracts.
