# Live capability implementation audit

`scripts/audit_live_capability_implementation.py` performs a static audit of
the 11 canonical operations and their existing capability IDs, manifests,
SKILL files, fixed bindings, schemas, side effects, verifier declarations,
and known gaps. It reads source and metadata only; it never imports or runs an
entrypoint.

The report also audits the dedicated
`dual_franka_robotiq_zerorpc.s10.v1` operator backend. That audit verifies the
fixed repository-relative client path, required backend/client methods,
absence of CLI selectors for a backend/client/executable, import ordering
after the operator gates, and the separation of recover, home, and stop
semantics. Constructing the binding only hashes and validates local source;
the ZeroRPC client is loaded lazily after all physical-run gates.

The same report separately audits `realsense_object_locator.s10.v1`. It binds
the existing RealSense viewer and object locator to three fixed camera serials,
the local calibration bundle, a fixed black-rack target, and two closed preset
IDs: local `grounded-sam` and `openrouter-vlm`. The latter uses the existing
object locator's local credential loading; no credential is included in the
binding or report. The audit verifies both preset files and their source
digests without opening a camera, loading a detector, or making a network
request. This backend is connected only to the dedicated S10 H1 runner; it
does not enable generic GraphExecutor live dispatch.

The audit distinguishes implementation preparation from hardware validation.
Existing fixed bindings are at most `IMPLEMENTATION_READY` with
`HARDWARE_ACCEPTANCE_PENDING`. The S10H-only backend has a real
`recover_robot` mapping, but the generic recovery capability remains
contract-only until its manifest/adapter binding and physical evidence exist.
Missing real-time guarded motion and safe-stop remain contract-only/disabled,
and grasp verification remains limited. The report always records
`real_hardware_validated_count: 0` during offline audit.
