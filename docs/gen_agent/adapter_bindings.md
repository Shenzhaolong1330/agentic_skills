# Fixed Adapter bindings

S4.5 uses one first-party binding per plan-capable capability. A caller supplies only a capability ID and schema-validated business arguments. The registry chooses the adapter, executable, repository-relative entrypoint, fixed flags, preset, working directory, output policy, parser, and error map.

The request cannot supply `executable`, `argv`, `adapter`, `backend`, `env`, `cwd`, a config path, an output path, a reset script, a client path, `--execute`, or `--hardware-allowed`. Mode, hardware authorization, execution intent, artifact directory, robot server, repository root, and manifest path are context-derived fields. `hardware_allowed` remains the only explicit hardware authorization bit and defaults to false.

The public plan redacts host absolute paths. `InvocationPlan` is a dry-run description; it is not a live validation result. Hardware adapters are plan-only and live is disabled or not validated in this phase. No adapter test opens a camera, connects RPC, invokes reset, moves an arm, or actuates a gripper.

The object locator and task-specific tube-flow capabilities remain unsupported because their current entrypoint contracts do not provide a bounded, safe first-party argument map in this repository state.
