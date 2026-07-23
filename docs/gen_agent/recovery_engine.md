# S8 Recovery Engine

S8 adds a deterministic, policy-driven recovery runtime in
`agentic_skills_harness/recovery/`. Recovery is a bounded continuation of a
compiled `TaskGraph`; it is not an arbitrary command runner and it does not
make live hardware available.

The runtime pipeline is:

`RecoveryContext -> candidate registry -> platform/policy guards -> deterministic selector -> local retry, recovery subgraph, replan, human request, or abort`.

`RecoverySelector` evaluates only registered first-party strategies. It
rejects strategies when the execution envelope, risk class, mode, budget,
attempt limit, no-progress guard, held-object evidence, or platform guard
does not permit them. Ties at the highest priority fail closed. Selection and
rejection reasons are serializable and are emitted as structured events.

The first-party strategy set includes re-observation, bounded retry with the
same parameters, safe retreat, alternate candidate, alternate arm, remainder
recompile, controller recovery, reset-home, human request, and abort. Reset or
controller recovery is never selected merely because a fault occurred while
holding an object: the runtime requires explicit evidence and invalidates
affected facts before any resume decision.

Recovery templates compile to ordinary `TaskGraph` objects. They are executed
through the same bounded `GraphExecutor`, checkpoint writer, dispatcher, and
event chain as the original graph. Template compilation cannot dispatch a
capability or construct a shell command.

Offline modes (`mock`, `dry_run`, and `from_artifacts`) are the only supported
modes in this repository. `live` is rejected by the task CLI and no physical
execution is reported by the acceptance suite.

S8/S9 do not use a language-model planner. Codex Planner and long-term Memory
remain unimplemented, and real controller recovery/reset is deferred to the
future S10 audit boundary.
