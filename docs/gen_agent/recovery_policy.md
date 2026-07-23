# Recovery policy and safety matrix

Recovery policy is represented by `RecoveryPolicyRegistry` and the platform
guards in `agentic_skills_harness/recovery/`. Production registration accepts
only first-party strategies with declared capabilities and complete risk,
budget, and held-object metadata.

| Condition | Allowed next step | Explicitly disallowed |
| --- | --- | --- |
| Perception not found or stale | re-observe, then bounded alternate candidate | controller reset, blind action retry |
| Motion/IK infeasible | safe retreat, alternate arm/candidate, bounded replan | retrying an unsafe motion unchanged |
| Grasp not confirmed | re-observe or retreat; request human when ambiguous | claiming that a command proves holding |
| Object dropped | re-observe/replan only if the goal remains well-defined | reset or same-parameter retry |
| Robot fault with no held-object evidence | controller recovery/reset may be considered by policy | automatic physical execution in offline modes |
| Robot fault with confirmed/tentative/unknown held object | human or abort unless fresh evidence makes a safe continuation explicit | automatic reset-and-resume |
| Emergency stop or unsafe state | request human or abort | all automatic recovery |
| Budget exhausted, repeated no-progress, or invalid frame | abort or request human | retry/reset loops |

The matrix is a selection guard, not a claim that the action is physically
safe. Every action still requires the normal envelope, dispatcher, gate, and
verifier checks. Recovery capabilities must be explicitly marked
`allowed_as_recovery`; ordinary task actions are not recovery actions by
default.
