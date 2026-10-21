# External P2P speed profiles

These are explicit experimental profiles of `external_tracking`. The default
remains `conservative`; neither the raw policy `step(action)` route nor the
robot-side impedance controller, gains or filters is modified.

The default total angular compensation cap remains 0.20 rad. A supervisor may
explicitly set `FRANKA_P2P_ROTATION_BIAS_LIMIT_RAD` to a finite positive value
below pi/2; the full saved starting offset still counts toward it. The console's
2026-10-17 validation configuration uses 0.30 rad after measuring a 0.2255 rad
offset that prevented vertical fine correction. This does not change arrival
tolerance, speed/acceleration, wrench checks, or any raw policy/contact route.
The selected value appears in `servo_config.max_rotation_bias_rad`; unconfigured
callers retain the existing 0.20 rad behavior.

Select with `FRANKA_P2P_TRACKING_PROFILE`. This selects **upper bounds**, not a
promised physical speed or completion time. Caller speed/step limits still win.

| Profile | Translation command ceiling | Translation acceleration | Orientation along the path |
|---|---:|---:|---|
| conservative | 20 mm/s | 40 mm/s² | Existing tracking governor |
| responsive | 20 mm/s | 40 mm/s² | Existing governor, smooth angular dead zone |
| fast_free_space | 100 mm/s | 200 mm/s² | Governed; 0.07–0.09 rad slowdown band |
| fast_endpoint | 100 mm/s | 200 mm/s² | Unconstrained task attitude until endpoint |
| fast_tracking | 60 mm/s | 100 mm/s² | Measured corridor retained; nominal-tolerance angular governor |

`fast_tracking` is the console candidate for the operator-authorized repeated
free-space trials. Earlier console trials failed an experimental joint-speed
guard. At the operator's request, that 0.15 rad/s stop threshold and its associated
joint-speed slowdown/pause/prediction logic have now been removed. It
uses the faster parallel XYZ/orientation feedback and precise stationary capture
from the endpoint experiments, while retaining the original measured orientation
corridor. It does **not** enable endpoint-only attitude. Its angular governor
slows from nominal rotation tolerance to the explicitly configured acceptance
boundary (at most twice nominal). At the console's 0.20 rad nominal tolerance
and two-times warning policy these thresholds are 0.20 and 0.40 rad. Without
a warning band it slows from 60% to 100% of nominal. Measured tracking error is
independent of command compensation: the latter retains its separately configured
cap. XYZ tracking slows between 6 and 15 mm of reference lag. Tracking-error
slowdowns immediately limit reference progression; only recovery is ramped.
Measured joint speed remains telemetry and is used to verify stationary start
and arrival, not as an additional in-motion stop or slowdown threshold.
Native robot faults, wrench checks and caller command limits still apply.
For `fast_tracking`, translation steps are at most 3 mm;
angular command speed remains 0.12 rad/s with acceleration 0.20 rad/s².

The planner now accounts for command step / control period when computing
effective speed and quintic duration. A 1 mm step at 20 Hz means at most 20 mm/s,
even if the caller requests 100 mm/s. Configuration and results expose the
nominal duration plus minimum hold, actual call time and whether 5 seconds was
met. The performance target never replaces physical arrival or extends a deadline.

Fast tracking captures at the caller's nominal position and rotation tolerances,
with fresh joint speed below 0.01 rad/s before latching. The console's 5 mm /
0.20 rad policy therefore has no hidden 3 mm / 0.08 rad capture requirement.
The wider warning band does not become the capture goal. Full measured stationary
hold and independent final verification are unchanged.

Completed calls retain their actual final reference, not a newly measured
orientation that would discard the meaning of accumulated compensation. A
receipt is kept on the client and, for console subprocesses, in the run-owned
`p2p_reference.json`. Reuse requires the same server session, exact command count,
matching saved targets on **both** arms, fresh measured attitude within nominal
tolerance of that reference, and the complete saved-to-reference bias inside the
original cap. Missing or invalid receipts never bypass normal initialization.
One acknowledged gripper-only RPC may advance the receipt's count after a fresh
read confirms unchanged saved arm targets. Unknown replies, foreign commands,
session changes and changed arm targets cannot advance it. Only fully completed
P2P calls write new references; raw policy `step` remains unchanged. There is no
implicit import of a receipt from a different console task.

At a saturated inherited command, moving the next trajectory's reference can
put both the nominal proposal and zero-velocity braking outside the new bias
sphere. The proposal first tests an inward direction with the same speed,
acceleration and braking-reserve constraints. If the reference moves too far for
that tick, it stays at its last acknowledged value while the command decelerates
within the already reserved space. Only one feasible proposal is dispatched;
the reference clock advances only after its ACK. The cap is never enlarged.
The recorded 2026-10-16 cross-stage start has regression coverage including
fixed-bias and lagged synthetic continuation and transport quaternion conversion.

`fast_endpoint` implements the operator's explicit requirement: intermediate
attitude is free; final attitude is constrained. Angular feedback runs alongside
translation, but angular error does not stop reference translation. The moving
arm has no intermediate measured-attitude corridor. The other, held arm retains
its original pose checks. Position corridors, native fault/collision/force checks,
command ownership and native robot fault checks remain active.

The endpoint profile allows angular equilibrium-command speed up to 0.12 rad/s,
angular acceleration 0.4 rad/s² and steps of 0.006 rad. Linear steps are at most
5 mm. These are reference-command limits, not measured tool or joint velocities.
Its Cartesian tracking governor slows between 6 and 15 mm of reference lag;
this **does not change** final acceptance. Strict hardware probes use 3 mm / 0.10 rad;
any explicit console arrival policy remains a separate caller choice. All profiles
now omit the former experimental joint-speed thresholds; command acceleration
and final stationary hold remain separate requirements.

Endpoint capture is tighter than acceptance: at most 1 mm / 0.08 rad (and tighter
when the caller asks for tighter tolerances). Feedback continues inside the
acceptance band until capture, reducing stop/drift/reacquire cycles observed on
hardware. Capture also waits for measured joint speeds below 0.01 rad/s before latching; until then small drift stays inside the running feedback loop. Full stationary hold and fresh final verification remain mandatory.
A 5-second performance target is not an instruction to report success at timeout.

When initial measured attitude already satisfies the requested endpoint,
fast profiles may use that endpoint attitude as the reference from the start.
The saved command is preserved and its entire angular compensation must fit the
same 0.20 rad cap around that reference. Excess compensation is rejected, not
cleared. Otherwise the ordinary angular trajectory is retained.

## Calling and rollback

Use `controller="external_tracking"` and the profile environment variable in a
free-space child process. For the final experimental endpoint configuration,
explicit caller ceilings are `max_translation_speed=.10`,
`max_translation_step=.005`, `max_rotation_speed=.12`,
`max_rotation_step=.006`, `rate_hz=20`. Smaller caller values are respected.
The standalone CLI defaults to a smaller translation step, so setting only a
larger speed does not enable the larger step.

The supervised probe at `robot_agent_lab/precision_research/run_external_tracking_probe.py`
provides resource ownership, fixed workspace origin, gripper baseline checks,
independent controller-receipt verification, and terminal reconciliation. It
retains a separate 0.20 rad absolute angular command envelope. Use only in the
operator-authorized free-space region. Do not apply this profile to contact,
insertion, unknown carried objects, or the end-to-end policy route.

To roll back, select `conservative` or unset the profile variable. Existing
`continuous_servo` and `segmented` routes remain available. The console runtime
configuration currently selects `fast_tracking` for the authorized repeated tests; it
raises speed/step to 60 mm/s and 3 mm only for the
empty-hand pregrasp XY transit and registered empty-hand atomic move. Grasp
descent, carried lift and other callers retain their lower speed and step
settings; contact insertion and reset retain their separate controller route.

The shared outer-servo geometry also handles quaternion roundoff at a saturated
cap: validation allows four 1e-12 numerical units, and an already acknowledged
command can be held exactly when a numerically saturated step can brake within
one cycle. No new outward increment or physical cap enlargement is introduced.

Hardware results and known limitations are recorded in the workspace report
`robot_agent_lab/precision_research/reports/20261016-p2p-five-seconds.md`.
Later console trials and the unqualified fast-tracking candidate are recorded in
`robot_agent_lab/reports/2026-10-16-console-p2p-validation.md`.
