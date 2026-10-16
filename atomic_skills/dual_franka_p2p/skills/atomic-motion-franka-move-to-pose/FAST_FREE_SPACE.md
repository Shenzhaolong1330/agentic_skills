# External P2P speed profiles

These are explicit experimental profiles of `external_tracking`. The default
remains `conservative`; neither the raw policy `step(action)` route nor the
robot-side impedance controller, gains or filters is modified.

Select with `FRANKA_P2P_TRACKING_PROFILE`. This selects **upper bounds**, not a
promised physical speed or completion time. Caller speed/step limits still win.

| Profile | Translation command ceiling | Translation acceleration | Orientation along the path |
|---|---:|---:|---|
| conservative | 20 mm/s | 40 mm/s² | Existing tracking governor |
| responsive | 20 mm/s | 40 mm/s² | Existing governor, smooth angular dead zone |
| fast_free_space | 100 mm/s | 200 mm/s² | Governed; 0.07–0.09 rad slowdown band |
| fast_endpoint | 100 mm/s | 200 mm/s² | Unconstrained task attitude until endpoint |

`fast_endpoint` implements the operator's explicit requirement: intermediate
attitude is free; final attitude is constrained. Angular feedback runs alongside
translation, but angular error does not stop reference translation. The moving
arm has no intermediate measured-attitude corridor. The other, held arm retains
its original pose checks. Position corridors, native fault/collision/force checks,
command ownership and the 0.15 rad/s measured joint-speed check remain active.

The endpoint profile allows angular equilibrium-command speed up to 0.12 rad/s,
angular acceleration 0.4 rad/s² and steps of 0.006 rad. Linear steps are at most
5 mm. These are reference-command limits, not measured tool or joint velocities.
Its Cartesian tracking governor slows between 6 and 15 mm of reference lag;
this **does not change** final acceptance. Strict hardware probes use 3 mm / 0.10 rad; any explicit console arrival policy remains a separate caller choice. Both fast profiles
slow reference progression above 0.10 rad/s measured joint speed and pause at
0.135 rad/s. Endpoint mode also scales feedback correction with this governor;
bounded command deceleration still applies. This is anticipatory feedback, not a
proof that the hard joint-speed threshold can never be exceeded.

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
`continuous_servo` and `segmented` routes remain available. No new fast profile
is selected automatically by the console.

The shared outer-servo geometry also handles quaternion roundoff at a saturated
cap: validation allows four 1e-12 numerical units, and an already acknowledged
command can be held exactly when a numerically saturated step can brake within
one cycle. No new outward increment or physical cap enlargement is introduced.

Hardware results and known limitations are recorded in the workspace report
`robot_agent_lab/precision_research/reports/20261016-p2p-five-seconds.md`.
