# Nominal start orientation and saved compensation

An orientation inside the final warning/stop band is not necessarily inside the
nominal task tolerance. External tracking estimates its path and constructs its
servo with the same start reference selection: use the goal orientation as an
already-satisfied anchor only when the measured pose is inside nominal path
rotation tolerance and the complete saved-command rotation bias fits the unchanged
cap. Otherwise start the nominal path at the measured orientation, preserving the
existing saved command and all velocity, acceleration, wrench and bias limits.

Source: web-20261021-151146-1bd79ff2, action-002-approach. The requested rotation was
0.307463 rad, nominal tolerance0.2 rad, warning band0.4 rad, cap0.3 rad. Previously
the warning-band shortcut chose the goal as its start, then rejected its own
saved-to-reference bias before any command (0/0 dispatch/ACK). This is a reference
selection conflict, not proof of a hardware contact or a reason to increase caps.

Tests: tests/test_reference_anchor.py reproduces the original preflight conflict,
checks preserved compensation and true over-cap rejection, and exercises a bounded
synthetic path. Offline plant success is not physical task success. Unknown ACKs,
changed controller sessions and genuinely over-cap saved state remain blockers.
