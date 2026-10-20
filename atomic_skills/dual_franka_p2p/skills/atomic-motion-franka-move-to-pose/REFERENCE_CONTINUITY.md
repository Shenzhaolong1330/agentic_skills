# Success reference across precision stages

Source: `web-20261020-224321-11c413b0`, `action-009-align-hole`.
The first alignment confirmed 201 commands through count 14881, with 0.597 mm
translation error and 0.06051 rad rotation error, inside its 0.2 rad criterion.
The following vertical correction required 0.035 rad. It failed before dispatch
because the stricter endpoint was incorrectly also used to reject the previous
compensation reference.

Every successful fast-tracking receipt now records its own accepted angular
tolerance. Exact controller session, count and both saved targets bind that
receipt to the next start. The previous tolerance validates this starting
reference only; the next endpoint still uses its own stricter criterion.
The 0.3 rad compensation cap is not recentered or enlarged.

Legacy receipts without the acceptance field remain conservative. Do not invent
the missing value in an original run. An independent recovery audit can inspect
the original successful stage and zero-dispatch failure, but cannot rewrite
task success, authorize a release, or replay historical targets.

`tests/test_strict_reference_transition.py` replays the recorded stage transition
offline and checks strict endpoint rejection, unchanged caps and changed-state
counterexamples. Physical continuation is a separate authorized task.
