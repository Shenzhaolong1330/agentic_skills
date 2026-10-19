---
name: robot-failure-improvement
description: Use after a robot task fails to investigate infra constraints, debug or extend skills, and preserve validated experience. Applies to offline maintenance of this workspace, not robot motion or automatic replay of failed actions.
---

Read the job's `reflection.json` and `MAINTENANCE.md`. Evidence and model assessments are hypotheses to verify against source files and raw receipts. Inspect `python3 tools/workspace_git.py status` and `audit` before modifying repositories.

Address each layer, recording either an implemented change, an evidence-backed no-change decision, or a deferred item with its missing evidence:

- **Infra:** trace the effective values at the tool/adapter/controller boundaries. Compare task accuracy, path bounds, compensation limits, settling conditions and hardware constraints separately. A constraint may be unnecessarily strict when the measured state satisfies the task yet a conflicting internal condition rejects it. Reproduce that mismatch before adjusting code. Lost ACK, stale native telemetry and physical contact are not reasons to relax limits.
- **Skill:** replay the failing input without hardware. Check defaults, argument propagation, frames, attachment assumptions, release conditions and explicit postconditions. Add a reusable capability if the task cannot be expressed by the existing tool catalog; register its adapter as well as its documentation. Do not turn a task-specific historical coordinate into a default target.
- **Experience:** capture a conditional rule in `robot_agent_lab/experiences/<layer>/<descriptive-id>.md`. Include evidence references, applicable tools, a counterexample and validation level. Keep untested hypotheses `candidate`; only reviewed lessons enter planning. The frontmatter is a JSON object between `---` lines; use an existing experience file as the schema example.

Complete authorized offline fixes, rather than stopping with suggestions. Prefer a narrow regression reproducing the original failure. Do not run the failed motion, reset, release a held object or install speculative trajectories as part of maintenance. A later physical validation is a separate execution task. Keep the original run outcome unchanged.

Commit code in its owning repository, then update parent gitlinks from the deepest parent outward. Preserve other staged and unstaged changes; use an isolated index or carefully scoped hunks when files contain unrelated work. Record source run IDs, changed files, test results, commits and unresolved risks in `robot_agent_lab/maintenance/CHANGELOG.md`.

Use `./run_agent.sh reflect complete --job <job-directory> --record <record.json>` to attach a closure record after validation. Its `layers` object must include infra, skill and experience, each with status `updated`, `no_change` or `deferred` and a reason. Include `job_id`, `validation` and `commits`. Deferred physical work stays explicit; registering maintenance does not certify physical success.
