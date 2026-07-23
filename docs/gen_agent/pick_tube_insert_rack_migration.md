# pick_tube_insert_rack migration audit

The offline task implementation is under
`task_skills/pick_tube_insert_rack/agentic/`. The mapping is machine-auditable
with `scripts/audit_task_migration.py`; it covers health preflight, tube and
rack observation, arm/candidate selection, geometry and grasp planning,
handover, insertion, release, retract, final verification, and abnormal
recovery.

The new graph has explicit nodes for observations, pure computations,
action-plan emission, and verifiers. It is not a single opaque task node.
Internal capability IDs are task-scoped and allowlisted in the task envelope;
they do not expose legacy shell entrypoints. The fake dispatcher returns
deterministic facts and plans and never opens a camera, connects to RPC, moves
an arm, or controls a gripper.

Example audit command:

```bash
python scripts/audit_task_migration.py --output-json /tmp/pick_tube_migration_audit.json
```

Compatibility remains deliberate: the legacy runner preserves its result
shape, while offline execution uses the new `GoalSpec`/`TaskGraph` runtime and
adds graph, lineage, recovery, and physical-execution flags. The legacy live
path is retained behind the existing shared hardware gate and was not run by
S8/S9 acceptance. S10 is reserved for any future real Adapter and pre-hardware
audit; this migration does not make the task live-ready.
