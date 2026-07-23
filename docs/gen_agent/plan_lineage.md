# Plan lineage and checkpoints

Every initial plan and replacement plan has a stable graph hash and a
`PlanLineage` record. A lineage entry records the parent plan, triggering
failure, invalidated facts, replan request, replacement graph, and the
decision/attempt identifiers that caused the transition.

The `GraphExecutor` checkpoint contains the normal execution state plus a
nested recovery state. Recovery state includes the serialized context,
candidate evaluation, selected strategy, attempts, replan request/result,
lineage, and budget snapshot. It is written through the same atomic
checkpoint path and is included in the hash-chained event stream.

Resume is fail-closed: a checkpoint with a broken event chain, changed graph
hash, incompatible mode/envelope, or inconsistent lineage is not resumed.
The records are for audit and deterministic replay; they do not contain
executable callbacks, shell text, adapter objects, or hardware handles.
