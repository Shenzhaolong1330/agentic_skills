# State Invalidation (S5)

`InvalidationEngine` accepts controlled selectors from manifest invalidation
contracts: exact fact IDs, subject/entity prefixes, predicates, source
capabilities, and safe dotted prefixes. It does not evaluate expressions.

Reset/recovery/home has a platform minimum: robot pose/state, gripper state,
holding and held-object relations, and dynamic object poses are invalidated
even if a declaration is incomplete. Gripper open/release invalidates holding
and requires independent observation; gripper close never creates `VERIFIED`
holding. Planned or denied actions do not invalidate state.
