# Execution budgets

The executor enforces the compiled envelope at task start, scheduling, node start, before and after Dispatcher calls, verification, routing, checkpoint, and next-node selection. Counters include nodes started, completed nodes, tool calls, elapsed time, recovery actions, same-error retries, node visits, and edge traversals.

Elapsed time uses an injectable monotonic clock; timestamps use an injectable UTC clock. Node and edge limits are checked before continuing, and no Dispatcher call is made after budget exhaustion. The progress fingerprint excludes run IDs, event IDs, attempt counters, timestamps, elapsed time, and temporary paths. Repeated fingerprints cause bounded `NO_PROGRESS` termination.
