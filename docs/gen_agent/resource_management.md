# Resource management

`ResourceLockManager` is an in-memory manager for `shared` and `exclusive` resources. Requirements are sorted by canonical resource ID before acquisition, so partial acquisition cannot deadlock. A failed acquisition rolls back the current request; `finally`-style release covers success, failure, timeout, cancellation, and retry.

Locks are not persisted. Resume clears the lock table and rebuilds it from the active node. S7 schedules one node at a time and does not implement real parallel execution. A graph that explicitly requires concurrency is rejected with `PARALLEL_EXECUTION_NOT_SUPPORTED_IN_S7`.
