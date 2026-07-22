# Node lifecycle

The runtime state machine is:

```text
PENDING -> READY -> RUNNING -> SUCCEEDED
                         \-> FAILED -> READY (bounded retry)
                         \-> CANCELLED
PENDING -----------------> SKIPPED or CANCELLED
READY -------------------> CANCELLED
```

Terminal node states cannot be re-entered. A retry keeps the previous attempt, uses the same compiled arguments, and requires a retryable catalogued error, retry policy capacity, global retry budget, and no semantic-progress violation. Parameter adjustment, alternate candidates, and replanning return `REPLAN_REQUIRED`.

Each attempt records a stable resolved-argument digest, dispatch/verification flags, bounded timestamps, and artifact references. Large outputs are never embedded in checkpoints.
