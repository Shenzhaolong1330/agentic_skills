# World State (S5)

`WorldFact` is a task-independent fact with an `EntityRef`, predicate, value,
status, confidence, frame, UTC observation time, optional expiry/TTL, source
capability and version, artifact references, calibration hash, revision, and
metadata. `WorldStateStore` is an in-memory deterministic store with monotonic
revisions, provenance-preserving history, query filters, snapshots, and
restore.

`INVALIDATED` facts are excluded by default. `TENTATIVE` facts are never final
goal evidence. A stale fact remains available for inspection but cannot satisfy
a fresh predicate. Clock injection keeps freshness tests independent of sleep.
## Planning integration

S6 compiled graphs may reference world facts and PredicateSpec evidence, but no Graph Executor currently updates or verifies World State. Physical success still requires a later S5 Verifier path.
