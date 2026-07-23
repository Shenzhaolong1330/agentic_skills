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
# S7 integration

OBSERVE updates World State only through explicit compiled fact projections. ACT invalidates conservatively and may create only TENTATIVE simulated effects; dry-run creates no physical effect. VERIFIED facts require the Verifier.
# Recovery fact handling

Recovery records invalidated facts explicitly. Holding, insertion, release,
pose, and occupancy facts are never inferred from a command result; recovery
must obtain fresh observations before treating the remainder as safe.
