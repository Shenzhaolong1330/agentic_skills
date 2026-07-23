# Events and checkpoints

Every run writes an append-only `events.jsonl` hash chain. Events contain sequence, causal event, run/graph identity, payload, previous digest, and event digest. Reads verify JSON, sequence, identity, and the complete chain; corruption fails closed.

`checkpoint.json` is canonical JSON with a digest and is written through a temporary file, flush/fsync, and `os.replace`. It stores graph identity, plan and registry digests, node records, counters, budget usage, world snapshot, progress fingerprints, and relative artifact references. It never stores executable data, secrets, pickle, or large output bodies.

Resume revalidates the compiled graph, manifest, capability index, schema digests, event chain, checkpoint digest, and world schema. A successful node is loaded from its artifact and is not dispatched again. Unknown in-flight results are conservative and become `EXECUTION_INTERRUPTED`; physical retry is not implemented.
# S8 additions

Recovery context, candidate evaluation, selection/rejection, attempts,
replanning, plan replacement, and lineage updates have dedicated event types.
The recovery state is stored in the checkpoint alongside the S7 executor
state, so a resumed run can verify both the event chain and the active plan
hash before continuing.
