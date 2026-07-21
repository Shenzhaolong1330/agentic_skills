# Capability Manifest v0.2 (S3)

`skill_manifest.json` is now `0.2.0`. The single source of truth remains
`skills[] -> entrypoints[]`; each entrypoint is one capability contract. The
registry flattens that structure for lookup without creating a second manual
capability list.

Every entrypoint declares a stable capability ID, SemVer version, kind,
visibility, local input/output schemas, hardware flags, side effects, risk,
resources, controlled preconditions/effects/invalidation strings, timeout,
verifier contract, standard errors, and recovery/execute policy.

`public` entries are eligible for the generated index. `internal` entries are
queryable only when explicitly requested, and `legacy` entries are retained for
compatibility but excluded by default. `allowed_as_recovery` never bypasses the
S1 `hardware_allowed` gate.

Physical entries must declare a verifier. A `physical_verification_limited`
verifier is an honest contract boundary: it records that the current evidence
is incomplete; it does not claim that physical verification has run.
Manifest validation also keeps the S1 hardware gate fields and does not require
any particular task skill to exist.
