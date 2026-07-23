# Relative motion

Relative motion observes a fresh current EE pose, validates an explicit
`base` or `tool` reference frame, composes a target, checks conservative step
and workspace limits, and delegates to the fixed move-to-pose contract. It
does not call a raw RPC and rejects stale or unavailable current pose.
