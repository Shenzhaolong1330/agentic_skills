# Generic meta operations

The canonical operation map is audited by
`scripts/audit_live_capability_implementation.py`. Fixed existing bindings are
implementation-ready only in the offline sense and remain hardware-acceptance
pending. Unknown bottom-layer APIs remain contract-only or disabled.

Robot state reports missing fields as `UNKNOWN`; scene observation uses fixed
preset IDs; gripper close never creates a holding fact. All actions require a
verifier contract and conservative invalidation.
