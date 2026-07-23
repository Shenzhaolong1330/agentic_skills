# S10 architecture

S10A is the offline implementation and readiness-audit phase. S10H is a
separate operator procedure for a real robot. S10A never imports a vendor
runtime, opens a camera, creates RPC, runs reset, or performs motion.

The live package binds capability version, fixed adapter digest, schema
digests, workspace/calibration identity, and acceptance evidence. Readiness is
not a second identity or secret authorization system: runtime authorization
remains `hardware_allowed`, plus `execute` for side effects. The generic
GraphExecutor remains offline-only; only the fixed S10H atomic runner is a
future place for operator acceptance.
