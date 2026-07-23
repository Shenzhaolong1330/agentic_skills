# Safe stop

Safe stop is distinct from E-stop, reset/home, and fault recovery. It requires
a real stop/hold/cancel/servo-stop API and verifies the resulting controller
state. It never opens a gripper, homes an arm, clears E-stop, or kills a local
process as a claim that a remote robot stopped. Missing stop API is unsupported.
