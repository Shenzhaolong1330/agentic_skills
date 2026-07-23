# Guarded move

Guarded motion requires a real bottom-layer guarded primitive and real-time
stop/cancel capability, plus distance, speed, force, torque, timeout,
direction, contact-success, and retreat contracts. A sequence of ordinary P2P
moves or end-of-segment force reads is not guarded motion. With no verified API
the capability is `CAPABILITY_UNSUPPORTED` and remains disabled.
