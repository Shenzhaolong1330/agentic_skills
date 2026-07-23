# Fault recovery

Fault recovery is distinct from `procedure.reset_home`. It requires a known
current error, clear E-stop/unsafe state, no confirmed/tentative/unknown held
object, and an explicitly recovery-allowed capability. Recovery re-observes
robot state, invalidates affected World State, and never resumes a task
automatically. E-stop is never auto-recovered.
