from agentic_skills_harness.recovery import build_first_party_recovery_policy_registry


TASK_RECOVERY_POLICY_IDS = (
    "REOBSERVE", "SAFE_RETREAT", "ALTERNATE_CANDIDATE", "ALTERNATE_ARM",
    "RECOMPILE_REMAINDER", "REQUEST_HUMAN", "ABORT",
)


def build_task_recovery_registry(capability_registry=None):
    return build_first_party_recovery_policy_registry(capability_registry)


__all__ = ["TASK_RECOVERY_POLICY_IDS", "build_task_recovery_registry"]
