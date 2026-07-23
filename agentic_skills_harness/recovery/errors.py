class RecoveryContractError(ValueError):
    """Invalid declarative recovery policy or runtime recovery payload."""


class RecoveryExhaustedError(RuntimeError):
    """All bounded recovery options have been consumed."""
