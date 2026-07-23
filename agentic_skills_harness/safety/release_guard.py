from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class ReleaseGuard:
    def allow(self, *, object_not_held: bool = False, object_supported: bool = False, receiving_gripper_verified: bool = False, explicit_empty_gripper_test: bool = False) -> bool:
        return object_not_held or object_supported or receiving_gripper_verified or explicit_empty_gripper_test

    def require(self, **kwargs: bool) -> None:
        if not self.allow(**kwargs):
            raise PermissionError("release_guard_unknown_support")
