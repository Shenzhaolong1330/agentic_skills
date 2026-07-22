from __future__ import annotations

from typing import Any

from ..contracts.results import ActionResult


class EffectProjector:
    """Conservative marker for S7 effect policy.

    Dry-run and planned-only results never project physical effects. A caller
    may use this helper to make that invariant explicit before verification.
    """

    @staticmethod
    def may_project_tentative(result: ActionResult, *, mode: str) -> bool:
        return mode == "mock" and result.command_executed and not result.planned_only
