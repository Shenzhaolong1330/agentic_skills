from __future__ import annotations

from dataclasses import dataclass
import subprocess
from typing import Protocol

from .models import DispatchContext, InvocationPlan


@dataclass(frozen=True)
class BackendResult:
    returncode: int | None = 0
    stdout: str = ""
    stderr: str = ""
    timed_out: bool = False


class ExecutionBackend(Protocol):
    def execute(self, plan: InvocationPlan, context: DispatchContext) -> BackendResult: ...


class NoExecutionBackend:
    def execute(self, plan: InvocationPlan, context: DispatchContext) -> BackendResult:
        return BackendResult(returncode=None, stdout="", stderr="", timed_out=False)


class FakeBackend:
    def __init__(self, result: BackendResult | None = None, *, stdout: str = "", stderr: str = "", returncode: int | None = 0, timed_out: bool = False) -> None:
        self.result = result or BackendResult(returncode, stdout, stderr, timed_out)
        self.calls = 0
        self.last_plan: InvocationPlan | None = None

    def execute(self, plan: InvocationPlan, context: DispatchContext) -> BackendResult:
        self.calls += 1
        self.last_plan = plan
        return self.result


class SubprocessBackend:
    """Only executes an already-built first-party InvocationPlan."""

    def execute(self, plan: InvocationPlan, context: DispatchContext) -> BackendResult:
        try:
            completed = subprocess.run(list(plan.argv), shell=False, cwd=plan.cwd, timeout=plan.timeout_s, capture_output=True, text=True, check=False)
            return BackendResult(completed.returncode, completed.stdout[-65536:], completed.stderr[-65536:], False)
        except subprocess.TimeoutExpired as exc:
            stdout = exc.stdout.decode(errors="replace") if isinstance(exc.stdout, bytes) else (exc.stdout or "")
            stderr = exc.stderr.decode(errors="replace") if isinstance(exc.stderr, bytes) else (exc.stderr or "")
            return BackendResult(None, stdout[-65536:], stderr[-65536:], True)
