from __future__ import annotations

from datetime import datetime, timezone
from threading import Event, RLock


class CancellationToken:
    def __init__(self) -> None:
        self._event = Event()
        self._lock = RLock()
        self.reason: str | None = None
        self.requested_at: str | None = None

    def cancel(self, reason: str = "cancelled") -> None:
        with self._lock:
            if not self._event.is_set():
                self.reason = str(reason)
                self.requested_at = datetime.now(timezone.utc).isoformat()
                self._event.set()

    @property
    def is_cancelled(self) -> bool:
        return self._event.is_set()

    def throw_if_cancelled(self) -> None:
        if self.is_cancelled:
            raise RuntimeError(self.reason or "cancelled")
