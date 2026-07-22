from __future__ import annotations

from datetime import datetime, timedelta, timezone
from typing import Protocol


class Clock(Protocol):
    def now(self) -> datetime: ...


class SystemClock:
    def now(self) -> datetime:
        return datetime.now(timezone.utc)


class FakeClock:
    def __init__(self, value: datetime | str) -> None:
        self._value = _utc(value)

    def now(self) -> datetime:
        return self._value

    def set(self, value: datetime | str) -> None:
        self._value = _utc(value)

    def advance(self, seconds: float) -> None:
        self._value += timedelta(seconds=seconds)


def _utc(value: datetime | str) -> datetime:
    if isinstance(value, str):
        parsed = datetime.fromisoformat(value[:-1] + "+00:00" if value.endswith("Z") else value)
    else:
        parsed = value
    if parsed.tzinfo is None or parsed.utcoffset() != timezone.utc.utcoffset(parsed):
        raise ValueError("clock value must be UTC")
    return parsed.astimezone(timezone.utc)


def iso(value: datetime) -> str:
    return _utc(value).isoformat().replace("+00:00", "Z")
