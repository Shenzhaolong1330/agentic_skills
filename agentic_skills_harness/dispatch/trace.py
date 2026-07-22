from __future__ import annotations

import json
from pathlib import Path
from typing import Any

from ..contracts.serialization import stable_dumps, utc_now_iso


class DispatchTraceWriter:
    def __init__(self, artifact_dir: str | Path) -> None:
        self.artifact_dir = Path(artifact_dir).resolve()
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        self.events: list[dict[str, Any]] = []

    def write(self, event: dict[str, Any]) -> Path:
        safe = dict(event)
        safe.setdefault("recorded_at", utc_now_iso())
        self.events.append(safe)
        path = self.artifact_dir / "dispatch_trace.json"
        path.write_text(json.dumps(self.events, ensure_ascii=False, indent=2, sort_keys=True) + "\n", encoding="utf-8")
        return path
