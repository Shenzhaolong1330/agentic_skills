from __future__ import annotations

from datetime import datetime
import json
from pathlib import Path
from typing import Any

from .types import SkillContext, to_plain, write_json


def new_run_id(prefix: str = "run") -> str:
    return f"{prefix}_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"


class TraceWriter:
    def __init__(self, artifact_dir: str | Path):
        self.artifact_dir = Path(artifact_dir)
        self.artifact_dir.mkdir(parents=True, exist_ok=True)
        self.command_plans: list[dict[str, Any]] = []

    def path(self, name: str) -> Path:
        return self.artifact_dir / name

    def write_context(self, context: SkillContext) -> Path:
        path = self.path("context.json")
        write_json(path, context)
        return path

    def write_manifest_snapshot(self, manifest: dict[str, Any]) -> Path:
        path = self.path("manifest_snapshot.json")
        write_json(path, manifest)
        return path

    def write_health(self, name: str, health: Any) -> Path:
        path = self.path(name)
        write_json(path, health)
        return path

    def write_reset(self, index: int, result: Any) -> Path:
        path = self.path(f"reset_recovery_{index}.json")
        write_json(path, result)
        return path

    def write_stage(self, index: int, stage: str, result: Any) -> Path:
        safe_stage = "".join(ch if ch.isalnum() or ch in "._-" else "_" for ch in stage)
        path = self.path(f"stage_{index:02d}_{safe_stage}.json")
        write_json(path, result)
        return path

    def append_command_plan(self, plan: Any) -> Path:
        self.command_plans.append(to_plain(plan))
        path = self.path("command_plan.json")
        path.write_text(json.dumps(self.command_plans, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
        return path

    def write_task_result(self, result: Any, output_json: str | Path | None = None) -> Path:
        path = Path(output_json) if output_json else self.path("task_result.json")
        write_json(path, result)
        if path != self.path("task_result.json"):
            write_json(self.path("task_result.json"), result)
        return path
