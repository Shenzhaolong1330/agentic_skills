from __future__ import annotations

import hashlib
import json
import os
from pathlib import Path
import uuid
from typing import Any, Mapping

from ..contracts.serialization import stable_dumps
from ..contracts.enums import ErrorCode
from .errors import CheckpointError
from .events import ArtifactStore


class CheckpointStore:
    def __init__(self, artifact_dir: str | Path, *, fsync: bool = True) -> None:
        self.artifacts = ArtifactStore(artifact_dir)
        self.path = self.artifacts.path("checkpoint.json")
        self.fsync = fsync

    @staticmethod
    def digest(payload: Mapping[str, Any]) -> str:
        unsigned = dict(payload)
        unsigned.pop("checkpoint_digest", None)
        return hashlib.sha256(stable_dumps(unsigned).encode("utf-8")).hexdigest()

    def write(self, payload: Mapping[str, Any]) -> str:
        value = dict(payload)
        value["checkpoint_digest"] = self.digest(value)
        temporary = self.artifacts.path(f".checkpoint.{uuid.uuid4().hex}.tmp")
        try:
            with temporary.open("w", encoding="utf-8") as handle:
                handle.write(stable_dumps(value) + "\n")
                handle.flush()
                if self.fsync:
                    os.fsync(handle.fileno())
            os.replace(temporary, self.path)
            if self.fsync:
                directory = os.open(self.artifacts.root, os.O_RDONLY)
                try:
                    os.fsync(directory)
                finally:
                    os.close(directory)
        except Exception as exc:
            try:
                temporary.unlink(missing_ok=True)
            except OSError:
                pass
            raise CheckpointError(ErrorCode.CHECKPOINT_CORRUPT, f"checkpoint atomic write failed: {exc}") from exc
        return "checkpoint.json"

    def load(self) -> dict[str, Any]:
        if not self.path.exists() or self.path.is_symlink():
            raise CheckpointError(ErrorCode.CHECKPOINT_CORRUPT, "checkpoint is missing or is a symlink")
        try:
            value = json.loads(self.path.read_text(encoding="utf-8"))
            if not isinstance(value, dict) or value.get("checkpoint_digest") != self.digest(value):
                raise ValueError("checkpoint digest mismatch")
            return value
        except CheckpointError:
            raise
        except Exception as exc:
            code = ErrorCode.CHECKPOINT_MISMATCH if "digest" in str(exc) else ErrorCode.CHECKPOINT_CORRUPT
            raise CheckpointError(code, f"checkpoint load failed: {exc}") from exc
