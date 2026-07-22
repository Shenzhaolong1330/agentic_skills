from __future__ import annotations

from pathlib import Path
from urllib.parse import urlsplit


class UnsafePathError(ValueError):
    """A path is outside an explicitly trusted artifact root."""


class SafePathPolicy:
    def __init__(self, artifact_root: str | Path, fixture_roots: tuple[str | Path, ...] = ()) -> None:
        self.artifact_root = Path(artifact_root).expanduser().resolve()
        self.fixture_roots = tuple(Path(item).expanduser().resolve() for item in fixture_roots)

    @staticmethod
    def _reject_text(value: str) -> None:
        if "\x00" in value:
            raise UnsafePathError("null byte in artifact path")
        parsed = urlsplit(value)
        if parsed.scheme or value.startswith(("//", "\\\\")):
            raise UnsafePathError("URL and network paths are forbidden")
        if value.startswith("~"):
            raise UnsafePathError("home expansion is forbidden")
        path = Path(value)
        if path.is_absolute() and not value.startswith(str(path)):
            raise UnsafePathError("absolute path is forbidden")
        if ".." in path.parts:
            raise UnsafePathError("path traversal is forbidden")

    @staticmethod
    def _inside(path: Path, root: Path) -> bool:
        try:
            path.relative_to(root)
            return True
        except ValueError:
            return False

    def output_dir(self, requested: str | Path | None = None) -> Path:
        if requested is None or str(requested) == "":
            self.artifact_root.mkdir(parents=True, exist_ok=True)
            return self.artifact_root
        value = str(requested)
        self._reject_text(value)
        candidate = Path(value).expanduser().resolve()
        if not self._inside(candidate, self.artifact_root):
            raise UnsafePathError("artifact output escapes context artifact root")
        candidate.mkdir(parents=True, exist_ok=True)
        return candidate

    def input_path(self, reference: str | Path) -> Path:
        value = str(reference)
        self._reject_text(value)
        path = Path(value)
        if path.is_absolute():
            candidates = [path]
        else:
            candidates = [self.artifact_root / path, *(root / path for root in self.fixture_roots)]
        for candidate in candidates:
            resolved = candidate.resolve()
            if any(self._inside(resolved, root) for root in (self.artifact_root, *self.fixture_roots)) and resolved.is_file():
                return resolved
        raise UnsafePathError("artifact reference is not an existing file under an allowed root")

    def validate_references(self, references: tuple[str, ...]) -> tuple[Path, ...]:
        return tuple(self.input_path(item) for item in references)
