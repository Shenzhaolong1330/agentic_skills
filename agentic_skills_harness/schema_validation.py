"""Safe, repository-local JSON Schema loading and validation."""

from __future__ import annotations

import json
from pathlib import Path, PurePosixPath
import re
from typing import Any

from jsonschema import Draft202012Validator, FormatChecker
from referencing import Registry, Resource

from .contracts.serialization import ContractValidationError


class SchemaReferenceError(ContractValidationError):
    """Raised when a schema reference leaves the repository boundary."""


class SchemaInstanceValidationError(ContractValidationError):
    def __init__(self, schema_ref: str, errors: list[dict[str, Any]]) -> None:
        super().__init__(f"instance does not satisfy {schema_ref}: {len(errors)} error(s)")
        self.schema_ref = schema_ref
        self.errors = errors

    def to_dict(self) -> dict[str, Any]:
        return {"schema_ref": self.schema_ref, "errors": self.errors}


def _split_ref(schema_ref: str) -> tuple[str, str | None]:
    if not isinstance(schema_ref, str) or not schema_ref.strip():
        raise SchemaReferenceError("schema reference must be a non-empty string")
    if re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*://", schema_ref) or schema_ref.startswith(("file:", "http:", "https:")):
        raise SchemaReferenceError("remote and file schema references are forbidden")
    path, separator, fragment = schema_ref.partition("#")
    if not path:
        raise SchemaReferenceError("schema reference must include a local path")
    if Path(path).is_absolute() or re.match(r"^[A-Za-z]:[\\/]", path) or path.startswith(("\\", "/")):
        raise SchemaReferenceError("absolute schema references are forbidden")
    if "\\" in path:
        raise SchemaReferenceError("schema references must use POSIX separators")
    parts = PurePosixPath(path).parts
    if ".." in parts:
        raise SchemaReferenceError("schema path traversal is forbidden")
    if separator and fragment and not fragment.startswith("/"):
        raise SchemaReferenceError("schema fragments must be JSON Pointers")
    return path, (fragment if separator else None)


def _pointer_get(document: Any, fragment: str | None) -> Any:
    if not fragment:
        return document
    current = document
    for token in fragment[1:].split("/"):
        token = token.replace("~1", "/").replace("~0", "~")
        if isinstance(current, list):
            try:
                current = current[int(token)]
            except (ValueError, IndexError) as exc:
                raise SchemaReferenceError(f"JSON Pointer does not resolve: #{fragment}") from exc
        elif isinstance(current, dict) and token in current:
            current = current[token]
        else:
            raise SchemaReferenceError(f"JSON Pointer does not resolve: #{fragment}")
    return current


def resolve_schema_path(schema_ref: str, repo_root: str | Path) -> tuple[Path, str | None]:
    relative, fragment = _split_ref(schema_ref)
    root = Path(repo_root).resolve()
    path = (root / relative).resolve()
    try:
        path.relative_to(root)
    except ValueError as exc:
        raise SchemaReferenceError("schema path escapes repository root") from exc
    if not path.is_file():
        raise SchemaReferenceError(f"schema file does not exist: {relative}")
    return path, fragment


def load_local_schema(schema_ref: str, repo_root: str | Path) -> Any:
    path, fragment = resolve_schema_path(schema_ref, repo_root)
    try:
        document = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise SchemaReferenceError(f"cannot read JSON schema: {schema_ref}") from exc
    if not isinstance(document, dict):
        raise SchemaReferenceError("schema root must be an object")
    try:
        Draft202012Validator.check_schema(document)
        selected = _pointer_get(document, fragment)
        Draft202012Validator.check_schema(selected)
    except Exception as exc:
        if isinstance(exc, SchemaReferenceError):
            raise
        raise SchemaReferenceError(f"invalid Draft 2020-12 schema: {schema_ref}") from exc
    return selected


def _local_store(repo_root: Path) -> Registry[Any]:
    store: dict[str, Resource[Any]] = {}
    base = "https://agentic-skills.local/"
    for path in (repo_root / "schemas").rglob("*.json"):
        try:
            document = json.loads(path.read_text(encoding="utf-8"))
        except (OSError, json.JSONDecodeError):
            continue
        if not isinstance(document, dict):
            continue
        relative = path.relative_to(repo_root).as_posix()
        resource = Resource.from_contents(document)
        store[base + relative] = resource
        store[base + path.relative_to(repo_root / "schemas").as_posix()] = resource
        if isinstance(document.get("$id"), str):
            store[document["$id"]] = resource
    return Registry().with_resources(store.items())


def validate_json(instance: Any, schema_ref: str, repo_root: str | Path) -> list[dict[str, Any]]:
    root = Path(repo_root).resolve()
    load_local_schema(schema_ref, root)
    path, fragment = resolve_schema_path(schema_ref, root)
    document = json.loads(path.read_text(encoding="utf-8"))
    schema = _pointer_get(document, fragment)
    base_uri = "https://agentic-skills.local/" + path.relative_to(root).as_posix()
    registry = _local_store(root)
    if base_uri not in registry:
        raise SchemaReferenceError(f"schema reference is not a local repository schema: {schema_ref}")
    validator = Draft202012Validator(schema, registry=registry, format_checker=FormatChecker())
    errors = []
    for error in sorted(validator.iter_errors(instance), key=lambda item: list(item.absolute_path)):
        errors.append({
            "path": list(error.absolute_path),
            "message": error.message,
            "validator": error.validator,
        })
    return errors


def assert_valid_json(instance: Any, schema_ref: str, repo_root: str | Path) -> None:
    errors = validate_json(instance, schema_ref, repo_root)
    if errors:
        raise SchemaInstanceValidationError(schema_ref, errors)


# Short aliases keep the loader easy to discover without exposing a second implementation.
load_schema = load_local_schema
validate_instance = validate_json
