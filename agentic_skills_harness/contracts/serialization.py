from __future__ import annotations

from dataclasses import fields, is_dataclass
from datetime import datetime, timezone
from enum import Enum
import json
import math
from pathlib import Path
from typing import Any, Iterable

from .enums import StrEnum


class ContractValidationError(ValueError):
    """Raised when an external contract cannot be safely decoded."""


def reject_unknown(data: Any, allowed: Iterable[str], required: Iterable[str] = ()) -> dict[str, Any]:
    if not isinstance(data, dict):
        raise ContractValidationError("contract value must be an object")
    allowed_set = set(allowed)
    unknown = sorted(set(data) - allowed_set)
    if unknown:
        raise ContractValidationError(f"unknown contract fields: {unknown}")
    missing = sorted(set(required) - set(data))
    if missing:
        raise ContractValidationError(f"missing contract fields: {missing}")
    return data


def require_string(value: Any, field_name: str, *, non_empty: bool = False) -> str:
    if not isinstance(value, str):
        raise ContractValidationError(f"{field_name} must be a string")
    if non_empty and not value.strip():
        raise ContractValidationError(f"{field_name} must not be empty")
    return value


def require_bool(value: Any, field_name: str) -> bool:
    if not isinstance(value, bool):
        raise ContractValidationError(f"{field_name} must be a boolean")
    return value


def require_number(value: Any, field_name: str, *, minimum: float | None = None, positive: bool = False) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(float(value)):
        raise ContractValidationError(f"{field_name} must be a finite number")
    number = float(value)
    if positive and number <= 0:
        raise ContractValidationError(f"{field_name} must be greater than zero")
    if minimum is not None and number < minimum:
        raise ContractValidationError(f"{field_name} must be at least {minimum}")
    return number


def require_list(value: Any, field_name: str) -> list[Any]:
    if not isinstance(value, list):
        raise ContractValidationError(f"{field_name} must be an array")
    return value


def require_object(value: Any, field_name: str) -> dict[str, Any]:
    if not isinstance(value, dict):
        raise ContractValidationError(f"{field_name} must be an object")
    return value


def require_enum(value: Any, enum_type: type[StrEnum], field_name: str) -> StrEnum:
    if isinstance(value, enum_type):
        return value
    if not isinstance(value, str):
        raise ContractValidationError(f"{field_name} must be a string enum value")
    try:
        return enum_type(value)
    except ValueError as exc:
        raise ContractValidationError(f"invalid {field_name}: {value!r}") from exc


def ensure_utc_timestamp(value: Any, field_name: str) -> str:
    timestamp = require_string(value, field_name, non_empty=True)
    normalized = timestamp[:-1] + "+00:00" if timestamp.endswith("Z") else timestamp
    try:
        parsed = datetime.fromisoformat(normalized)
    except ValueError as exc:
        raise ContractValidationError(f"{field_name} must be ISO 8601") from exc
    if parsed.tzinfo is None or parsed.utcoffset() != timezone.utc.utcoffset(parsed):
        raise ContractValidationError(f"{field_name} must contain a UTC timezone")
    return timestamp


def utc_now_iso() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="microseconds")


def to_plain(value: Any) -> Any:
    if isinstance(value, StrEnum):
        return value.value
    if isinstance(value, Enum):
        return value.value
    if isinstance(value, Path):
        return str(value)
    if is_dataclass(value):
        return {item.name: to_plain(getattr(value, item.name)) for item in fields(value)}
    if isinstance(value, dict):
        if any(not isinstance(key, str) for key in value):
            raise ContractValidationError("object keys must be strings")
        return {key: to_plain(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [to_plain(item) for item in value]
    if isinstance(value, float) and not math.isfinite(value):
        raise ContractValidationError("non-finite numbers are not JSON serializable")
    if value is None or isinstance(value, (str, int, bool, float)):
        return value
    raise ContractValidationError(f"unsupported non-JSON value: {type(value).__name__}")


def ensure_jsonable(value: Any, field_name: str = "value") -> Any:
    try:
        plain = to_plain(value)
        json.dumps(plain, ensure_ascii=False, allow_nan=False)
        return plain
    except (TypeError, ValueError, ContractValidationError) as exc:
        raise ContractValidationError(f"{field_name} must be JSON serializable") from exc


def stable_dumps(value: Any) -> str:
    return json.dumps(to_plain(value), ensure_ascii=False, allow_nan=False, sort_keys=True, separators=(",", ":"))
