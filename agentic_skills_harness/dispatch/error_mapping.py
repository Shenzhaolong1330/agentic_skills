from __future__ import annotations

from typing import Any

from ..contracts.enums import ErrorCode
from ..contracts.errors import ERROR_CATALOG, ErrorInfo, standardize_error_code


def make_error(code: str | ErrorCode, message: str, *, details: dict[str, Any] | None = None, source: str = "dispatcher") -> ErrorInfo:
    normalized, normalized_details = standardize_error_code(code, details)
    catalog = ERROR_CATALOG[normalized]
    return ErrorInfo(code=normalized, category=catalog["category"], severity=catalog["severity"], message=message, retryable=catalog["retryable"], safe_to_replan=catalog["safe_to_replan"], requires_human=catalog["requires_human"], details=normalized_details, source=source)


def map_structured_errors(payload: dict[str, Any]) -> tuple[ErrorInfo, ...]:
    raw = payload.get("errors", [])
    if isinstance(payload.get("error"), dict):
        raw = [payload["error"], *raw] if isinstance(raw, list) else [payload["error"]]
    if not isinstance(raw, list):
        return (make_error(ErrorCode.UNKNOWN_ERROR, "structured errors field is not an array", source="structured_output"),)
    result: list[ErrorInfo] = []
    for item in raw:
        if isinstance(item, dict):
            result.append(make_error(item.get("code", "UNKNOWN_ERROR"), item.get("message", "capability reported an error"), details=dict(item.get("details") or {}), source="structured_output"))
        else:
            result.append(make_error(ErrorCode.UNKNOWN_ERROR, "capability reported a malformed error", details={"raw": str(item)}, source="structured_output"))
    return tuple(result)


def map_legacy_text(stderr: str, stdout: str, declared: tuple[ErrorCode, ...] = ()) -> tuple[ErrorInfo, ...]:
    text = f"{stderr}\n{stdout}".lower()
    patterns = (("estop", ErrorCode.ROBOT_ESTOP_OR_UNSAFE), ("unsafe", ErrorCode.ROBOT_ESTOP_OR_UNSAFE), ("timeout", ErrorCode.MOTION_TIMEOUT), ("not found", ErrorCode.PERCEPTION_NOT_FOUND), ("grasp", ErrorCode.GRASP_NOT_CONFIRMED), ("release", ErrorCode.RELEASE_NOT_CONFIRMED), ("target", ErrorCode.MOTION_TARGET_NOT_REACHED))
    for marker, code in patterns:
        if marker in text and (not declared or code in declared):
            return (make_error(code, f"legacy output matched {marker}", details={"classification_source": "legacy_text_fallback"}, source="legacy_text_fallback"),)
    if stderr.strip():
        return (make_error(ErrorCode.UNKNOWN_ERROR, "capability wrote an unclassified error", details={"classification_source": "legacy_text_fallback"}, source="legacy_text_fallback"),)
    return ()
