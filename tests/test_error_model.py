from __future__ import annotations

import math
import pytest

from agentic_skills_harness.contracts import ErrorCode, ErrorInfo, error_catalog
from agentic_skills_harness.contracts.serialization import ContractValidationError


def test_error_catalog_covers_standard_codes():
    catalog = error_catalog()
    assert set(catalog) == {item.value for item in ErrorCode}
    assert all({"category", "severity", "retryable", "safe_to_replan", "requires_human"} <= set(item) for item in catalog.values())


def test_error_rejects_unknown_fields_and_non_json_details():
    base = ErrorInfo(code=ErrorCode.INVALID_INPUT, category="INPUT", severity="FATAL", message="bad", retryable=False, safe_to_replan=True, requires_human=False, state_invalidated=False, details={}, source="test").to_dict()
    with pytest.raises(ContractValidationError):
        ErrorInfo.from_dict({**base, "unexpected": 1})
    with pytest.raises(ContractValidationError):
        ErrorInfo(code=ErrorCode.INVALID_INPUT, category="INPUT", severity="FATAL", message="bad", retryable=False, safe_to_replan=True, requires_human=False, state_invalidated=False, details={"value": float("nan")}, source="test")


def test_unknown_code_never_becomes_retryable():
    result = ErrorInfo(code="not-a-standard-code", message="vendor", details={}, source="vendor")
    assert result.code is ErrorCode.UNKNOWN_ERROR
    assert result.retryable is False
    assert result.safe_to_replan is False
    assert result.requires_human is True
