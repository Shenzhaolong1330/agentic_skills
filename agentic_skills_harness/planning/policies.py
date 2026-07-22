from __future__ import annotations

from typing import Any

from ..contracts.serialization import ContractValidationError, ensure_jsonable


FORBIDDEN_EXECUTION_FIELDS = {
    "hardware_allowed", "execute", "operator_token", "env", "environment", "executable", "argv",
    "adapter", "adapter_id", "backend", "script", "cwd", "python_path", "client_path", "reset_script",
    "config_path", "output_path", "result_path", "extra_args", "passthrough_args", "shell",
}


def reject_execution_fields(value: Any, path: str = "$", *, error_type: type[Exception] = ContractValidationError) -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            if not isinstance(key, str):
                raise error_type(f"{path} keys must be strings")
            if key.lower() in FORBIDDEN_EXECUTION_FIELDS:
                raise error_type(f"forbidden execution field at {path}.{key}")
            reject_execution_fields(item, f"{path}.{key}", error_type=error_type)
    elif isinstance(value, list):
        for index, item in enumerate(value):
            reject_execution_fields(item, f"{path}[{index}]", error_type=error_type)
    else:
        try:
            ensure_jsonable(value, path)
        except Exception as exc:
            raise error_type(str(exc)) from exc


def require_id(value: Any, field: str) -> str:
    if not isinstance(value, str) or not value.strip() or any(char.isspace() for char in value):
        raise ContractValidationError(f"{field} must be a non-empty identifier")
    return value
