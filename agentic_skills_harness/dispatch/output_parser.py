from __future__ import annotations

import json
from typing import Any


class OutputParseError(ValueError):
    pass


def parse_json_output(stdout: str) -> dict[str, Any]:
    if not stdout.strip():
        return {}
    try:
        value = json.loads(stdout)
    except json.JSONDecodeError as exc:
        raise OutputParseError("stdout is not valid JSON") from exc
    if not isinstance(value, dict):
        raise OutputParseError("structured capability output must be a JSON object")
    return value
