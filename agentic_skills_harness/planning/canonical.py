from __future__ import annotations

import hashlib
from typing import Any

from ..contracts.serialization import stable_dumps, to_plain


def canonical_json(value: Any) -> str:
    return stable_dumps(to_plain(value))


def digest(value: Any) -> str:
    return hashlib.sha256(canonical_json(value).encode("utf-8")).hexdigest()
