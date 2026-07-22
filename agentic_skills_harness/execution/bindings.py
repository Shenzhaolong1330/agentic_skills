from __future__ import annotations

from copy import deepcopy
from typing import Any, Mapping

from ..contracts.enums import ErrorCode
from ..contracts.serialization import ensure_jsonable, stable_dumps
from ..dispatch.error_mapping import make_error
from ..planning.canonical import digest
from ..planning.compiler import CompiledNode
from ..schema_validation import validate_json
from ..world.models import FactStatus, WorldFact
from ..world.store import WorldStateStore
from .errors import ExecutionError


FORBIDDEN_KEYS = {"executable", "argv", "env", "cwd", "adapter", "backend", "script", "hardware_allowed", "execute", "reset_path", "client_path"}


def _walk_safe(value: Any, path: str = "$") -> None:
    if isinstance(value, dict):
        for key, item in value.items():
            if str(key).lower() in FORBIDDEN_KEYS:
                raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, f"forbidden runtime binding field at {path}.{key}")
            _walk_safe(item, f"{path}.{key}")
    elif isinstance(value, list):
        for index, item in enumerate(value):
            _walk_safe(item, f"{path}[{index}]")


def json_pointer_get(value: Any, pointer: str) -> Any:
    if pointer == "":
        return value
    if not isinstance(pointer, str) or not pointer.startswith("/"):
        raise KeyError("JSON Pointer must start with /")
    current = value
    for token in pointer[1:].split("/"):
        token = token.replace("~1", "/").replace("~0", "~")
        if isinstance(current, dict) and token in current:
            current = current[token]
        elif isinstance(current, list) and token.isdigit() and int(token) < len(current):
            current = current[int(token)]
        else:
            raise KeyError(pointer)
    return current


def json_pointer_set(value: dict[str, Any], pointer: str, item: Any) -> None:
    if pointer == "":
        if not isinstance(item, dict):
            raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, "root binding must resolve to an object")
        value.clear()
        value.update(deepcopy(item))
        return
    if not pointer.startswith("/"):
        raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, "target argument path is not a JSON Pointer")
    tokens = [token.replace("~1", "/").replace("~0", "~") for token in pointer[1:].split("/")]
    current: Any = value
    for token in tokens[:-1]:
        if not isinstance(current, dict):
            raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, "target argument path traverses a non-object")
        if token not in current:
            current[token] = {}
        current = current[token]
    if not isinstance(current, dict):
        raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, "target argument path does not address an object")
    current[tokens[-1]] = deepcopy(item)


class RuntimeBindingResolver:
    def __init__(self, *, registry: Any | None = None) -> None:
        self.registry = registry

    @staticmethod
    def _world_fact(selector: Mapping[str, Any], world: WorldStateStore, *, now: str) -> WorldFact:
        data = dict(selector)
        fact_id = data.get("fact_id")
        if fact_id:
            fact = world.get_fact(str(fact_id))
            candidates = () if fact is None else (fact,)
        else:
            candidates = world.query(predicate=data.get("predicate"), entity_id=data.get("entity_id", data.get("subject_id")), source_capability_id=data.get("source_capability_id"), fresh=bool(data.get("fresh", data.get("require_fresh", False))), now=now)
        status = data.get("status", data.get("required_status"))
        expected = None if status is None else (status if isinstance(status, FactStatus) else FactStatus(str(status)))
        for fact in candidates:
            if expected is not None and fact.status != expected:
                continue
            if fact.status == FactStatus.INVALIDATED:
                continue
            if data.get("fresh", data.get("require_fresh", False)) and not fact.is_fresh(now):
                continue
            if data.get("frame") is not None and fact.frame != data["frame"]:
                continue
            if data.get("min_confidence") is not None and (fact.confidence is None or fact.confidence < float(data["min_confidence"])):
                continue
            if data.get("require_verified") is True and fact.status != FactStatus.VERIFIED:
                continue
            return fact
        raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, "world fact binding could not resolve a usable fact", {"selector": data})

    def resolve(self, node: CompiledNode, *, outputs: Mapping[str, Any], world: WorldStateStore, now: str, artifact_refs: Mapping[str, str] | None = None) -> tuple[dict[str, Any], str]:
        arguments = deepcopy(dict(node.arguments))
        for raw in node.input_bindings:
            binding = dict(raw)
            source_type = str(binding.get("source_type", ""))
            try:
                if source_type == "LITERAL":
                    resolved = deepcopy(binding.get("literal"))
                elif source_type == "NODE_OUTPUT":
                    source_node = str(binding.get("source_node_id", ""))
                    if source_node not in outputs:
                        raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, "source node output is unavailable", {"source_node_id": source_node})
                    resolved = json_pointer_get(outputs[source_node], str(binding.get("source_output_path", "")))
                elif source_type == "WORLD_FACT":
                    fact = self._world_fact(binding.get("world_fact_selector") or {}, world, now=now)
                    resolved = fact.value
                    selector = binding.get("world_fact_selector") or {}
                    if selector.get("value_path"):
                        resolved = json_pointer_get(resolved, selector["value_path"])
                else:
                    raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, "unsupported binding source", {"source_type": source_type})
                json_pointer_set(arguments, str(binding.get("target_argument_path", "")), resolved)
            except ExecutionError:
                raise
            except Exception as exc:
                raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, f"input binding failed: {exc}", {"binding_id": binding.get("binding_id")}) from exc
        try:
            arguments = ensure_jsonable(arguments, "resolved arguments")
            _walk_safe(arguments)
        except ExecutionError:
            raise
        except Exception as exc:
            raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, f"resolved arguments are not safe JSON: {exc}") from exc
        if self.registry is not None and node.capability_id is not None:
            capability = self.registry.require(node.capability_id)
            errors = validate_json(arguments, capability.input_schema_ref, self.registry.repo_root)
            if errors:
                raise ExecutionError(ErrorCode.INPUT_BINDING_FAILED, "resolved arguments failed capability input schema", {"errors": errors})
        return arguments, digest(arguments)
