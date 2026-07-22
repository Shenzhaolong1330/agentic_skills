"""Task-independent world facts, predicates, and invalidation."""

from .clock import Clock, FakeClock, SystemClock
from .models import EntityRef, FactStatus, WorldFact
from .predicates import PredicateEngine, PredicateResult, PredicateSpec
from .snapshot import WorldSnapshot
from .store import WorldStateStore
from .invalidation import InvalidationEngine

__all__ = ["Clock", "EntityRef", "FactStatus", "FakeClock", "InvalidationEngine", "PredicateEngine", "PredicateResult", "PredicateSpec", "SystemClock", "WorldFact", "WorldSnapshot", "WorldStateStore"]
