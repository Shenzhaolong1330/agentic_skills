# Predicate Engine (S5)

`PredicateSpec` is a closed data language supporting existence, equality and
numeric comparisons, ranges, freshness, confidence, frames, pose error,
relations, health/resources, and bounded `all`/`any`/`not` composition.

The engine never evaluates Python expressions, imports functions, runs user
regular expressions, reads files, or treats missing data as true. Results are
structured `PredicateResult` values with satisfaction, operator, evidence fact
IDs, reason, errors, and evaluation time.
