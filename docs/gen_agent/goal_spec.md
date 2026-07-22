# GoalSpec

`GoalSpec` is a closed, typed description of a structured objective. It contains a goal kind (`OBSERVATION`, `COMPUTE`, or `PHYSICAL_STATE_CHANGE`), human description, entities, structured assumptions, an S5 `PredicateSpec` success condition, failure predicates with terminal states, required evidence, ambiguities, and metadata.

Natural-language description is explanatory only. It is never sent to a command and never substitutes for a predicate or evidence requirement. Blocking ambiguities without a selected resolution are compiler errors. Non-blocking ambiguities receive a conservative warning; the compiler never expands capability, risk, workspace, or authorization based on an ambiguity.

Physical goals require evidence that can be represented by World State, Predicate Engine, and Verifier Engine. Tentative, invalidated, stale, or action-return-code-only evidence cannot prove physical success.
