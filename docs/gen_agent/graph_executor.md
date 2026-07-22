# S7 Graph Executor

S7 executes only a `CompiledTaskGraph` produced by `TaskGraphCompiler`. It does not accept natural language, `GoalSpec`, an uncompiled `TaskGraph`, or runtime command details. It never recompiles a graph or constructs an executable, argv, adapter, backend, environment, or working directory.

Execution is deterministic and single-active-node. The only capability entry point is `CapabilityDispatcher`; every dispatch is preceded by preflight, input-schema validation, runtime preconditions, cancellation and budget checks.

Supported modes are `mock`, `dry_run`, and `from_artifacts`. Live execution is disabled in S7. `dry_run` produces `PLAN_COMPLETED` and `ExecutionScope.NONE`; it is not physical completion. Mock and artifact replay may produce simulated or replay evidence, but `physical_execution_performed` and `physical_goal_verified` are always false.

`APPROVAL` and `HUMAN_ACTION` write `human_action_request.json`, checkpoint, return `NEEDS_HUMAN`, and terminate the bounded run. S7 executes only explicit `RECOVER` nodes; automatic recovery selection and graph generation are S8 work.
