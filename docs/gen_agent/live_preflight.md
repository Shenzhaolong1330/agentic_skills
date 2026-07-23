# Live preflight

`LivePreflightPolicy` checks mode, `hardware_allowed`, `execute` for side
effects, readiness/evidence identity, robot health, E-stop, resources, input
schema, workspace, motion/force limits, held-object safety, and verifier
availability. Any failure returns structured reasons and permits zero backend
calls. It never updates acceptance evidence and is not connected to live
GraphExecutor execution.
