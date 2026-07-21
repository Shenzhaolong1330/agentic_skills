# Gen-agent environment

`agentic-gen-agent` is an isolated Conda environment for the outer harness,
JSON Schema contracts, manifest validation, offline task logic tests, and
static acceptance. It does not replace Conda `base`, ROS, an atomic skill's
virtualenv, a robot RPC environment, or model/camera environments.

Create or incrementally update it with:

```bash
./scripts/bootstrap_gen_agent_env.sh
```

The script prefers `mamba`, then `conda`, creates the environment only when it
is absent, and uses an update without `--prune` when it already exists. It
does not use interactive activation. Run later commands explicitly through:

```bash
conda run -n agentic-gen-agent python <script>
conda run -n agentic-gen-agent python -m pytest -q tests
```

The dependency bounds are intentionally small and reproducible. `opencv-python-headless`
is used only for offline imports and task-logic tests; no camera is opened by
this environment check or by the test suite. Hardware packages including
`pyrealsense2`, ROS bindings, robot SDKs, CUDA, Torch, Transformers, VLM/SAM
models, and RPC servers are intentionally not dependencies here.

The checker writes `/tmp/agentic_skills_gen_agent/S02_S03/environment/environment_report.json`.
The report records the interpreter, package imports, Draft 2020-12 support,
and harness import status. Temporary reports are not committed.
