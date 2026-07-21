# S0/S1 基线记录

- base SHA: `c454585fe6a4cfd5ad18d3743fde5a514a380f03`
- branch: `develop/gen_agent`
- 修改前工作区：干净
- `git diff --check`: 通过
- `python3 -m compileall -q agentic_skills_harness`: 通过
- `python3 -m pytest -q tests`: 未运行成功，系统 Python 3.9.6 未安装 `pytest`；原始日志见 `/tmp/agentic_skills_gen_agent/S00_S01/baseline/pytest-q-tests.log`
- 等价命令 `python3 -m unittest discover -s tests -v`: 18 passed，2 collection error；原始日志见 `/tmp/agentic_skills_gen_agent/S00_S01/baseline/unittest-tests.log`
- collection error 签名：`tests/test_grasp_controller_recovery.py` 与 `tests/test_insertion_retry_logic.py` 导入 `numpy` 失败（`ModuleNotFoundError`）。这两个错误是 baseline known failure，不由本阶段掩盖。
- skipped: 0
- 硬件限制：baseline 未打开相机、未连接 RPC、未启动 ROS/SAM/RealSense 服务、未执行 reset、运动或夹爪动作。
- 拓扑限制：`git submodule status --recursive` 因缺失 `.gitmodules` 与现存 gitlink 不匹配而失败；详见 `repository_topology.md`。

