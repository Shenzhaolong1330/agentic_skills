# S0 + S1 验收说明

## 目标

S0 冻结仓库拓扑、架构、安全不变量、入口 inventory 和 baseline；S1 将真机入口收敛到统一 `hardware_allowed` Gate，并删除第二种秘密授权路径。

## 自动验收

```bash
python3 scripts/run_gen_agent_acceptance.py \
  --phase s0-s1 \
  --repo-root . \
  --output-dir /tmp/agentic_skills_gen_agent/S00_S01/final
```

报告的 `status` 只有在 branch、静态扫描、授权测试、离线硬件保护、语法检查、diff 检查和 baseline 回归约束均满足时才可为 `PASS`。最终报告不得把硬件连接次数、相机打开次数、reset 次数或运动次数写成隐含成功；本阶段所有这些计数都应为零。

## 使用边界

公开 live task 入口必须同时显式给出 `--hardware-allowed` 和（任务有物理副作用时）`--execute`。只读观测入口只需 `--hardware-allowed`。mock、dry-run 和 artifact 回放继续可在没有任何硬件授权的情况下运行。

