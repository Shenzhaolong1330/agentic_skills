结论：最近 10 次运行的主日志都在，时间范围是 2026-07-16 00:49:52–01:33:53。10 个 `full_flow.log` 均非空且有结束汇总；当前也没有残留运行进程。

但只有 7/10 次实现“两支试管均软件判定完成”。另外 3 次各失败一支。

### 统计结果

| 运行 | 完成试管 | 硬失败 | 上提软告警 |
|---|---:|---|---:|
| [004952](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_004952/full_flow.log:31704) | 2/2 | 无 | 2 |
| [005502](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_005502/full_flow.log:31500) | 2/2 | 无 | 1 |
| [005954](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_005954/full_flow.log:18341) | 1/2 | tube 2，stage4 交接返回失败 | 1 |
| [010350](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_010350/full_flow.log:130) | 1/2 | tube 1，stage3 抓取下降失败 | 1 |
| [010812](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_010812/full_flow.log:33509) | 2/2 | 无 | 2 |
| [011251](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_011251/full_flow.log:29313) | 2/2 | 无，发生过控制器恢复 | 0 |
| [011732](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_011732/full_flow.log:30862) | 2/2 | 无 | 0 |
| [012157](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_012157/full_flow.log:13578) | 1/2 | tube 2，stage4 交接返回失败 | 0 |
| [012531](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_012531/full_flow.log:31538) | 2/2 | 无 | 0 |
| [012954](/tmp/agentic_skills_runs/full_pick_tube_insert_rack_20260716_012954/full_flow.log:31896) | 2/2 | 无 | 1 |

汇总：

- 运行级硬失败：3/10，30%
- 试管级完整软件成功：17/20，85%
- 抓取/交接失败：3/20，15%
- 进入插入阶段：17 次
- 插入硬失败：0/17
- 力保护碰撞失败：0/17
- 插入结果：15 次 `seated_axial_contact`，2 次 `commanded_pose_reached`
- 失败后 full reset：3 次
- 释放后上提软告警：8/17，47.1%

这里说的是“软件判定成功”。日志没有插入后的独立视觉验收，因此不能仅凭这些日志保证 17 支物理状态全部正确。

### 三次硬失败原因

1. `005954` tube 2

已经夹住试管，返回交接位时失败。最终左臂旋转残差为 `0.0509317 rad`，阈值是 `0.05 rad`，仅超出约 `0.053°`；第二次重试左臂位姿完全没变化。

远端 ROS 日志同时记录：

```text
1784134984.288 / 01:03:04
communication_constraints_violation
```

因此这次不是单纯阈值误差：底层通信约束异常让左臂控制器失效，而 stage4 没有启用 stall recovery，最终触发 full reset。

2. `010350` tube 1

下降第一轮已经移动约 `75.5 mm`，位置误差只有约 `3.6 mm`，但旋转误差 `0.05768 rad > 0.05 rad`。第二轮完全零运动，控制器恢复虽然返回成功，第三轮也只移动 `0.217 mm`，最终旋转误差仍为 `0.05782 rad`。

远端 ROS 日志在 `01:04:48` 明确记录了同样的 `communication_constraints_violation`。这是控制器/实时通信故障，不是 VLM/SAM 定位失败。

3. `012157` tube 2

没有对应的底层 ROS abort。两次 stage4 后左臂旋转误差分别为：

- `0.052566 rad`
- `0.051900 rad`

位置误差均约 4 mm，只有旋转略高于 `0.05 rad`。这是明确的 stage4 硬阈值问题。

### 随机还是代码问题

不是单一原因，而是“间歇性底层异常 + 代码恢复缺口”。

- 这 44 分钟内远端 ROS 共出现 3 次 `communication_constraints_violation`：`005954`、`010350` 和 `011251`。`011251` 被代码恢复并继续成功，前两次导致任务失败。发生时机是随机的，但频率已经不能视为偶然噪声，属于控制器实时通信的系统性风险。
- stage1–3 会传入 `recover_stalled_side`，但 stage4 没传，因此“已经夹住试管之后”反而没有 stall recovery：[grasp_right_arm_xyz.py](/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/grasp_right_arm_xyz.py:982)。
- 19 次 stage4 中，17 次成功的左臂最终旋转误差全部贴着 `0.05 rad`，范围约 `0.04834–0.04999 rad`。这是明显的阈值悬崖。代码已经支持单独的 `--transition-rotation-tolerance-rad`，但当前包装脚本只传统一的 `0.05 rad`：[grasp_right_arm_xyz.sh](/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/grasp_right_arm_xyz.sh:71)。
- stage4 失败后，顶层不区分“尚未抓住”和“正在持管”，统一 full reset 并打开夹爪：[run_full_pick_tube_insert_rack.sh](/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/run_full_pick_tube_insert_rack.sh:385)。这是高风险实现；日志无法证明两支持管失败时试管最终落在哪里。
- `COMPLETE` 只是循环结束。脚本随后检测失败计数并退出 1：[run_full_pick_tube_insert_rack.sh](/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/run_full_pick_tube_insert_rack.sh:495)。

另外，8 次上提告警都是 `retract_after_release_p2p_failed`。这 8 次上提的 Z 轴残差都不超过约 `2.33 mm`，主要是 XY/旋转使完整 6D 容差失败，随后回 home 没有失败。因此更像“判定标准不符合上提安全目标”，不是 8 次都没抬起来。但代码只记录内部 warning，顶层仍打印插入成功，这个告警目前被隐藏了：[insert_down_from_current_pose.py](/home/deepcybo/agentic_skills/task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/insert_down_from_current_pose.py:901)。

优先应修复：stage4 增加持管状态下的控制器恢复、恢复后验证真实运动、禁止持管故障直接开夹爪；之后再考虑将 stage4 专用旋转容差调整到约 `0.055–0.06 rad`。不能只放宽容差，否则可能把控制器已经掉线的情况误判成成功。

本次只进行了只读日志和代码分析，没有修改文件。另一个审计风险是脚本没有把原始 argv 写入日志，因此日志能确认 `LIVE hardware flow`，但不能严格证明每次都没有附加其他参数。