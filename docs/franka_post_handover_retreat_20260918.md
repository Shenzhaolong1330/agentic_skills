# 交接后 go home 导致下一次 P2P 大幅偏移

运行目录：`/tmp/agentic_skills_runs/demo_1_vial_insert_extract_20260918_135248_023864`。

## 已确认的证据

- `demo.log` 中 `post_transfer_go_home_left` 使用 joint home，报告完成后切回 Cartesian controller。
- `insert_1/result.json` 的 `observe_rack / align_holder_tcp_vertical` 阶段只要求右手转姿。
  左手起点和目标几乎相同，位置约 `[0.580159, 0.383087, -0.118061]` m。
- 同一阶段第 0 次纠偏记录左手位置误差 **0.390744 m**、旋转误差 **0.566410 rad**。
  后续 P2P 又将左手拉回目标。这证明空手在应当保持不动时发生了大幅移动，
  不是正常规划的左手避让。
- 只读 SSH 检查服务端：`dual_franka_env.py::_go_home_joints` 不更新
  `SimplifiedFrankaEnv.target_pose`；RPC `go_home` 返回观测但同样不更新该目标。
  `simplified_franka_env.py::retarget` 将后续增量累加到旧 `target_pose`。
  即使左手增量近乎零，双臂 `step` 仍会发布左手旧目标。
- 底层 Cartesian controller 的 `on_activate` 会以当前实测姿态初始化目标，
  因此不能把问题简单描述为“控制器一激活立即追旧目标”。证据指向后续增量命令重新发布旧目标。
- 新增的交接后 home 路径没有调用 `step(None)` 同步目标；已有插入技能的
  `move_side_to_home` 已做此同步，两条路径行为不一致。

## 本次修改

Demo 1/2 不再使用 `--go-home-after-transfer`，改用
`--retreat-after-transfer-m`，默认 0.20 m（YAML `handover_retreat_distance_m`）。
接收夹爪确认稳定并等待后，原手释放，等待 0.5 秒，先用 `step(None)` 同步当前目标，
再从新观测计算侧移终点：左空手 base +Y 20 cm，右空手 base -Y 20 cm。
保持 X、Z 和旋转，接收手的终点为当前姿态。侧移平移速度上限 0.04 m/s、
步长上限 0.001 m；使用原有严格到位判定，失败不回退到 home，不重发结果未确认的动作。
侧移成功后持管手直接去观察/插入，观察步骤不重复移动空手。

此改动只替换交接后的 home。旧抓取流程中交接前的 home、插入释放后的回撤/home
不在本次替换范围。下一支从侧移后的姿态开始，不能将本次改动当作之前第二支
到位误差问题已经通过真机验证。

本次排查只读取日志和服务端代码，验证使用模拟客户端，没有执行真机动作或重启服务端。
