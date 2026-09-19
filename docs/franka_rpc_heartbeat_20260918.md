# 2026-09-18 Franka RPC 心跳丢失排查

已捕获根因：RPC 的同步日志写入被 ROS launch / VS Code 终端输出背压阻塞。
不是仅凭 LostRemote 猜测：故障时读取了主线程正在执行的系统调用及其写缓冲区。

## 证据

- 12:50 单支复现：同一个 step 请求已到达服务端，服务端也收到后续两次心跳，但没有发送该请求的回复；RPC 主线程处于 `anon_pipe_write`。
- 12:58 验证：RPC PID 152611 阻塞于 `write(fd=2)` → `pipe:[531011]`，缓冲区为 `[INFO] ... Received action`；ROS launch PID 152194 阻塞于 `write(fd=1)` → `/dev/pts/4`，写入对应日志前缀。
- 关闭 `ixon` 后仍然复现。不能把 Ctrl-S 认定为根因，`stty -ixon` 不是充分修复。VS Code 为什么停止消费输出尚未单独证明。
- 故障的动作结果可能晚于客户端超时才完成，因此禁止自动重发未确认动作。

## 当前会话修复

没有重启 RPC、ROS、控制器，也没有 reset / 重置夹爪。
Franka PC 上增加了终端输出读取器 PID 157797，将原终端积压输出导入：

`/tmp/franka_log_backpressure_fix/ros_terminal_output.log`

它只读取终端输出，不发送机器人命令；原 ROS launch 退出后自动结束。
原 VS Code 终端可能不再显示所有日志片段，导流内容保存在上述文件；原有 ROS 日志仍保留。
不要在当前 ROS 会话中关闭读取器，否则原输出背压可能再次出现。

## 下次启动的持久修复

Franka PC 配置：

`/home/owen/franka_dual_ros2/src/interface_ros2/exp_env_interact/config/dual_franka_robotiq_bringup.yaml`

仅修改以下两项，其他参数保持原值：

```yaml
rpc_env:
  process_prefix: "bash /home/owen/franka_dual_ros2/rpc_log_to_file.sh"
  log_actions: false
```

前缀脚本将 RPC stdout/stderr 直接写入 `~/.ros/log/franka_rpc/rpc_日期_时间_PID.log`，从 RPC 日志路径中移除 ROS launch 终端管道。普通 ROS 日志功能仍保留。
仓库中的脚本副本为 `task_skills/demo_NO/skills/task-configurable-manipulation-demos/scripts/rpc_log_to_file.sh`。
新配置在下次正常启动 ROS 时生效；这次没有为了应用配置重启现场服务。
该 launch 的 RPC 配置含 `on_exit=Shutdown()`，单独杀死 RPC 会连带关闭整套 ROS，不要直接 kill 来应用配置。

原配置备份：

`/home/owen/franka_dual_ros2/rpc_logging_fix_backup/dual_franka_robotiq_bringup_20260918_050636.yaml`

## 验证结果和范围

导流后的真机 P2P 验证：

`/tmp/agentic_skills_runs/demo_1_rpc_probe_20260918_130753_665305`

按 message_id 对比两端系统调用记录，1,133 条 step 均满足客户端发送、服务端收到、服务端发送回复、客户端收到回复，缺失数为 0；持续约 30 秒，没有 LostRemote。
核对结果：`grasp_handover_1_rpc_trace/paired_step_verification.json`。
日志前缀测试确认 stdout/stderr 进入文件且退出码 7 原样保留。

本次单支动作没有完成交接：阶段一最终位置误差约 6.6 mm、姿态误差约 0.083 rad，超过 5 mm / 0.05 rad 容差，已在夹取前停止，没有放宽阈值。它是运动收敛失败，不是本次已定位的日志阻塞。
完整 demo 还遇到独立的 OpenRouter HTTP 402（余额不足），因此四支插入/拔出尚未完成真机验收。

## 需要重新捕获时

Demo 1 配置的 `rpc_trace_ssh_host` 默认已恢复为 `null`，避免日常 strace 开销。
设为 `prime-u7-03-franka` 可在每次 grasp_handover 开始时自动绑定双端跟踪，结束时保存到本次 artifacts 的 `grasp_handover_N_rpc_trace/`。跟踪未就绪不启动该抓取命令，不会自动重试动作。
