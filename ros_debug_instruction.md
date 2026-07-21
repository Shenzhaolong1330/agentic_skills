结论：这不是普通 ROS 指令“偶尔没响应”，而是 Franka 的 1 kHz FCI 实时链路发生了超时。当前证据更支持“主机实时配置没有完整生效 + 左臂特有链路风险”叠加，暂时不能把问题单独归因于网线或控制器硬件。

```text
1 ms 周期未满足
→ 连续约 20 周期无有效控制包
→ communication_constraints_violation
→ hardware read 返回 ERROR
→ controller_manager 停用控制器
→ 上层表现为机械臂不动/恢复失败
```

Franka 官方说明也是连续约 20 ms 收不到命令就停止控制，并建议监控 `control_command_success_rate`。[Franka FCI 文档](https://frankarobotics.github.io/docs/doc/libfranka/docs/overview.html)

## SSH 检查结果

- 内核确实是 `PREEMPT_RT`，controller manager 也成功进入 `SCHED_FIFO 98`。
- “You are not using a real-time kernel”是误报：当前 libfranka 0.20.4 只检查 `/sys/kernel/realtime`，但这个 RT 内核没有该文件。
- 更大的问题是该误判还把 `RealtimeConfig` 从 `kEnforce` 降成了 `kIgnore`，以后 libfranka 设置实时调度失败也不会阻止启动。

故障明显偏左：

- 最近三次准确发生在 2026-07-16 01:03:04、01:04:48、01:15:02，全部是左臂。
- 本次开机以来，左臂有 22 次 communication violation，右臂有 2 次。
- 但历史上右臂也发生过，所以还不能直接判定左机器人硬件损坏。

主机实时配置存在几个确定风险：

- 内核隔离了 CPU 8–11，但故障时段这四个 CPU 全部 `100% idle`，控制线程实际没有使用隔离核。
- CPU 8–11 的 C3 唤醒延迟标称 `1048 μs`，已经超过 FCI 的 1 ms 总周期。
- 当前没有进程持续持有 `/dev/cpu_dma_latency`。
- 短测 cyclictest 在隔离核最大延迟约 210–249 μs；该主机历史测试显示开启 latency QoS 后可降低到约 4–5 μs。
- `/etc/security/limits.conf` 和 `/etc/security/limits.d/99-realtime.conf` 的 `priority 99`，在本机实际把所有 `owen` 普通进程变成了 `NI=19`。这不影响已经切到 FIFO98 的控制循环，但会拖慢 RPC、executor、spawner 和服务线程，很可能参与了“控制器有时不响应”。

网络没有发现持续性物理故障：

- `enp87s0` 为 1 Gbps Full Duplex。
- CRC、RX/TX error、missed、no-buffer、timeout、drop 都是 0。
- 故障时段利用率约 2.27%。
- 左右机器人同时 ping 均为 0% 丢包，最大 RTT 约 0.31 ms。

但当前 GRO/GSO/TSO 仍开启，收发中断合并为 3 μs，说明 `franka_fci_rt_tune.sh` 本次启动后没有生效。普通 ping 和网卡计数也看不到“包已经到主机，但控制线程处理晚了”的情况。

## 建议按这个顺序修复

1. 修正 PAM limits

删除两个文件中的 `soft/hard priority 99`，并把配置统一保留在 `/etc/security/limits.d/99-realtime.conf`：

```text
@realtime soft rtprio 99
@realtime hard rtprio 99
@realtime soft memlock unlimited
@realtime hard memlock unlimited
```

重新登录后确认：

```bash
ps -o ni,cls,rtprio -p $$
ulimit -r
ulimit -l
```

普通 shell 应为 `NI=0`，实时优先级上限为 99。

2. 只绑定 1 kHz 控制线程，不要 `taskset/chrt` 整个进程

在 `franka_bringup/config/controllers.yaml` 增加：

```yaml
/left_arm:
  controller_manager:
    ros__parameters:
      cpu_affinity: 8
      lock_memory: true

/right_arm:
  controller_manager:
    ros__parameters:
      cpu_affinity: 10
      lock_memory: true
```

ros2_control 4.44 会在线程内部应用 `cpu_affinity`，不会把 executor/service 一起锁到隔离核，比顶层 `taskset` 更合适。[ros2_control 4.44 源码](https://github.com/ros-controls/ros2_control/blob/4.44.0/controller_manager/src/ros2_control_node.cpp)

3. 同时处理 C-state/PM QoS

不能只绑核。建议做一个 root systemd 服务，在 ROS 运行期间持续持有 `/dev/cpu_dma_latency=0`。注意必须保持文件描述符打开，普通 `echo 0` 后立即关闭没有持续效果。

第一次 A/B 测试也可临时只禁用 CPU8、10 的 C2/C3：

```bash
for c in 8 10; do
  echo 1 | sudo tee \
    /sys/devices/system/cpu/cpu$c/cpuidle/state2/disable \
    /sys/devices/system/cpu/cpu$c/cpuidle/state3/disable
done
```

4. 调整 NIC，但不要原样执行现有 tune 脚本

建议关闭 offload 和中断合并：

```bash
sudo ethtool -K enp87s0 gro off gso off tso off lro off
sudo ethtool -C enp87s0 rx-usecs 0 tx-usecs 0
```

EEE 当前已经关闭。

现有 `franka_fci_rt_tune.sh` 会把所有 NIC IRQ 都压到 CPU1，不适合双臂共享的多队列网卡。应保留目前队列 IRQ 分散在 CPU2–5，让控制线程独占 CPU8、10。

5. 修正 libfranka RT 检测

给 `libfranka/src/control_tools.cpp::hasRealtimeKernel()` 增加 `uname`/`PREEMPT_RT` fallback，恢复 `RealtimeConfig::kEnforce`，避免实时调度失败时带病启动。

6. 打开可观测性

当前配置关闭了 overrun 诊断：

```yaml
overruns:
  manage: false
  print_warnings: false
```

调试期间建议改成 `true`，并持续记录：

```text
/left_arm/franka_robot_state_broadcaster/robot_state
/right_arm/franka_robot_state_broadcaster/robot_state
```

重点字段是 `control_command_success_rate`，它表示最近 100 个控制命令成功到达机器人的比例。[RobotState 字段说明](https://frankarobotics.github.io/libfranka/latest/structfranka_1_1RobotState.html)

## 如果调优后仍然只坏左臂

按这个顺序做 A/B：

1. 安全条件满足且 ROS 停止后，分别运行官方 `communication_test`；该测试会移动机器人。
2. 交换左右网线或交换机端口，看故障跟随线缆/端口还是仍留在左机器人。
3. 最终建议每台 Franka 使用独立 NIC、独立子网，直接连接 Control。
4. 如果换线、换端口、独立 NIC 后仍固定发生在左臂，再检查左 Control 固件和硬件日志。

另有一个恢复层代码风险：自定义 `franka_hardware_interface.cpp` 在 read/write 异常后过早调用 `resetControlState()`，先清除了 `claimed_`/`running_`，导致随后 controller_manager 报 `Not acceptable command interfaces combination`。它不是通信错误的起因，但会让通信错误后的自动恢复更加混乱，应在基础实时问题修好后一起调整。

本次只做了只读 SSH 检查，没有修改配置、重启服务或移动机器人。