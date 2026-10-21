# 本地原子 skill 调用指南

原子目录：`/home/deepcybo/agentic_skills/atomic_skills`。下面是建立候选映射的导航，调用前读取对应 `SKILL.md`；其接口与实时代码优先于本导航。复用现有脚本，不复制底层驱动或改写其安全限制。

| 能力 | 必须读取的入口 |
|---|---|
| 拍照、识别与三维定位 | [object_locator](../../atomic_skills/object_locator/skills/atomic-perception-locate-object-3d/SKILL.md) |
| 一臂或双臂 P2P 位姿运动 | [dual_franka_p2p](../../atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/SKILL.md) |
| 左右夹爪状态、打开和关闭 | [dual_franka_gripper](../../atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/SKILL.md) |
| 机器人状态与头部/腕部 RGB-D | [state_realsense_viewer](../../atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/SKILL.md) |
| 当前故障检查与条件复位 | [dual_franka_reset](../../atomic_skills/dual_franka_reset/skills/atomic-state-dual-franka-reset/SKILL.md) |

## 拍照与定位

用户指定 `object_locator` 时优先使用该项目；仅拍照可使用其现有诊断采图脚本，不必启动检测模型。以下命令拍右腕一张新图，不运动、不复位：

```bash
cd /home/deepcybo/agentic_skills
atomic_skills/object_locator/.venv/bin/python \
  atomic_skills/object_locator/scripts/realsense_check.py \
  --capture --cycles 1 \
  --config atomic_skills/object_locator/config_rack_empty_hole_right_wrist_generic_vlm.yaml
```

从实际输出的 `artifacts` 目录取 `rgb_1.jpg` 并检查本次采集是否成功。不要硬编码历史输出目录。采集失败时按相机 skill 诊断，不能自动换成另一台相机或默认硬件 reset。

建立时对应 SDK 串号为 head `348522072761`、left_wrist `347622074336`、right_wrist `337322072568`；实际调用前核对配置，不能用 USB 描述符串号替换 SDK 串号。定位用于机器人运动时，必须检查 `position_base.available` 和对应相机的标定/实时腕姿。

## 运动

入口为 `dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/scripts/move_to_pose.py`，接收绝对 `[x,y,z,rx,ry,rz]`，单位米与弧度，真实发送用 `--execute`。

读取最新当前位姿并固定目标；不要使用 skill 文档中的示例坐标作为现场目标。只传一个臂的 pose 时，另一臂由执行器保持当前位姿。先验证实际 frame、工作空间、路径和速度/步长，再调用。`--max-translation-speed` 是上限，不是平均速度；“快速”应在已验证的控制参数范围内选值。

当前脚本可能退出码为 0 而 `result.ok=false`。检查 `result.final_error`、`left_final`、`right_final` 及实时故障信息，分别记录到位情况。它已有内置平滑轨迹与有限反馈修正；不要在外层重新下发原相对位移补偿。

## 夹爪

读取状态：

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py status --side right
```

已授权开合时，将 `status` 分别改成 `open` 和 `close` 并加 `--execute`，按顺序各调用一次。侧别使用实际请求的 `left`、`right` 或 `both`。该脚本的命令回复本身不保证机械动作到位；须继续只读检查新鲜驱动状态，并有界等待。

核对 `driver_status` 的 `sample_valid`、`sample_fresh`、`registers.fault_byte`、请求目标与实测位置，以及 `requested_position_reached`。不同环境的开闭端点编码可能不同，不硬编码“闭合必为 255”。闭合因接触物体停止与完全闭合要区分；`object_detected=true` 不自动证明物体已稳固抓住。反馈缺失时报告未确认，不伪报成功或反复开合。

## 状态与复位

正常观察优先只读接口。reset skill 的 `status` 用于分类，不能因为日志缺字段就宣称健康。其 `ensure --execute` 可能使双臂回 home 并打开夹爪，不能用来解决普通位置误差，也不能当成运动前的默认步骤。只有明确故障且授权覆盖全部副作用时才考虑执行。
