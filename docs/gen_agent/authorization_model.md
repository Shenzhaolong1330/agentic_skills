# 硬件授权模型

## 唯一人工授权位

`hardware_allowed` 是唯一由用户明确授予真实硬件访问的布尔授权位，默认值为 `false`。它只能来自明确的 `--hardware-allowed` CLI 参数或显式构造的 `SkillContext`；不会从环境变量、主机名、用户名、路径、网络地址或运行模式推断，也不会因为 `live` 或 `execute` 自动变为 `true`。

仓库不使用第二种秘密、口令或 credential gate。不存在基于字符串匹配的硬件放行机制。

## 三个上下文字段

| 字段 | 职责 | 默认值 | 说明 |
| --- | --- | --- | --- |
| `mode` | 选择 `mock`、`dry_run`、`from_artifacts` 或 `live` | `mock` | 非 live 硬件能力只生成计划 |
| `hardware_allowed` | 允许访问真实相机、RPC 或其他硬件 | `false` | 唯一人工硬件授权 |
| `execute` | 允许具有物理副作用的动作 | `false` | 不是身份认证，不能替代硬件授权 |

## 统一判定表

| mode | 硬件能力 | `hardware_allowed` | 物理副作用 | `execute` | recovery manifest | 结果 |
| --- | --- | ---: | ---: | ---: | ---: | --- |
| `mock` / `dry_run` / `from_artifacts` | 任意 | 任意 | 任意 | 任意 | 任意 | `planned_only=true`，不访问硬件 |
| `live` | 否 | 任意 | 否 | 任意 | 不适用 | 允许普通非硬件逻辑 |
| `live` | 是 | 否 | 任意 | 任意 | 任意 | 拒绝：`hardware_allowed_required` |
| `live` | 是 | 是 | 否 | 否 | 不适用 | 允许只读硬件访问 |
| `live` | 是 | 是 | 是 | 否 | 不适用 | 拒绝：`execute_required_for_side_effects` |
| `live` | 是 | 是 | 是 | 是 | 不适用 | 允许 |
| `live` recovery | 是 | 是 | 按上行规则 | 按上行规则 | 否 | 拒绝：`recovery_not_allowed` |
| `live` recovery | 是 | 是 | 按上行规则 | 按上行规则 | 是 | 按上行规则继续判定 |

只读能力包括相机观测、机器人状态、gripper status 和只读 RPC health check。移动、夹爪控制、reset/home、清除控制器故障、恢复控制器、改变持物状态或 manifest 明确标记的其他物理状态改变，均属于有物理副作用能力。

所有判定由 `evaluate_hardware_authorization` 提供，`HardwareGate.evaluate` 只是该函数的共享入口。错误原因使用稳定标识：`non_live_mode_planned_only`、`hardware_allowed_required`、`execute_required_for_side_effects`、`recovery_not_allowed`、`hardware_gate_passed`。

