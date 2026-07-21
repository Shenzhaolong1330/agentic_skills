# 安全不变量

以下不变量在 S0/S1 冻结，后续 Planner、dispatcher、graph executor 和 skill 扩展均不得削弱：

1. 默认不访问硬件。
2. `hardware_allowed` 默认是 `false`。
3. `mock`、`dry_run`、`from_artifacts` 永不访问真实硬件。
4. 有物理副作用的 live 操作同时要求 `hardware_allowed` 和 `execute`。
5. 只读 live 硬件操作要求 `hardware_allowed`，但不强制 `execute`。
6. recovery entrypoint 必须在 manifest 中声明 `allowed_as_recovery=true`。
7. E-stop 或 unsafe 状态不自动解除；Agent 只能进行允许的只读诊断或返回人工处理要求。
8. reset 可能移动双臂、打开夹爪或改变持物状态；它不是无副作用的错误清除。
9. 未授权必须在硬件初始化、后台服务启动和子进程调用之前失败。
10. 硬件相关行为必须写入 trace，或至少保留明确的 gate decision。
11. 测试不得连接真实设备、启动 ROS、打开相机、执行 reset 或运动机械臂。
12. 仓库不存在第二种秘密或 credential 授权机制。
13. 低层内部脚本不得作为公开的未授权 live 入口；底层 OS 级调用仍需由部署环境自行保护。
14. 任何未来 Planner 都不能改变这些安全不变量。

E-stop 后只能进行允许的只读诊断，或明确返回需要人工处理。Agent 不自动解除 E-stop；`reset`、`home` 和控制器故障恢复不能被描述为 E-stop recovery。

