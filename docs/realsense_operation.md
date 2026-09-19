# RealSense 操作规范

适用于本仓库的 object-locator、vial inventory 和 head/wrist 常驻采集服务。
正常运行不需要先 reset 相机，也不应每次失败都立即重复启动整个机械臂任务。

## 当前问题的证据

2026-09-18 的日志有两类问题。软件侧曾在开流后写深度预设、缺少同串号互斥、
启动重试和任务复位叠加，且入口的采图超时不一致。上述问题已在代码中整理。

硬件侧，头部 D435i 经 `2-9 → 2-9.4 → 2-9.4.3` 多层 USB Hub 连接，内核实际出现
`SET_CUR -110/-71`、`hub_ext_port_status failed`、端口 `cannot reset / Cannot enable`。
三次 SDK 复位及一次仅针对该设备的 USB reset 均失败。不能仅凭这些日志确定
是哪根线、哪一级 Hub、供电还是相机硬件损坏，也不能用修改 Python 参数证明已修复。
详见 [事故记录](franka_camera_busy_20260918.md)。

多相机布线需考虑供电、USB 带宽及线缆长度，依据
[RealSense 官方多相机说明](https://realsenseai.com/how-to-multiple-camera-setup-with-ros/)。
排查这只头部相机时，先将它绕过当前多层 Hub 直接接主机 USB 3，保持同一配置测试，
再逐项对照线缆与 Hub；不要一边变更拓扑一边重跑机器人流程。
本次没有修改系统 USB autosuspend、驱动、固件或父 Hub 状态。

## 统一生命周期

1. 以配置中的 **SDK serial_number** 选择设备。多相机环境不能省略串号，不能回退到任意设备。
2. 按串号取得跨进程排他锁，然后设置流格式。深度预设只在开流前、且当前值不同时写入。
3. 开流、预热、采集同一组对齐 RGB-D，复制帧数据。
4. 停流并释放 SDK 引用与排他锁，之后才发出 capture-ready，才开始 VLM/SAM 推理。
5. 异常退出也释放资源。任务终止子进程时先发 SIGTERM，给上下文清理机会；
   SDK 本地调用若卡死仍由父进程在等待后 kill，不把未完成采集当成成功。

`/tmp/agentic_skills_realsense_<uid>/` 存储锁与复位时间，锁由内核随进程退出释放，
不用手动删除所谓“锁文件”。删除正在使用的文件反而会绕过互斥。
此锁覆盖采用本仓库 RealSenseCamera 的同一用户进程，不会约束 realsense-viewer、
另一个用户或外部 ROS 相机节点；使用它们前先停止对应仓库采集服务，反之亦然。

单次采图 CLI 和常驻相机服务是两种所有权方式。常驻服务开启时，消费者应从其
缓存/接口取图；不能同时用 CLI 再打开同一串号。不同串号可以分别持有锁。

## 恢复规则

- 缺帧可在采集调用自身规定的次数内等待；UVC/设备异常不当作普通丢帧反复等待。
- 任务层采集失败：结束并等待原采集进程 → 等 2 秒 → 同串号干净重开一次 →
  已知采集错误仍存在时，默认只尝试一次 SDK hardware_reset。
- 复位前必须持有设备锁；同一串号两次复位至少相隔 60 秒，即使换一个 CLI 进程也有效。
  复位后至少等待配置的 reset_wait_s（默认 5 秒）并核对同串号重现。
- 占用冲突（含 SDK 的 Device or resource busy）、复位冷却期、配置错误、清理失败、VLM/HTTP 错误不触发相机复位。
  复位失败后停止任务；常驻服务停止循环复位，等待设备重新出现或修复连接后重启服务。
- 不自动复位父 Hub，不重启机器人，不重发任何结果未确认的运动。

默认采集窗口统一为 30 秒，恢复采集窗口也是 30 秒；它们不限制 capture-ready
之后的 VLM/SAM 推理。原先的 3 秒/8 秒默认值不再混用。
`REALSENSE_CAPTURE_WATCHDOG_SEC`、`REALSENSE_RESET_CAPTURE_TIMEOUT_SEC`、
`REALSENSE_RESET_MAX_ATTEMPTS` 保留显式覆盖；增加次数不会绕过 60 秒复位冷却。
`tube_insertion_skill.py --rack-capture-ready-timeout-sec` 默认同为 30 秒。
正常配置 `reset_on_start: false`，不要把 `--reset-realsense` 加入每次例行采图命令。

## 检查工具

只读取 USB 拓扑、内核日志和本仓库会话记录，不打开相机：

```bash
python3 atomic_skills/object_locator/scripts/realsense_check.py
```

重连后，在没有其他采集者占用时，连续做三次打开/采图/释放验证：

```bash
timeout 90s atomic_skills/object_locator/.venv/bin/python \
  atomic_skills/object_locator/scripts/realsense_check.py \
  --capture --cycles 3 \
  --config atomic_skills/object_locator/config_rack_center_generic_vlm.yaml
```

此命令只保存本地 RGB 和深度有效像素统计，不调用 OpenRouter、不控制机器人、
不做任何 reset。输出在 `/tmp/agentic_skills_runs/realsense_check_*/`，
失败时查看 report.json 和内核日志；不要拿旧图片代替本次采图成功。
若命令被外层超时强制终止，可能没有最终报告，不应将其解释为成功。
腕相机改用下表中对应的配置文件。

| 相机 | SDK 串号 | 配置 |
|---|---|---|
| head | 348522072761 | config_rack_center_generic_vlm.yaml |
| left | 347622074336 | config_rack_empty_hole_left_wrist_generic_vlm.yaml |
| right | 337322072568 | config_rack_empty_hole_right_wrist_generic_vlm.yaml |

头部相机 USB 描述符串号为 253443066095，与 SDK 串号不同；不要把它填进 SDK 配置。
修改串号、分辨率或相机物理安装位置时，必须核对对应标定，不能靠换相机规避错误。

## 本次验证范围

自动测试覆盖同串号跨进程竞争、进程被 kill 后锁释放、跨进程复位冷却、
重复预设不写、初始化失败清理、恢复次数边界、服务释放后退避、忙碌时不复位。
这不等同于硬件稳定性验证。14:33 的头部相机 USB 故障仍需现场重连后实际采图确认。
