# 2026-09-18 交接后头部相机 busy

故障运行：`/tmp/agentic_skills_runs/demo_1_vial_insert_extract_20260918_132108_853200`。
抓取交接已成功，失败发生于 `rack_before_insert_1`，尚未执行插入。
相机为 head / SDK serial 348522072761；该错误不是机器人 RPC 心跳错误。

实际检查：

- 未发现进程占用当前有效的视频节点；adb 仅持有已经删除的旧视频节点，不能据此判断它占用了当前相机。
- 同时段内核记录 USB 端口 2-9.4.3.2 的 -71 视频传输错误。
- 单独采图复现无帧；仅复位该相机后采集恢复。
- 再次开流复现 `get_xu(ctrl=1) ... Device or resource busy`，完整栈指向开流后的 `depth_sensor.set_option(visual_preset)`。

修改：

1. `RealSenseCamera` 先根据配置解析指定设备并设置相同深度预设，再启动流；不改变串号、预设值或标定。
2. Demo 单次感知通过 `demo_camera_locate.py` 复用旧插入技能的采图 watchdog，覆盖开流前的特定 UVC busy/timeout/protocol error 和无帧超时。
3. 故障时结束原采集进程，再做有限次数的相机复位恢复。不会重做抓取交接，也不重放运动。
4. 新 demo 的采图窗口默认 8 秒，包含 CLI 冷启动、配置和预热；原 3 秒边界在实际 CLI 验证中触发过不必要的恢复。VLM/SAM 推理不受该采图窗口限制。
5. API 401/402、目标未找到、配置错误不触发相机恢复。

验证：

- 修改后连续三次 RGB-D 采集成功，仅第一次做了相机复位。
- 经用户明确授权，将当前桌面图像发送至已配置 OpenRouter，红架定位返回 found=true，深度/基座坐标可用，结果图已人工检查。
- 结果：`/tmp/agentic_skills_runs/demo_1_vial_insert_extract_20260918_132108_853200/rack_recovery_check/result.json`。
- 图像：同目录 `panel.jpg`；该完整识别测试经历一次相机自动恢复，成功返回，未进行机器人动作。
- 相机启动、CLI、感知恢复、demo 编排和插入回归测试通过。

相机恢复不代表 USB 硬件已完全可靠；若 -71 持续出现，应检查该相机的 USB 连接和供电。
本次只验证到感知恢复，没有执行第一支插入。检查时右夹爪保持闭合并反馈 object_detected=true，左夹爪打开；不能从头重跑包含开爪的抓取流程。

## 14:33 Demo 4：复位耗尽，USB 端口不可恢复

运行 `/tmp/agentic_skills_runs/demo_4_vial_extract_20260918_143353_306477` 在首次
`rack_before_extract_1` 失败，尚未运动。完整 stderr 证明 8 秒采图超时后已经执行
3 次 SDK 相机复位，依次遇到 UVC 超时和 Protocol error，不是恢复机制未触发。

主机内核 14:33:59 起记录 head 端口 `2-9.4.3.2` 的 `SET_CUR -110/-71`；
相邻 Hub 端口 `.1` 也持续出现 suspend -71。未发现有效视频节点被进程占用。
核对 USB descriptor serial `253443066095`（SDK serial `348522072761`）后，
只对这只设备 `/dev/bus/usb/002/066` 执行了一次 `USBDEVFS_RESET`，返回 ENODEV。
14:36:04 内核明确报告 `2-9.4.3-port2 cannot reset / Cannot enable` 及
`hub_ext_port_status failed (err = -71)`。

已请求现场重连头部相机，尽量绕开多层 Hub 直连主机 USB 3。
这些证据指向 USB 链路故障，尚不能独立确定是线缆、Hub、供电还是相机硬件。
未重置父 Hub、未操作机器人。终端摘要新增“已复位 3 次仍未恢复”，避免隐藏恢复耗尽信息。
