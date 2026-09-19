---
name: task-configurable-manipulation-demos
description: Configure the numbered Franka demos for four-vial insertion/extraction, extraction-only with selectable jaw orientation and corner order, elongated-object handover, and the copied original all-vial flow. Use for these demo scripts and configs, not dataset processing or general robot reset.
---

# 编号演示：可配置抓取、交接、插拔

入口和配置按 `demo_序号_任务内容` 命名，所在目录仍保留 `task_skills/demo_NO/`。
默认只规划；实现、审查、测试时只使用
`--dry-run` 或 `--mock`，不连接相机/RPC。真机入口沿用原 shell wrapper 的
`--mode live --execute` 方式；首次运动前默认等待 Enter，不使用 harness operator token。

## 入口

从仓库根目录运行：

```bash
bash task_skills/demo_NO/skills/task-configurable-manipulation-demos/scripts/demo_1_vial_insert_extract.sh --dry-run
bash task_skills/demo_NO/skills/task-configurable-manipulation-demos/scripts/demo_2_elongated_handover.sh --dry-run
bash task_skills/demo_NO/skills/task-configurable-manipulation-demos/scripts/demo_3_all_vials_to_rack.sh --dry-run
bash task_skills/demo_NO/skills/task-configurable-manipulation-demos/scripts/demo_4_vial_extract.sh --dry-run
```

真机时将 `--dry-run` 换成 `--mode live --execute`。保留原 Demo 的 Franka/ROS/RPC/
RealSense/SAM/OpenRouter 环境；启动方法见
[已有操作说明](../../../../docs/pick_tube_insert_rack_demo.md)。这里不自动启动 ROS 服务。
公共 smooth P2P 在每次调用的首段前将服务端目标同步到当时双臂实测位姿，
避免 home、人工移臂、中断后重新发布旧目标；同步失败不发轨迹。
段内纠偏从已确认发送完成的命令终点计算，不反复归零；对实际位置/姿态残差作有界补偿：
增益 0.6，单轮最多 3 mm / 0.03 rad，相对原请求目标累计最多 25 mm / 0.20 rad。
接近上限时分别缩小平移/旋转纠偏步长，用完该方向的剩余空间；
残差未达标且无可用纠偏空间时失败停止，不宣称成功、不自动开爪。到位仍按原目标与原容差判断。
子进程输出实时保存，Ctrl-C 终止该子进程组，但**不代表机械臂已停止**，
服务端最后目标可能仍有效。异常突动时使用现场急停，不自动 reset、恢复控制器或重发动作。
2026-09-18 15:51 事件证据和离线验证见
[公共 P2P 目标同步排查](../../../../docs/franka_p2p_target_sync_20260918.md)。
调试 Demo 1 的相机/识别时，可加 `--stop-after-inventory`：实际采集并校验四支 vial 后退出，
不会等待运动确认或调用机器人。结果的 `inventory_complete` 表示识别校验完成，
`completion_flag` 仍为 false，因为完整插拔任务没有执行。

Demo 1/2 的单次定位通过 `demo_camera_locate.py` 复用旧插入技能的 RGB-D
采集 watchdog。等待采图超时，或采图前发生已知 UVC `get_xu/set_xu/xioctl`
timeout、protocol error 时，先结束原采集进程，再按同一相机配置进行
先干净重开一次，仍失败时默认最多 1 次相机硬件复位；不重试已完成的抓取、交接或插入。
采图完成后，VLM/SAM 使用正常推理超时；HTTP 401/402、未识别到目标、
配置错误不触发相机恢复。持续 USB 故障仍会停止，不能用重试证明硬件可靠。
各定位入口统一给采图阶段 30 秒（含 CLI 启动、相机配置和预热），不再混用 3/8 秒
阈值；显式 `REALSENSE_CAPTURE_WATCHDOG_SEC` 仍优先。
RealSense 的深度预设在按指定序列号解析设备后、开流前设置，避免开流后的
`set_option(visual_preset)` 触发此次观察到的 UVC busy；预设值、串号和标定不变。
2026-09-18 13:21 运行在 `rack_before_insert_1` 遇到 head 相机 `get_xu ... busy`；
内核同时段出现 USB -71，现场未发现有效视频节点被其他进程占用。
仅复位该相机后 RGB-D 采集恢复，未重做交接或改变持管状态。

前两个入口分别默认读取 [demo_1_vial_insert_extract.yaml](config/demo_1_vial_insert_extract.yaml) 和
[demo_2_elongated_handover.yaml](config/demo_2_elongated_handover.yaml)。可使用 `--config /绝对路径/自定义.yaml` 替换，
`--artifact-dir /tmp/新目录` 指定记录位置。真机目录必须为空；
`--no-wait-before-motion` 用于明确需要无人值守的调用。

前两个 demo 的 `--mock` 使用合成定位结果走完整编排，记录但不执行任何子进程；不能用作物理成功证据。
`--dry-run` 生成任务顺序和每个孔位的感知配置，不生成假坐标运动计划。

## Demo 1

默认流程：一次识别四支平放 vial，按初始头部图像从左到右编号，依次抓取、交接、
重新识别红架、用接收手腕部相机寻找指定空孔、插入、释放、回撤回 home。
交接时确认接收夹爪稳定并完成等待后，原手松开，等待 0.5 秒，**仅原手水平向外侧平移 20 cm**。
左臂沿自身 base +Y（左侧），右臂沿 base -Y（右侧）；保持当前 X、Z、姿态，接收手目标保持当前位置。
`handover_retreat_distance_m: 0.20` 控制距离；避让平移速度不超过 0.04 m/s、步长不超过 0.001 m。
全程使用笛卡尔 P2P，不调用交接后的 joint go home、不切换控制器。
避让成功后持管手直接进入观察/插入，插入前两手都不回 home；观察步骤不重复避让。
避让失败则停止，不自动改用 go home。下一轮从避让姿态开始，仍严格检查抓取到位误差。
原因及日志证据见 [交接后突动排查](../../../../docs/franka_post_handover_retreat_20260918.md)。
插入完成并释放后的回撤/home 保留。共用 home helper 的目标同步使用 `step(None)`，
不调用会重新激活夹爪的 `reset()`。拔出观察同样不自动回位另一只手。
插入结果通过本次运行 `insert_N/result.json` 读取，不把含误差提示的 stdout 当作纯 JSON。
结果文件缺失、损坏或未明确报告完成时停止；动作可能已经执行，不自动重跑插入。
**四支全部插完后**，按 `cycles` 顺序拔出指定位置；第 i 次拔出由第 i 次插入的手完成。
拔出前观察/转姿显式使用 `extract_observe_max_correction_iters: 10` 次纠偏，
保持 3 mm / 0.03 rad 到位容差。2026-09-18 首次拔出曾因漏传该参数，只纠偏 1 次后
仍有 12.03 mm / 0.0534 rad 残差而停止；当时还未抓管。
观察失败时终端显示 JSON 报告的 `stopped_reason` 和最终残差，缓存识别提示不作为错误原因。
拔出时夹盖/露出的上端，竖直提升 `extract_lift_m`，再移到侧边空桌面、下降到释放高度、松爪。
不做第二次 handover，不尝试让物品直立或稳定落座。

```yaml
grasp_position: 1  # 1 尾端1/5；2 前端1/5
pick_arm: auto     # 也可固定 left / right
extract_timing: after_all_inserts
cycles:
  - {insert: [1, 1], extract: 左上}
  - {insert: [3, 1], extract: 左下}
  - {insert: [1, 2], extract: 右上}
  - {insert: [3, 2], extract: 右下}
```

孔位为 **3 行 × 2 列**，均从 1 编号，配置次序是 `[行, 列]`：

```text
             第1列       第2列
第1行（上）  [1,1] 左上   [1,2] 右上
第2行        [2,1]        [2,2]
第3行（下）  [3,1] 左下   [3,2] 右下
```

`extract` 也可直接写 `[行,列]`，支持中间孔。`cycles` 每项可加 `grasp_position: 2`
覆盖全局抓取选择。`initial_occupied: []` 默认空架；预放的 vial 必须在这里列出孔位。
程序在首次感知之前检查顺序占用关系，拒绝插入已占用孔、拔出空孔或越界孔。
也支持 `extract_timing: after_each_insert`，但本需求默认四支插完再拔。

孔位编号是逻辑方向，不应直接假设两只腕相机的画面方向相同。
`wrist_view.left/right.row1_at` 和 `col1_at` 指定各腕图像中逻辑第 1 行/列所在的边，
可为 `top/bottom/left/right`；行列轴必须垂直。默认是图像上方第 1 行、左侧第 1 列，
这是示例映射，需根据现场观察位图像核对。转过 90° 的架子也可通过这两个值映射。
架子摆放或 wrist 姿态改变后重新核对。程序生成只允许选指定位置的 VLM 提示，
目标被遮挡、占用状态不符时要求 `found=false`，不回退到其他孔。
当前依赖 VLM 的行列识别，尚无独立几何孔位身份验证；保存腕部识别图供检查。
不使用旧的固定 3×4 `cached-grid`。

## Demo 2

每轮重新识别一件平放长条物品，按指定端点位置抓取，再执行旧交接流程。
`count` 是本次确切处理件数；不足、定位失败或头尾坐标缺失时停止并报告失败。
`object_description` 描述物品及取料区，`front_definition` 定义前端。
默认优先采用明显的功能端/标记端；对称物体以头部图像 x 较小的端为前端。
此配置关闭试管黑盖 SAM 端点重估，直接使用 VLM 端点和已有深度/标定流程。

默认 `after_handover: drop`：接收手把物品放到侧边桌面上方释放，再处理下一件。
交接后仅松开的原手向外侧平移 20 cm（同 Demo 1）；接收手保持夹持，继续执行配置的 drop 或 hold。
只想展示交接并保持夹持，设置 `after_handover: hold` 和 `count: 1`。
取料区与落料区应在 `object_description` / `drop_description` 中清楚区分，避免重新抓取落下的物品。

## 抓取、交接和落料参数

- `handover_receiver_stable_sec: 0.5`：接收夹爪达到原闭合阈值后，要求带更新 `stamp`
  的位置反馈持续稳定（闭合比例变化范围 ≤0.01），避免夹爪仍在闭合时就放开原手。
  `handover_close_timeout_sec: 5.0` 为单次闭合确认超时；失败保留原手夹持。
  确认后还等待 `handover_release_delay_sec: 2.0` 秒再释放原手，两种交接方向通用。
  这些是夹爪运动完成的判断，不能证明物品已夹住；交接位置仍需正确标定。
  旧抓取 helper 同步采用稳定确认和 2 秒等待默认值，因此 Demo 3 也受益。
- Demo 1/2 的 `p2p` 控制散物抓取、转姿、提升及交接动作。默认平移上限
  `max_translation_speed: 0.12` m/s、旋转上限 `max_rotation_speed: 0.6` rad/s，
  与旧 `grasp_right_arm_xyz.sh` 的速度一致；新 demo 初版使用的 Python 默认值分别为 0.04 和 0.2。
  `rate_hz: 80`、平移步长上限 0.003 m、旋转步长上限 0.03 rad。
  `approach_max_translation_speed: 0.08` / `approach_max_rotation_speed: 0.4`
  单独控制夹取前的下探。可在两个 YAML 中修改这些值，dry-run 会输出实际生效值。
  姿态/位置到达容差、插孔下降速度、拔出和落料参数不随此项调整。
- `grasp_position: 1` 使用 `tail + 0.2*(head-tail)`；`2` 使用 `tail + 0.8*(head-tail)`。
  vial 的 head 是盖子/口端，tail 是相反封闭端。需要有效 base 头尾坐标，不允许退回 bbox 中心。
- `pick_arm: auto` 复用原来 base Y 的选臂约定（右侧为负 Y），前端抓取时按前端选臂；
  在 2 mm 死区内拒绝自动选臂。固定 `left/right` 可让全部抓取用同一只手。
- `transition_json: null` 复用原标定，非空路径相对 YAML 文件目录；
  `handover_transition_branch: active` 默认选抓取手分支，`partner` 选另一分支。
  改变抓取端或长条物品尺寸不会自动重标定交接姿态；需核对接收夹持位置与插入朝向。
  本实现保持原 tail→head 朝向向量，不擅自把工具 yaw 翻转 180°。
- `grasp_z_offset_m` 为 TCP 抓点 Z 偏移，`lift_m` 为散物抓起提升量，
  `extract_lift_m` 应足以使 vial 底部完全高于架体后再横移。
- `insert_depth_m` / `hole_standoff_m` 控制旧插入流程，保留原姿态校正、分段下降和力检查。
- `drop_xyz.left/right: null` 每次用头部相机选侧边空桌面；释放 TCP 高度为桌面 Z
  加 `drop_height_m`。若填写 `[x,y,z]`，这是对应手 base 坐标中的 **TCP 释放位置**，
  不再叠加高度。物品会自然下落，需按物品露出 TCP 的长度选择释放高度。

## Demo 3：旧版全部 vial 入架程序副本

[demo_3_all_vials_to_rack.sh](scripts/demo_3_all_vials_to_rack.sh) 是旧
`run_full_pick_tube_insert_rack.sh` 的完整编排副本，放在前两个 demo 旁边。
它的资源路径指回原技能目录，继续共享原来的定位、SAM/相机缓存、抓取、交接和插入代码。
支持原来的参数，包括 `--stop-after-inventory`、`--tube-config`、`--rack-config`、
`--grasp-arg`、`--insertion-arg`、`--no-reset-after-error`。

此 demo 按初始图像从左到右处理所有识别到的散落 vial，插入当前空孔，**不拔出**。
保留原程序失败后 reset 并处理下一支的策略，与 Demo 1/2 的失败即停止策略不同。
`--mock` / `--dry-run` 调用原安全 harness，不连接硬件；真机主流程保持原样。

默认配置为 [demo_3_all_vials_to_rack.env](config/demo_3_all_vials_to_rack.env)，
使用原程序的 Bash 环境变量格式。文件会被 `source`，只使用可信任配置。
通过 `DEMO_CONFIG=/绝对路径/自定义.env bash .../demo_3_all_vials_to_rack.sh ...`
替换配置；原 CLI 参数优先于配置。前两个 demo 仍使用 YAML。

## Demo 4：仅拔出四支 vial

入口 [demo_4_vial_extract.sh](scripts/demo_4_vial_extract.sh)，配置
[demo_4_vial_extract.yaml](config/demo_4_vial_extract.yaml)。红架四角已经各有一支 vial，
所选手应为空手；不需要把它们重新放回桌面，也不运行 inventory、handover 或插入。

```yaml
extract_arm: right
gripper_opening_direction: 1  # 1 垂直于 base X；2 平行于 base X
extract_order: [右下, 左上, 右上, 左下]
```

`extract_order` 必须恰好包含左上、左下、右上、右下各一次。四次均使用
`extract_arm`（left/right）。X 指所选机械臂的 base X，不是图像 X 或红架长轴。
`gripper_opening_direction` 控制两指开合的连线：1 沿 base Y，2 沿 base X；
两种选择都保持 TCP Z 朝下。`jaw_opening_axis: y` 是 TCP 局部开合轴的标定定义，
不应为了切换朝向而修改它。

复用原拔出流程：首次识别并检查红架 → 空手转到指定姿态和腕部观察位 → 用本次实际腕部
标定定位指定角落 vial 的盖子 → 保持朝向抓住盖子 → 竖直提升 → 水平移到右侧空桌面上方，下降并释放。
四次观察共用本轮首次红架参考；期间架子须保持不动。每支管盖仍用当前腕图和实际位姿重新定位。
首次红架 Z 必须在 `rack_z_range_m` 内（当前桌面为 [-0.25,-0.10] m），管盖也必须在
参考架子附近（XY 半径 `extract_cap_max_xy_m`，相对高度 `extract_cap_z_range_m`）。
校验失败在对应观察/开爪/下探动作之前停止，不把异常坐标截断成某个可执行值。
2026-09-18 15:29 的运行中，第二次头图被右夹爪遮挡，红架 Z 从 -0.2024 m 跳到 +0.0893 m，
观察目标因此上升约 23 cm；现在不再用夹爪挡住时的新头图覆盖本轮参考。
`extract_observe_height_m: 0.12` 是红架参考到观察 TCP 的高度；
`extract_lift_m: 0.12` 是夹紧后正常竖直拔出的距离，两者不是这次异常上升。

右侧按原工作区约定定义为 **base -Y**，与选哪只手无关。每次重新选择右侧裸露桌面，
并检查相对红架的右移距离 `drop_lateral_range_m: [0.12,0.30]`、前后距离
`drop_max_fore_aft_m: 0.15` 以及桌面相对高度 [-0.10,+0.02] m。
显式 `drop_xyz.left/right` 同样接受检查；它是松爪 TCP 点，桌面 Z 按减去 `drop_height_m` 校验。
没有合适右侧区域就停下，禁止退回前方或左侧。保持拔管姿态，横移阶段保持 Z，
下降到桌面上方 `drop_height_m` 后松爪，不再额外向上回撤 10 cm。
若释放点要求 TCP 上升超过 5 mm，放置 helper 在任何放置运动前停止，不松爪。
这些位置门限适用于当前桌面/标定，改变安装或架子位置后应核对配置，不能为通过错误识别而放宽门限。
不调用 joint home，不自动 reset。Demo 4 观察最多采用 20 次纠偏，位置容差由
`extract_observe_position_tolerance_m: 0.005` 设为 5 mm，姿态容差由
`extract_observe_rotation_tolerance_rad: 0.06` 设为约 3.44 度。仅观察/转姿阶段使用该容差。
2026-09-18 平行 X 的观察转姿在第 10 次纠偏后残差为 4.2064 mm / 0.0219 rad，
故将 Demo 4 的观察位置容差单独调整；后续使用实际腕部标定重新定位管盖。
按用户要求区分关键动作与后续移动，Demo 4 下探夹盖使用
`extract_position_tolerance_m: 0.003` / `extract_rotation_tolerance_rad: 0.03`。
抓紧后的拔出提升单独使用 `extract_lift_position_tolerance_m: 0.005` /
`extract_lift_rotation_tolerance_rad: 0.06`，右侧落料使用
`drop_position_tolerance_m: 0.010` / `drop_rotation_tolerance_rad: 0.10`。
未配置这些参数的旧入口仍使用原 helper 默认值。
到管盖上方的 stage1 横移单独配置 `extract_pregrasp_xy_position_tolerance_m: 0.005`、
`extract_pregrasp_xy_rotation_tolerance_rad: 0.06` 和 `extract_pregrasp_xy_settle_time_sec: 0.3`。
每段轨迹/纠偏后的固定等待分别为观察 `extract_observe_settle_time_sec: 0.5`、
下探 `extract_approach_settle_time_sec: 0.6`、提升 `extract_lift_settle_time_sec: 0.4`、
落料 `drop_settle_time_sec: 0.3`。旧配置缺少这些字段时仍使用原默认值。
夹爪物理闭合反馈、闭合后的等待和各阶段到位检查保留。固定等待缩短后仍需观察真实收敛情况。
保持当前姿态时不再执行重复的 stage2 XY 运动，
下探目标仍使用识别的管盖 XYZ，并按上述配置检查。公共抓取 helper 在补偿耗尽后
不再整段重试或恢复控制器，错误直接包含最终位置与姿态残差。
落料选点先检查图像框完全处于固定红架参考框右侧（至少 5 px），再检查 base -Y
距离、前后偏移和桌面高度；两张头部图像必须同尺寸，头部相机/红架在批次内保持固定。
`drop_perception_retries: 1` 允许不合格的落料候选重新采图识别一次，提示中带入参考框
和拒绝原因，每次分别保存记录。只在候选通过全部检查后发送一次落料命令；
运动失败不重跑、不会复用先前已投放 vial 的落料点。未通过限次重识别或其他阶段失败则停止。
2026-09-19 13:29 记录中第二次落料把红架识别成空桌面，不能靠放宽移动容差解决；
详情见 [落料误识别修复](../../../../docs/vial_drop_misidentification_20260919.md)。
固定红架在整批只识别一次，第 2–4 支终端显示 `reuse_rack_reference_N`，不执行红架识别。
每支的管盖定位和右侧空桌面检查仍执行，因为机械臂位置、剩余 vial 和已落料 vial 会变化。
第一次红架识别后沿用 Enter 确认；`--dry-run`/`--mock` 不连接相机或机器人。

**方向与孔位映射是两件事**：切换开合方向会旋转腕相机画面，需核对配置中的
`wrist_view`（row1_at/col1_at）是否仍对应架子的上边/左边。
默认 top/left 为示例映射，不能据此证明两种姿态下均能正确选孔。
`extract_lift_m`、`grasp_z_offset_m`、`drop_xyz`、`drop_height_m` 与 Demo 1 含义相同。

Demo 4 的 `p2p` 现在实际传入观察、拔管和落料三个入口。默认平移 0.10 m/s、
旋转 0.50 rad/s、80 Hz、步长 0.002 m / 0.020 rad；下探使用
`approach_max_translation_speed: 0.04` / `approach_max_rotation_speed: 0.20`。
原先这些入口漏传参数，实际上使用 helper 的 0.04 m/s / 0.20 rad/s。
速度上限提高 2.5 倍不等于全过程耗时缩短 2.5 倍：RPC、定位、纠偏和稳定等待仍占时间。
2026-09-18 23:12 的原地转姿失败与修复边界见
[纠偏与速度配置修复](../../../../docs/franka_p2p_tracking_bias_20260918.md)。

## VLM 回复长度截断

Demo 4 使用 `vlm_reasoning_effort: low`，写入生成的定位配置 `openrouter.reasoning_effort`。
2026-09-19 13:17 的第二支管盖识别两次 `finish_reason=length`，各有 3928 reasoning tokens，
4096 token 总额度几乎耗尽，JSON 未完成。长度截断时，共用定位客户端保留当前 JSON Schema，
使用同一张已采图最多补试一次，输出额度加倍（最低 4096、最高 8192，已有更高额度不降低）。
持续截断则停止，不执行取消格式约束的回退，也不接受可局部解析的截断 JSON。
非长度原因的 JSON 解析失败仍可按原配置执行一次 schema 回退；原始回复逐次保存。
该识别重试不采新图、不重启相机、不触发机械臂动作。详情见
[VLM 截断修复](../../../../docs/vlm_truncation_20260919.md)。

## OpenRouter HTTP 402 排查

2026-09-17 的 inventory 失败日志显示：RealSense 已采集成功，识别架子时 OpenRouter
返回 `HTTP 402 / Insufficient credits`，当时尚未开始机器人运动。
需要为当前 API key 对应的 OpenRouter 账户补充余额，或换用可用的 key。
key 来自运行环境/`atomic_skills/object_locator/.env` 中的 `OPENROUTER_API_KEY`；
若终端已导出同名变量，Demo 1 inventory 的 dotenv 默认不覆盖它，修改 `.env` 后需核对是否仍在使用旧环境变量。
不要将 key 写入 demo YAML 或运行日志。仅降低 `max_tokens` 不能保证解决余额不足。
前两个 demo 会在终端和 `task_result.json` 直接记录这一原因，并把原始 stderr 单独保存。

## RealSense 初始化控制超时

`xioctl(VIDIOC_S_CTRL) ... Contrast ... errno=110` 可发生在 SDK 写入 `visual_preset`
期间；这属于相机 USB 控制请求失败，不能解释为 VLM 或机械臂 RPC 错误。
相机复位现在等待完整 `reset_wait_s`（默认 5 秒），不会仅因设备重新枚举就提前继续。
相机类不再内部反复重开；任务层先关闭采集进程，干净重开一次后才允许一次硬件复位。
同串号的仓库采集进程共享排他锁；占用冲突或 SDK busy 不复位。深度预设已相同时不重写，
同一相机两次复位至少间隔 60 秒；常驻服务在一次恢复失败后等待重连。
不忽略预设写入失败、不切换相机。资源释放失败或重试耗尽则停止。
此恢复仅作用于相机，不调用机械臂 reset。持续 USB/Hub 故障仍需检查连接，重试不代表硬件已恢复。
日常操作与只读检查命令见 [RealSense 操作规范](../../../../docs/realsense_operation.md)。

## OpenRouter TLS EOF / HTTP 400 排查

`SSL: UNEXPECTED_EOF_WHILE_READING` 表示 HTTPS 连接被提前关闭，不能据此判断 key 无效。
2026-09-17 的另一份日志中，架子识别成功，批量 vial 请求先收到上游 HTTP 400，
移除 JSON Schema 后的重发遇到 TLS EOF；HTTP 400 的具体参数原因尚未确认。
共享 OpenRouter 客户端对连接失败、超时和 TLS EOF 最多尝试 3 次，间隔 1、2 秒，
使用同一张已采集图像。不会重新执行抓取或其他机器人动作，也不会关闭证书校验。
证书校验失败、HTTP 401/402 等错误不作为瞬时网络故障重试。
批量 inventory 仅在结构化请求收到 HTTP 400/422 时尝试一次无 Schema 请求，
同时保留明确的 JSON 字段提示；后者失败时保留两个原因。HTTP 400 并不一定由 Schema 导致。
仍失败时查看本次 `inventory_stderr.txt` 并检查主机网络/HTTPS 代理。

## 复用边界和结果

`zerorpc.exceptions.LostRemote: Lost remote after 10s heartbeat` 是机器人 RPC 心跳丢失，
不是视觉 API 错误或一般的 P2P 到达容差失败。2026-09-17 23:07 的记录显示：
左手第一步平移完成，第二步转姿期间断联，尚未执行下探夹取。
对应时段的两臂 controller 日志未发现新的错误记录；这不足以确定断联根因。
终端现在优先显示 stderr 的真正异常和最后运动阶段，不再用 stdout 的 `p2p attempt` 覆盖。
RPC 断联后不自动重发运动，需先检查服务端和机器人状态；提速不代表断联问题已解决。
客户端在内存中保留最近 64 条 ZeroRPC 事件元数据；失联/超时时以
`zerorpc_transport_failure` 写入 stderr（随子任务日志保存），包含请求/响应 ID、
心跳接收记录及发送/接收/分发协程状态，不保存请求载荷。
`enqueue_return` 仅表示进入发送队列，不代表服务端收到或物理动作完成。
服务端计数比最后确认计数多 1 也不能单独证明物理动作成功，不能据此自动续跑。

2026-09-18 已在单支真机复现中捕获一条具体故障链路：服务端收到 `step`
及两次心跳，却没有发回复；RPC 主线程处于 `anon_pipe_write`，ROS launch
进程阻塞于向 VS Code 启动终端 `/dev/pts/4` 写输出。关闭 `ixon` 软件流控
只能短暂解除阻塞，随后验证再次复现，因此不能归因于误按 Ctrl-S，也不能把
`stty -ixon` 当作充分修复。再次故障时直接读到 RPC 阻塞于 stderr 的
`Received action` 写入，确认日志背压进入了同步 RPC 执行路径。
修复应让日志脱离终端：Franka PC 的 `rpc_env.process_prefix` 设置为
`bash /home/owen/franka_dual_ros2/rpc_log_to_file.sh`（部署副本见本 skill 的
`scripts/rpc_log_to_file.sh`），并设置 `rpc_env.log_actions: false`。
该前缀把 stdout/stderr 直接写入 `~/.ros/log/franka_rpc/`，保留原进程退出码。
配置变更在下次正常启动 ROS 时生效；不改变夹爪/启动复位参数。
当前 ROS launch 设置了 RPC `on_exit=Shutdown()`，不能为了切换日志随意终止
RPC；这会连带关闭整套 ROS。当前会话使用单独的终端输出读取器导流到
`/tmp/franka_log_backpressure_fix/ros_terminal_output.log`，随原 launch 结束退出。
已有未确认动作时先核对现场，恢复日志可能解除一个尚未返回的请求的阻塞。

`rpc_trace_ssh_host` 可设为 Franka PC 的 SSH alias（`null` 关闭）。开启后，
每次 live `grasp_handover_N` 会通过 `trace_rpc_command.py` 绑定客户端 strace
和服务端 ZeroMQ IO 线程 strace，日志自动写入 `grasp_handover_N_rpc_trace/`。
跟踪未就绪不启动该抓取命令；结束时自动脱离。SSH 控制通道关闭亦会脱离，
远端另有 20 分钟上限。它会记录网络帧前 256 字节（含部分机器人数据），
与上面的无载荷事件记录不同。`capture.json` 保存远端日志路径，便于收集失败时取回。
跟踪会增加一定开销，只用于定位；不重启服务、不恢复控制器、不重发运动。

主要复用旧 `locate_all_tubes_once.py`、`grasp_right_arm_xyz.py`、
`run_manual_grip_to_insert.py`、`tube_insertion_skill.py`、
`prepare_runtime_wrist_locator_config.py`、`place_held_object_on_table.py`
以及 atomic gripper。取料前打开两爪；交接沿用接收爪闭合确认后才释放抓取爪的实现。
拔出前用本次观察位生成的 wrist 外参重新绑定 cap 定位配置，避免使用旧姿态的标定。
`demo_1_insert.py` 在旧插入流程的观察阶段之后、腕部定位之前做相同绑定；
此适配器只装饰当前进程的阶段派发函数，不改动旧脚本，也不替换原有运动/力检查。

Demo 1/2 的新编排遇到失败立即终止，**不自动 reset 后继续下一支**，因为指定孔位计划依赖已完成的动作。
旧子技能已有的内部姿态校正/力处理保持原样。失败后按现场原恢复流程处理并重新核对物品状态。

输出目录包含 `config_snapshot.json`、`command_plan.json`、`task_result.json`、
生成的 `configs/`、各次感知 JSON/图像和真机日志 `demo.log`。
即使动作命令完成，`physical_verified` 仍为 false：夹爪闭合反馈不等于物品在手证明，
尚未加入独立插入/拔出成功视觉验收。已在用户确认现场后进行单支真机通信故障复现，
完整四支插入/拔出流程尚未完成真机验收。

离线验证：`python3 -m unittest discover -s tests -p test_demo_NO.py -v`。
