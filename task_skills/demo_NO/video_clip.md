# 任务视频拍摄 TODO

- [ ] 1. 7 根试管全部成功插入。
- [ ] 2. 7 根试管场景下，拍摄几次 handover（双臂交接）失败。
- [ ] 3. 4 根试管成功插入与拔出，保留完整流程。
- [ ] 4. 4 根试管场景下，拍摄 handover 失败。
- [ ] 5. 4 根试管插入位置太密集，导致拔出失败。
- [ ] 6. 4 根试管插入位置不密集，但拔出姿势不对，导致拔出失败。
- [ ] 7. 小刀的 handover。
- [ ] 8. 积木的 handover。

每段素材记录视频文件名、使用的配置和本次运行日志目录。失败素材保留初始摆放、失败动作及失败后的物品状态，便于区分交接失败、间距问题和姿态问题；是否成功以实际画面核对。

## scripts 中的 `.sh` 有什么作用、怎么用

脚本目录：`/home/deepcybo/agentic_skills/task_skills/demo_NO/skills/task-configurable-manipulation-demos/scripts`。
以下命令先在同一个终端设置目录变量：

```bash
cd /home/deepcybo/agentic_skills
DEMO_DIR="$PWD/task_skills/demo_NO/skills/task-configurable-manipulation-demos"
```

四个 demo 默认只生成计划。`--dry-run` 用于查看流程，不连接硬件；`--mock` 使用模拟数据检查编排；`--mode live --execute` 才执行真机流程，并默认在首次运动前等待 Enter。真机所需的 ROS/RPC、相机及识别环境需要提前启动，见[现有操作说明](../../docs/pick_tube_insert_rack_demo.md)。这些入口执行机器人任务，不负责录制拍摄视频。

### 1. `demo_1_vial_insert_extract.sh`：4 根试管抓取、交接、插入与拔出

默认读取 `config/demo_1_vial_insert_extract.yaml`。一次识别 4 根散放试管，按初始图像从左到右依次抓取、交接并插入指定孔；全部插完后，再依次拔出并放到侧边桌面。适合拍摄任务 3，以及任务 4 的交接阶段。

```bash
# 查看计划
bash "$DEMO_DIR/scripts/demo_1_vial_insert_extract.sh" --dry-run

# 真机完整插拔
bash "$DEMO_DIR/scripts/demo_1_vial_insert_extract.sh" --mode live --execute

# 仅做真实相机识别和四管数量校验，不开始机器人运动
bash "$DEMO_DIR/scripts/demo_1_vial_insert_extract.sh" \
  --mode live --execute --stop-after-inventory
```

主要配置：`cycles` 指定每根管的插入、拔出孔位；`extract_timing: after_all_inserts` 表示全部插完再拔；`grasp_position` 选择抓取端，`pick_arm` 选择抓取手。当前实现要求 `count: 4` 且有 4 个 cycle，不能直接改成 7 管。孔位为 3 行 × 2 列，写成 `[行, 列]`；默认插入四角 `[1,1]`、`[3,1]`、`[1,2]`、`[3,2]`。任务 5 若使用其他密集孔位，可在此配置中同时修改 `insert` 和 `extract`；`extract` 支持直接填写 `[行, 列]`。

### 2. `demo_2_elongated_handover.sh`：长条物体抓取与交接

默认读取 `config/demo_2_elongated_handover.yaml`。每轮重新识别一件物体，抓取后执行双臂交接；默认 `count: 4`、`after_handover: drop`，交接后放到侧边桌面再处理下一件。适合以小刀或长条积木为目标配置任务 7、8。

单件交接拍摄可先复制配置，再修改副本：

```bash
cp "$DEMO_DIR/config/demo_2_elongated_handover.yaml" \
   "$DEMO_DIR/config/video_handover.yaml"
```

在副本中设置 `count: 1`、`after_handover: hold`，即可在交接后保持夹持。把 `object_description` 改成实际小刀或积木及取料区描述，使用 `front_definition` 明确前端，例如小刀刀尖或积木的标记端；并按物体尺寸核对 `grasp_position` 与交接标定 `transition_json`。

```bash
bash "$DEMO_DIR/scripts/demo_2_elongated_handover.sh" \
  --config "$DEMO_DIR/config/video_handover.yaml" --dry-run

bash "$DEMO_DIR/scripts/demo_2_elongated_handover.sh" \
  --config "$DEMO_DIR/config/video_handover.yaml" --mode live --execute
```

此脚本依赖物体的头尾端点。普通方块积木未必适用，需要先确认能否定义并识别稳定的抓取方向，不能直接把默认长条物体配置视为已适配。

### 3. `demo_3_all_vials_to_rack.sh`：将所有识别到的试管插入架子

默认读取 `config/demo_3_all_vials_to_rack.env`。一次识别桌面全部散放试管和试管架，再按初始图像从左到右逐根抓取、交接、寻找当前空孔并插入；不执行拔出。适合任务 1、2 的 7 管场景，现场需有足够空孔，并核对 inventory 确实识别出 7 根。它没有固定 7 管的 `count` 参数。

```bash
bash "$DEMO_DIR/scripts/demo_3_all_vials_to_rack.sh" --dry-run

# 先核对真实识别结果
bash "$DEMO_DIR/scripts/demo_3_all_vials_to_rack.sh" \
  --mode live --execute --stop-after-inventory

# 完整入架流程
bash "$DEMO_DIR/scripts/demo_3_all_vials_to_rack.sh" --mode live --execute
```

此入口使用 `.env` 配置，替换方式是 `DEMO_CONFIG=/绝对路径/自定义.env bash "$DEMO_DIR/scripts/demo_3_all_vials_to_rack.sh" --dry-run`。默认抓取、交接或插入失败后会执行机器人 reset（打开夹爪、双臂回 home），成功 reset 后继续处理下一根；若 reset 失败则停止。`--no-reset-after-error` 只关闭失败后的 reset，仍会继续下一根，并非“失败即停止”。

### 4. `demo_4_vial_extract.sh`：仅拔出架子四角的 4 根试管

默认读取 `config/demo_4_vial_extract.yaml`。开始前架子四角应各有一根试管，执行手为空手；脚本不抓取散管、不做 handover、不插入，只逐根夹住管盖、竖直拔出并放到架子右侧桌面。适合任务 3 的拔出补拍，以及任务 5、6 中符合四角布局的拔出片段。

```bash
bash "$DEMO_DIR/scripts/demo_4_vial_extract.sh" --dry-run
bash "$DEMO_DIR/scripts/demo_4_vial_extract.sh" --mode live --execute
```

主要配置：`extract_arm: left/right` 选择拔管手；`extract_order` 指定四角顺序，必须恰好包含左上、左下、右上、右下各一次。`gripper_opening_direction: 1` 表示夹爪开合连线沿机械臂 base Y，`2` 表示沿 base X，当前配置为 `2`。切换朝向后还需核对腕相机的 `wrist_view` 孔位映射。该入口不能直接用任意四个中间孔替代四角；这类布局使用 Demo 1 的 `cycles` 配置。

### 5. `rpc_log_to_file.sh`：将 RPC 进程日志写入文件

这是日志辅助脚本，不是拍摄任务入口。它运行传入的命令，将其标准输出和错误输出追加到日志文件，避免 RPC 日志受启动终端输出阻塞影响。

```bash
# 无硬件操作的用法示例：把示例文字写入 /tmp/demo_rpc_logs 下的日志
FRANKA_RPC_LOG_DIR=/tmp/demo_rpc_logs \
  bash "$DEMO_DIR/scripts/rpc_log_to_file.sh" printf '%s\n' 'RPC log example'
```

调用格式为 `bash rpc_log_to_file.sh <命令> [参数...]`，必须传入命令。默认日志目录为 `~/.ros/log/franka_rpc/`，可用 `FRANKA_RPC_LOG_DIR` 覆盖，文件名为 `rpc_时间戳_进程号.log`。部署在 Franka PC 时可作为 ROS 的 `rpc_env.process_prefix` 使用，配置方式见[详细说明](skills/task-configurable-manipulation-demos/SKILL.md#复用边界和结果)。

### 配置与素材记录

Demo 1、2、4 均可通过 `--config /绝对路径/自定义.yaml` 替换配置；建议每种拍摄场景保留一份副本。四个 demo 均支持 `--artifact-dir /tmp/本次拍摄目录`，每次使用新的目录；Demo 1、2、4 的真机入口会拒绝非空目录。

Demo 1、2、4 遇到失败会停止，Demo 3 按上述恢复策略继续。脚本没有“保证 handover 失败”或“保证拔出失败”的专用开关，清单中的失败类型需结合现场视频和日志确认，不能只凭所选配置判定。
