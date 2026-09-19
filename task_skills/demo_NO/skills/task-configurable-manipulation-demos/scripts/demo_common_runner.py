#!/usr/bin/env python3
"""Configurable demos composed from the existing pick/insert skill. Safe by default."""
from __future__ import annotations

import argparse
from datetime import datetime
import json
import math
import os
import re
import signal
from pathlib import Path
import subprocess
import sys

import yaml

ROOT = Path(__file__).resolve().parents[5]
OLD = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"
LOCATOR = ROOT / "atomic_skills/object_locator"
GRIPPER = ROOT / "atomic_skills/dual_franka_gripper/skills/atomic-gripper-franka-open-close/scripts/gripper_control.py"
AXES = ("x_m", "y_m", "z_m")
DEMO_NAMES = {1: "demo_1_vial_insert_extract", 2: "demo_2_elongated_handover", 4: "demo_4_vial_extract"}
P2P_DEFAULTS = {
    "rate_hz": 80.0,
    "max_translation_speed": 0.12,
    "max_rotation_speed": 0.6,
    "max_translation_step": 0.003,
    "max_rotation_step": 0.03,
    "approach_max_translation_speed": 0.08,
    "approach_max_rotation_speed": 0.4,
    "approach_max_translation_step": 0.002,
    "approach_max_rotation_step": 0.02,
}
CORNERS = {"左上": [1, 1], "左下": [3, 1], "右上": [1, 2], "右下": [3, 2],
           "top_left": [1, 1], "bottom_left": [3, 1], "top_right": [1, 2], "bottom_right": [3, 2]}


def opposite(side):
    return "right" if side == "left" else "left"


def slot(value):
    value = CORNERS.get(value, value) if isinstance(value, str) else value
    if (not isinstance(value, list) or len(value) != 2
            or any(type(x) is not int for x in value)
            or value[0] not in (1, 2, 3) or value[1] not in (1, 2)):
        raise ValueError(f"孔位必须是 [行1..3, 列1..2] 或四角名称: {value!r}")
    return list(value)


def vector(value, name):
    if not isinstance(value, list) or len(value) != 3:
        raise ValueError(f"{name} must have three finite numbers")
    if any(isinstance(x, bool) or not isinstance(x, (float, int)) or not math.isfinite(x) for x in value):
        raise ValueError(f"{name} must have three finite numbers")
    return value


def point(payload, key):
    if payload.get("found") is not True or not payload.get("points_base", {}).get("available"):
        raise ValueError("定位失败或缺少 points_base；停止，不降级为 bbox 抓取")
    p = payload["points_base"].get(key, {})
    p = p if all(k in p for k in AXES) else p.get("base", {})
    return vector([p.get(k) for k in AXES], key)


def grasp_geometry(payload, choice, arm):
    if type(choice) is not int or choice not in (1, 2):
        raise ValueError("grasp_position must be 1 or 2")
    head, tail = point(payload, "head"), point(payload, "tail")
    direction = [h - t for h, t in zip(head, tail)]
    length = math.sqrt(sum(v * v for v in direction))
    if length < 0.01 or math.hypot(*direction[:2]) < 0.01:
        raise ValueError("头尾距离过小或物品未平放，无法定向抓取")
    fraction = 0.2 if choice == 1 else 0.8
    xyz = [t + fraction * d for t, d in zip(tail, direction)]
    # Reuse the original tail-side policy; front grasp selects the head-side arm.
    if arm == "auto":
        dy = (tail[1] - head[1]) * (1 if choice == 1 else -1)
        if abs(dy) < 0.002:
            raise ValueError("抓取端方向在选臂死区内；请在配置里指定 pick_arm")
        arm = "right" if dy < 0 else "left"
    return xyz, [v / length for v in direction], arm


def read_json(path):
    return json.loads(Path(path).read_text(encoding="utf-8"))


def write_json(path, data):
    Path(path).parent.mkdir(parents=True, exist_ok=True)
    Path(path).write_text(json.dumps(data, ensure_ascii=False, indent=2), encoding="utf-8")


def failure_reason(stderr, stdout):
    """Surface the current child's failure instead of a generic exit code."""
    detail = (stderr + "\n" + stdout).strip()
    if "LostRemote" in stderr or "Lost remote after" in stderr:
        stages = re.findall(r"^=== (.+?) ===$", stdout, re.MULTILINE)
        stage = f"，发生在 {stages[-1]}" if stages else ""
        cause = next((line.strip() for line in reversed(stderr.splitlines()) if "LostRemote" in line), "LostRemote")
        diagnostics = [line for line in stderr.splitlines() if line.startswith("p2p_rpc_failure: ")]
        context = f"；{diagnostics[-1]}" if diagnostics else ""
        return (f"{cause}{stage}{context}；机器人 RPC 心跳丢失，当前动作结果未确认。"
                "已停止后续命令；检查服务端日志及机器人当前状态后再恢复，不自动重发运动")
    if "OpenRouter" in detail and ("HTTP 402" in detail or "Insufficient credits" in detail):
        return ("OpenRouter HTTP 402：当前 API key 对应账户余额不足。"
                "请补充 OpenRouter 余额，或更换可用的 OPENROUTER_API_KEY 后重试；"
                "降低 max_tokens 不能保证解决余额不足")
    if "OpenRouter" in detail and ("UNEXPECTED_EOF_WHILE_READING" in detail or "SSLEOFError" in detail):
        previous = "；此前结构化识别请求还收到 HTTP 400，请查看完整日志" if "HTTP 400" in detail else ""
        return ("OpenRouter TLS 连接被提前关闭，网络重试后仍失败。"
                "检查网络/HTTPS 代理后重试，保持证书校验开启" + previous)
    exhausted = re.search(r"RealSense did not recover after (\d+) forced reset attempt", stderr)
    if exhausted:
        causes = [line.strip() for line in detail.splitlines()
                  if line.strip().startswith("ERROR:") and any(word in line for word in ("xioctl", "get_xu", "set_xu"))]
        cause = f"；最后采图错误：{causes[-1]}" if causes else ""
        return (f"RealSense 采图失败，已执行 {exhausted.group(1)} 次相机复位仍未恢复"
                f"{cause}；需检查对应相机的 USB 连接、Hub 和供电，已停止后续步骤")[:1000]
    # Ordinary stdout progress must never hide a traceback on stderr.
    for stream in (stderr, stdout):
        lines = [line.strip() for line in stream.splitlines() if line.strip()]
        errors = [line for line in lines if re.match(r"^(?:[\w.]+(?:Error|Exception|TimeoutExpired)|ERROR):", line)]
        if errors:
            return errors[-1][:1000]
    # The observation helper returns JSON on stdout and progress on stderr.
    # A cached-detection notice is not the cause of its nonzero exit status.
    try:
        report = json.loads(stdout)
    except (ValueError, TypeError):
        report = None
    if isinstance(report, dict) and report.get("stopped_reason"):
        reason = str(report["stopped_reason"])
        stages = report.get("stages", [])
        for stage in reversed(stages if isinstance(stages, list) else []):
            result = stage.get("result") if isinstance(stage, dict) else None
            if not isinstance(result, dict) or result.get("ok") is not False:
                continue
            error = result.get("final_error")
            if isinstance(error, dict):
                for key, label, scale, unit in (
                    ("max_translation_error_m", "position_error", 1000, "mm"),
                    ("max_rotation_error_rad", "rotation_error", 1, "rad"),
                ):
                    value = error.get(key)
                    if type(value) in (int, float) and math.isfinite(value):
                        reason += f"; {label}={value * scale:.4f} {unit}"
                if "correction_index" in error:
                    reason += f"; correction_index={error['correction_index']}"
            stalled = result.get("stalled")
            if isinstance(stalled, dict) and stalled.get("reason") == "tracking_compensation_limit":
                reason += "; tracking_compensation_limit（已耗尽当前方向的有界补偿，增加次数不能扩大此范围）"
                for arm in ("left", "right"):
                    bias = stalled.get(f"{arm}_bias", {})
                    for key, limit_key, scale, unit in (
                        ("translation_m", "max_translation_m", 1000, "mm"),
                        ("rotation_rad", "max_rotation_rad", 1, "rad"),
                    ):
                        value, limit = bias.get(key), bias.get(limit_key)
                        if all(type(v) in (int, float) and math.isfinite(v) for v in (value, limit)):
                            reason += f"; {arm}_{key}={value * scale:.4f}/{limit * scale:.4f} {unit}"
            break
        return reason[:1000]
    lines = [line.strip() for line in (stderr or stdout).splitlines() if line.strip()]
    return lines[-1][:1000] if lines else "子进程未输出错误详情"


def p2p_settings(config):
    overrides = config.get("p2p", {})
    if not isinstance(overrides, dict) or set(overrides) - set(P2P_DEFAULTS):
        raise ValueError("p2p must be a mapping containing only the documented speed/rate/step settings")
    settings = {**P2P_DEFAULTS, **overrides}
    for key, value in settings.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"p2p.{key} must be a positive finite number")
    return settings


def p2p_args(config, *, include_approach=True):
    return [arg for key, value in p2p_settings(config).items()
            if include_approach or not key.startswith("approach_")
            for arg in ("--" + key.replace("_", "-"), str(value))]


def extract_tcp_rotvec(choice, jaw_axis="y"):
    """TCP Z down; the jaw opening LINE is perpendicular/parallel to base X."""
    if type(choice) is not int or choice not in (1, 2) or jaw_axis not in ("x", "y"):
        raise ValueError("gripper_opening_direction must be 1 or 2; jaw_opening_axis must be x or y")
    yaw = math.pi / 2 if (choice == 2) == (jaw_axis == "y") else 0.0
    # Rz(yaw) Rx(pi): a pi rotation about this horizontal axis.
    return [math.pi * math.cos(yaw / 2), math.pi * math.sin(yaw / 2), 0.0]


def validate_extract_only(config):
    retries = config.get("drop_perception_retries", 1)
    if type(retries) is not int or not 0 <= retries <= 2:
        raise ValueError("drop_perception_retries must be an integer from 0 to 2")
    if config.get("vlm_reasoning_effort") not in (None, "minimal", "low", "medium", "high"):
        raise ValueError("vlm_reasoning_effort must be minimal/low/medium/high or null")
    if config.get("extract_arm") not in ("left", "right"):
        raise ValueError("extract_arm must be left or right")
    extract_tcp_rotvec(config.get("gripper_opening_direction"), config.get("jaw_opening_axis", "y"))
    order = config.get("extract_order")
    corners = {"左上", "左下", "右上", "右下"}
    if not isinstance(order, list) or len(order) != 4 or any(not isinstance(p, str) for p in order) or set(order) != corners:
        raise ValueError("extract_order 必须恰好包含左上、左下、右上、右下各一次")
    for key in ("grasp_z_offset_m", "extract_lift_m", "drop_height_m"):
        value = config.get(key)
        if type(value) not in (int, float) or not math.isfinite(value) or value < 0:
            raise ValueError(f"{key} must be finite and nonnegative")
    if min(config["extract_lift_m"], config["drop_height_m"]) <= 0:
        raise ValueError("extract_lift_m and drop_height_m must be positive")
    for key, default in (("rack_z_range_m", [-0.25, -0.10]),
                         ("extract_cap_z_range_m", [-0.02, 0.12]),
                         ("drop_lateral_range_m", [0.12, 0.30])):
        values = config.get(key, default)
        if (not isinstance(values, list) or len(values) != 2 or
                any(type(v) not in (int, float) or not math.isfinite(v) for v in values) or values[0] >= values[1]):
            raise ValueError(f"{key} must be an increasing finite [min, max]")
    if config.get("drop_lateral_range_m", [0.12, 0.30])[0] <= 0:
        raise ValueError("drop_lateral_range_m must stay strictly right of the rack")
    for key, default in (("extract_observe_height_m", 0.12), ("extract_cap_max_xy_m", 0.10),
                         ("drop_max_fore_aft_m", 0.15),
                         ("extract_pregrasp_xy_position_tolerance_m", 0.003),
                         ("extract_pregrasp_xy_rotation_tolerance_rad", 0.03),
                         ("extract_pregrasp_xy_settle_time_sec", 2.0),
                         ("extract_observe_settle_time_sec", 1.0),
                         ("extract_approach_settle_time_sec", 2.0),
                         ("extract_lift_settle_time_sec", 2.0),
                         ("drop_settle_time_sec", 1.0),
                         ("extract_position_tolerance_m", 0.003),
                         ("extract_rotation_tolerance_rad", 0.03),
                         ("extract_lift_position_tolerance_m", config.get("extract_position_tolerance_m", 0.003)),
                         ("extract_lift_rotation_tolerance_rad", config.get("extract_rotation_tolerance_rad", 0.03)),
                         ("drop_position_tolerance_m", 0.005),
                         ("drop_rotation_tolerance_rad", 0.035)):
        value = config.get(key, default)
        if type(value) not in (int, float) or not math.isfinite(value) or value <= 0:
            raise ValueError(f"{key} must be positive and finite")
    corrections = config.get("extract_observe_max_correction_iters", 10)
    if type(corrections) is not int or corrections < 1:
        raise ValueError("extract_observe_max_correction_iters must be a positive integer")
    if not isinstance(config.get("drop_description"), str) or not config["drop_description"].strip():
        raise ValueError("drop_description is required")
    for side in ("left", "right"):
        if config["drop_xyz"][side] is not None:
            vector(config["drop_xyz"][side], f"drop_xyz.{side}")
        view = config["wrist_view"][side]
        if view["row1_at"] not in ("top", "bottom", "left", "right") or view["col1_at"] not in ("top", "bottom", "left", "right"):
            raise ValueError("wrist_view directions must be top/bottom/left/right")
        if (view["row1_at"] in ("top", "bottom")) == (view["col1_at"] in ("top", "bottom")):
            raise ValueError("wrist_view row and column axes must be perpendicular")


def validate(config):
    p2p_settings(config)
    for key, default in (("extract_observe_position_tolerance_m", 0.003),
                         ("extract_observe_rotation_tolerance_rad", 0.03)):
        observe_tolerance = config.get(key, default)
        if type(observe_tolerance) not in (int, float) or not math.isfinite(observe_tolerance) or observe_tolerance <= 0:
            raise ValueError(f"{key} must be a positive finite number")
    if config.get("demo") == 4:
        validate_extract_only(config)
        return
    trace_host = config.get("rpc_trace_ssh_host")
    if trace_host is not None and (not isinstance(trace_host, str) or
                                  not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.@-]*", trace_host)):
        raise ValueError("rpc_trace_ssh_host must be an SSH alias or null")
    for key, default in (("handover_receiver_stable_sec", 0.5), ("handover_release_delay_sec", 2.0), ("handover_close_timeout_sec", 5.0)):
        value = config.get(key, default)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or value < 0:
            raise ValueError(f"{key} must be finite and nonnegative")
    if config.get("handover_close_timeout_sec", 5.0) <= config.get("handover_receiver_stable_sec", 0.5):
        raise ValueError("handover_close_timeout_sec must exceed handover_receiver_stable_sec")
    retreat = config.get("handover_retreat_distance_m", 0.20)
    if isinstance(retreat, bool) or not isinstance(retreat, (float, int)) or not math.isfinite(retreat) or retreat <= 0:
        raise ValueError("handover_retreat_distance_m must be a positive finite number")
    if config.get("demo") not in (1, 2):
        raise ValueError("demo must be 1, 2 or 4")
    if type(config.get("grasp_position")) is not int or config["grasp_position"] not in (1, 2):
        raise ValueError("grasp_position must be 1 or 2")
    if config.get("pick_arm") not in ("left", "right", "auto"):
        raise ValueError("pick_arm must be left, right or auto")
    if type(config.get("count")) is not int or config["count"] < 1:
        raise ValueError("count must be a positive integer")
    for key in ("grasp_z_offset_m", "lift_m", "extract_lift_m", "drop_height_m", "insert_depth_m", "hole_standoff_m"):
        v = config[key]
        if isinstance(v, bool) or not isinstance(v, (float, int)) or not math.isfinite(v) or v < 0:
            raise ValueError(f"{key} must be finite and nonnegative")
    if min(config["lift_m"], config["extract_lift_m"], config["drop_height_m"], config["insert_depth_m"]) <= 0:
        raise ValueError("lift, extraction, drop height and insertion depth must be positive")
    for side in ("left", "right"):
        if config["drop_xyz"][side] is not None:
            vector(config["drop_xyz"][side], f"drop_xyz.{side}")
    if config["handover_transition_branch"] not in ("active", "partner"):
        raise ValueError("handover_transition_branch must be active or partner")
    if config["jaw_opening_axis"] not in ("x", "y") or config["jaw_perpendicular_turn"] not in ("cw", "ccw"):
        raise ValueError("invalid jaw axis or turn")
    if config["demo"] == 1:
        corrections = config.get("extract_observe_max_correction_iters", 10)
        if type(corrections) is not int or corrections < 1:
            raise ValueError("extract_observe_max_correction_iters must be a positive integer")
        if config["count"] != 4 or len(config["cycles"]) != 4:
            raise ValueError("Demo 1 必须配置四次插入/拔出")
        if config["extract_timing"] not in ("after_each_insert", "after_all_inserts"):
            raise ValueError("invalid extract_timing")
        occupied = {tuple(slot(x)) for x in config.get("initial_occupied", [])}
        inserts, extracts = [], []
        for cycle in config["cycles"]:
            inserts.append(tuple(slot(cycle["insert"])))
            extracts.append(tuple(slot(cycle["extract"])))
            choice = cycle.get("grasp_position", config["grasp_position"])
            if type(choice) is not int or choice not in (1, 2):
                raise ValueError("cycle grasp_position must be 1 or 2")
        schedule = ([action for pair in zip(zip(["insert"] * 4, inserts), zip(["extract"] * 4, extracts)) for action in pair]
                    if config["extract_timing"] == "after_each_insert"
                    else list(zip(["insert"] * 4, inserts)) + list(zip(["extract"] * 4, extracts)))
        for action, position in schedule:
            if action == "insert":
                if position in occupied:
                    raise ValueError(f"计划插入已占用孔 {position}；检查 cycles / initial_occupied")
                occupied.add(position)
            else:
                if position not in occupied:
                    raise ValueError(f"计划拔出空孔 {position}；预放 vial 请填写 initial_occupied")
                occupied.remove(position)
        for side in ("left", "right"):
            view = config["wrist_view"][side]
            if view["row1_at"] not in ("top", "bottom", "left", "right") or view["col1_at"] not in ("top", "bottom", "left", "right"):
                raise ValueError("wrist_view directions must be top/bottom/left/right")
            if (view["row1_at"] in ("top", "bottom")) == (view["col1_at"] in ("top", "bottom")):
                raise ValueError("wrist_view row and column axes must be perpendicular")
    elif config["after_handover"] not in ("drop", "hold") or (config["after_handover"] == "hold" and config["count"] != 1):
        raise ValueError("after_handover=hold 时 count 必须为 1；多物品需 drop")


def target_description(position, view, occupied=False):
    row, col = slot(position)
    target = "vial cap seated in" if occupied else "empty opening of"
    condition = ("The requested position MUST contain a vial. Return a tight box around its cap only."
                 if occupied else "The requested position MUST be empty. Return a tight box around only its opening. A cap or tube means occupied.")
    return (f"Find the {target} exactly row {row}, column {col} of the RED vial rack. "
            f"The physical rack has exactly 3 rows and 2 columns (6 positions). "
            f"In this wrist image, row 1 is at the {view['row1_at']} edge, and rows increase toward the opposite edge; "
            f"column 1 is at the {view['col1_at']} edge, and column 2 is opposite. "
            "Count ALL six rack positions, including occupied ones; never renumber just the empty holes or visible caps. "
            f"{condition} If this exact position or the full layout is not identifiable, return found=false. "
            "Never substitute a nearby position or the clearest/nearest empty hole. Ignore other racks, the held object and gripper.")


def validate_extract_rack(payload, config):
    xyz = point(payload, "bbox_center")
    # The observation helper consumes position_base, so check that exact field too.
    position = payload.get("position_base", {})
    used = vector([position.get(k) for k in AXES], "rack.position_base")
    if not position.get("available") or math.dist(xyz, used) > 0.01:
        raise ValueError("红架定位坐标不一致；未执行观察动作")
    low, high = config.get("rack_z_range_m", [-0.25, -0.10])
    if not low <= used[2] <= high:
        raise ValueError(f"红架高度异常 z={used[2]:.4f} m，不在 [{low}, {high}]；可能被机械臂遮挡，未执行观察动作")
    return used


def validate_extract_cap(payload, rack_xyz, config):
    xyz = point(payload, "bbox_center")
    low, high = config.get("extract_cap_z_range_m", [-0.02, 0.12])
    if (math.dist(xyz[:2], rack_xyz[:2]) > config.get("extract_cap_max_xy_m", 0.10)
            or not low <= xyz[2] - rack_xyz[2] <= high):
        raise ValueError("拔管目标偏离本轮固定红架位置/高度；停止，不开夹爪或下探")
    return xyz


def validate_right_drop(xyz, rack_xyz, config, *, surface=False):
    vector(xyz, "drop_xyz")
    lateral = rack_xyz[1] - xyz[1]  # Workspace right is base -Y, for either arm.
    low, high = config.get("drop_lateral_range_m", [0.12, 0.30])
    if not low <= lateral <= high or abs(xyz[0] - rack_xyz[0]) > config.get("drop_max_fore_aft_m", 0.15):
        raise ValueError(f"落料点不在红架右侧允许区域（base -Y {low}..{high} m）；"
                         f"实际右移={lateral:.4f} m，前后偏移={xyz[0] - rack_xyz[0]:.4f} m；停止，不移动或松爪")
    surface_z = xyz[2] if surface else xyz[2] - config["drop_height_m"]
    if not -0.10 <= surface_z - rack_xyz[2] <= 0.02:
        raise ValueError("右侧落料桌面高度异常；停止，不移动或松爪")


def drop_image_bounds(payload):
    box = payload.get("detection", {}).get("bbox", {})
    intrinsics = payload.get("intrinsics", {})
    values = [box.get(k) for k in ("x_min", "y_min", "x_max", "y_max")]
    width, height = intrinsics.get("width"), intrinsics.get("height")
    if (any(type(v) not in (int, float) or not math.isfinite(v) for v in [*values, width, height])
            or not 0 <= values[0] < values[2] <= width or not 0 <= values[1] < values[3] <= height):
        raise ValueError("落料图像检查缺少有效识别框或图像尺寸；停止，不移动或松爪")
    return values, (width, height)


def validate_drop_image(payload, rack_payload):
    box, size = drop_image_bounds(payload)
    rack_box, rack_size = drop_image_bounds(rack_payload)
    if size != rack_size:
        raise ValueError("落料与红架参考图像尺寸不同；停止，不移动或松爪")
    # Head-camera image right is the workspace right for this demo setup.
    if box[0] < rack_box[2] + 5:
        raise ValueError(f"落料识别框未完全位于红架图像右侧：x_min={box[0]:.1f}，"
                         f"要求 >= {rack_box[2] + 5:.1f} px；可能把架子当作桌面")


def run_logged_command(argv, cwd, stdout_path, stderr_path):
    """Persist child output as it happens, and stop the child tree on Ctrl-C.

    Stopping a client is NOT a controller emergency stop: the last robot target
    may still be active. Never send reset/home/open or retry RPCs while cleaning up.
    """
    with stdout_path.open("w", encoding="utf-8") as stdout, stderr_path.open("w", encoding="utf-8") as stderr:
        process = subprocess.Popen(argv, cwd=cwd, stdout=stdout, stderr=stderr,
                                   env={**os.environ, "PYTHONUNBUFFERED": "1"}, start_new_session=True)
        try:
            code = process.wait()
        except KeyboardInterrupt:
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=3)
            except subprocess.TimeoutExpired:
                pass
            finally:
                # Also terminate descendants if their parent exited first.
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=3)
            raise
    return subprocess.CompletedProcess(argv, code, stdout_path.read_text(encoding="utf-8", errors="replace"),
                                       stderr_path.read_text(encoding="utf-8", errors="replace"))


class Demo:
    def __init__(self, config, config_path, artifacts, mode, wait=True, stop_after_inventory=False):
        validate(config)
        if stop_after_inventory and config["demo"] != 1:
            raise ValueError("--stop-after-inventory 仅适用于 Demo 1")
        self.stop_after_inventory = stop_after_inventory
        config = {**config, "p2p": p2p_settings(config)}
        self.c, self.config_path, self.out, self.mode, self.wait = config, Path(config_path), Path(artifacts), mode, wait
        self.out.mkdir(parents=True, exist_ok=True)
        self.events = []
        self.extract_rack_reference = None
        self.extract_rack_xyz = None
        self.python = config.get("robot_python", "python3")
        self.locator_python = str(LOCATOR / ".venv/bin/python")

    def record(self, name, **data):
        self.events.append({"stage": name, **data})
        write_json(self.out / "command_plan.json", self.events)
        print(f"[{DEMO_NAMES[self.c['demo']]}] {name}", flush=True)

    def command(self, name, argv):
        argv = [str(x) for x in argv]
        self.record(name, argv=argv, executed=self.mode == "live")
        if self.mode == "live":
            trace_host = self.c.get("rpc_trace_ssh_host")
            if trace_host and name.startswith("grasp_handover_"):
                argv = [self.python, str(Path(__file__).with_name("trace_rpc_command.py")),
                        "--host", trace_host, "--output-dir", str(self.out / f"{name}_rpc_trace"), "--", *argv]
                print(f"[rpc_trace] starting client/server capture for {name}", flush=True)
            try:
                result = run_logged_command(argv, LOCATOR, self.out / f"{name}_stdout.txt", self.out / f"{name}_stderr.txt")
            except KeyboardInterrupt:
                with (self.out / "demo.log").open("a", encoding="utf-8") as log:
                    log.write(f"\n=== {name} INTERRUPTED; robot stop unconfirmed ===\n")
                    for suffix in ("stderr", "stdout"):
                        path = self.out / f"{name}_{suffix}.txt"
                        if path.exists():
                            log.write(path.read_text(encoding="utf-8", errors="replace"))
                raise
            with (self.out / "demo.log").open("a", encoding="utf-8") as log:
                log.write(f"\n=== {name} ===\n")
                log.write(result.stderr)
                log.write(result.stdout)
            (self.out / f"{name}_stdout.txt").write_text(result.stdout, encoding="utf-8")
            (self.out / f"{name}_stderr.txt").write_text(result.stderr, encoding="utf-8")
            if result.returncode:
                reason = failure_reason(result.stderr, result.stdout)
                raise RuntimeError(f"{name} failed ({result.returncode}): {reason}; see {self.out / 'demo.log'}")
            return result.stdout
        return ""

    def legacy(self, name, script, *args):
        return self.command(name, [self.python, OLD / script, *args, "--execute"])

    def locator_config(self, name, base, description=None, *, object_name=None, refine=True):
        path = Path(base)
        if not path.is_absolute():
            path = LOCATOR / path
        data = yaml.safe_load(path.read_text(encoding="utf-8"))
        calibration = Path(data["calibration"]["file"])
        if not calibration.is_absolute():
            calibration = path.parent / calibration
        data["calibration"]["file"] = str(calibration.resolve())
        if description:
            data["target"] = {"name": object_name or name, "description": description}
        data["detector"].update(mode="vlm", color="auto", validate_vlm_color=False,
                                 fallback_to_color_on_vlm_mismatch=False, refine_vlm_with_sam=refine)
        data["realsense"]["reset_on_start"] = False
        if self.c["demo"] == 4 and self.c.get("vlm_reasoning_effort") is not None:
            data.setdefault("openrouter", {})["reasoning_effort"] = self.c["vlm_reasoning_effort"]
        dest = self.out / "configs" / f"{name}.yaml"
        dest.parent.mkdir(parents=True, exist_ok=True)
        dest.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding="utf-8")
        return dest

    def locate(self, name, config, *, mock_key="bbox_center"):
        dest = self.out / f"{name}.json"
        self.command(name, [self.python, Path(__file__).with_name("demo_camera_locate.py"), "--config", config, "--json", "--result-json", dest,
                            "--history-dir", self.out / f"{name}_history", "--output", self.out / f"{name}_panel.jpg",
                            "--output-rgb", self.out / f"{name}_rgb.jpg", "--save-vlm-response", self.out / f"{name}_vlm.json"])
        if self.mode == "mock":
            points = {"head": [0.5, -0.10, -0.10], "tail": [0.5, 0.10, -0.10], "bbox_center": [0.5, 0.0, -0.10]}
            if self.c["demo"] == 4:
                points["bbox_center"] = ([0.5, -0.20, -0.23] if name.startswith("drop_area_") else
                                         [0.5, 0.0, -0.16] if name.startswith("extract_cap_") else [0.5, 0.0, -0.20])
            mock = {"found": True, "mock": True, "position_base": {"available": True, **dict(zip(AXES, points[mock_key]))},
                    "points_base": {"available": True, **{k: {"base": dict(zip(AXES, v))} for k, v in points.items()}}}
            if self.c["demo"] == 4:
                bounds = [460, 180, 520, 240] if name.startswith("drop_area_") else [280, 180, 360, 280]
                mock.update(intrinsics={"width": 640, "height": 480},
                            detection={"bbox": dict(zip(("x_min", "y_min", "x_max", "y_max"), bounds))})
            write_json(dest, mock)
        result = read_json(dest)
        point(result, mock_key)
        return dest

    def pause(self):
        if self.mode == "live" and self.wait:
            if not sys.stdin.isatty():
                raise ValueError("运动前确认需要终端；自动化时显式使用 --no-wait-before-motion")
            input(f"检查 {self.out} 中识别图和配置。Enter 开始运动，Ctrl-C 退出: ")

    def grasp(self, source, choice, index):
        xyz, direction, arm = grasp_geometry(read_json(source), choice, self.c["pick_arm"])
        receiver = opposite(arm)
        branch = arm if self.c["handover_transition_branch"] == "active" else receiver
        self.record(f"grasp_geometry_{index}", choice=choice, xyz=xyz, axis=direction, pick_arm=arm, receiver=receiver,
                    source=str(source), fraction_from_tail=0.2 if choice == 1 else 0.8, p2p=self.c["p2p"])
        # The legacy helper does not open the grasping/receiving grippers itself.
        for side in (arm, receiver):
            self.command(f"open_before_grasp_{index}_{side}", [self.python, GRIPPER, "open", "--side", side, "--execute"])
        self.legacy(f"grasp_handover_{index}", "grasp_right_arm_xyz.py", "--arm", arm,
                    "--xyz", json.dumps(xyz), "--orientation-vector", json.dumps(direction), "--directional-orientation",
                    "--grasp-target-z-offset-m", self.c["grasp_z_offset_m"], "--lift-after-grasp-m", self.c["lift_m"],
                    "--transition-json", self.transition_path(), "--transition-side", branch,
                    "--go-home-before-transition",
                    "--retreat-after-transfer-m", self.c.get("handover_retreat_distance_m", 0.20),
                    "--partner-gripper-stable-sec", self.c.get("handover_receiver_stable_sec", 0.5),
                    "--partner-gripper-close-timeout-sec", self.c.get("handover_close_timeout_sec", 5.0),
                    "--after-partner-close-sleep-sec", self.c.get("handover_release_delay_sec", 2.0),
                    "--settle-time-sec", "0.7", "--max-correction-iters", "10", "--position-tolerance-m", "0.005",
                    "--rotation-tolerance-rad", "0.05", "--transition-rotation-tolerance-rad", "0.06", *p2p_args(self.c))
        return receiver

    def transition_path(self):
        value = self.c.get("transition_json")
        return (self.config_path.parent / value).resolve() if value else OLD / "transition.json"

    def rack_config(self):
        return self.locator_config("red_rack", "config_rack_center_generic_vlm.yaml",
                                   "Locate the entire RED vial rack with exactly 3 rows and 2 columns of positions. "
                                   "Ignore other racks. If no such red rack is visible, return found=false.", object_name="red vial rack")

    def hole_configs(self, index, position, occupied=False):
        paths = {}
        for side in ("left", "right"):
            paths[side] = self.locator_config(f"{'cap' if occupied else 'hole'}_{index}_{side}",
                f"config_rack_empty_hole_{side}_wrist_generic_vlm.yaml",
                target_description(position, self.c["wrist_view"][side], occupied),
                object_name="vial cap at specified rack position" if occupied else "specified empty red rack hole")
        return paths

    def insert(self, index, receiver, position):
        configs = self.hole_configs(index, position)
        dest = self.out / f"insert_{index}"
        rack_config = self.rack_config()
        rack = self.locate(f"rack_before_insert_{index}", rack_config)
        result_path = dest / "result.json"
        self.command(f"insert_{index}", [self.python, Path(__file__).with_name("demo_1_insert.py"), "--execute", "--holder-side", receiver,
                    "--output-json", result_path,
                    "--rack-config", rack_config, "--rack-result-json", rack,
                    "--left-hole-config", configs["left"], "--right-hole-config", configs["right"],
                    "--wrist-perception-mode", "legacy-vlm", "--artifact-dir", dest, "--log-file", dest / "insert.log",
                    "--jaw-perpendicular-to-rack", "--jaw-opening-axis", self.c["jaw_opening_axis"],
                    "--jaw-perpendicular-turn", self.c["jaw_perpendicular_turn"],
                    "--insert-depth-m", self.c["insert_depth_m"], "--hole-standoff-m", self.c["hole_standoff_m"]])
        if self.mode == "live":
            try:
                report = read_json(result_path)
            except (OSError, ValueError) as exc:
                raise RuntimeError(f"insertion {index} result missing/invalid: {result_path}; motion may already have occurred; do not replay") from exc
            if not isinstance(report, dict) or report.get("completion_flag") is not True:
                raise RuntimeError(f"insertion {index} did not report completion; see {result_path}; do not replay")
            self.record(f"insert_result_{index}", result_json=str(result_path),
                        completion_status=report.get("completion_status"),
                        insert_tolerance_warning=report.get("insert_tolerance_warning"),
                        physical_verified=report.get("physical_verified", False))

    def drop(self, index, side):
        xyz = self.c["drop_xyz"][side]
        if xyz is None:
            description = self.c["drop_description"]
            retries = 0
            rack_payload = None
            if self.c["demo"] == 4:
                rack_payload = read_json(self.extract_rack_reference)
                rack_box, size = drop_image_bounds(rack_payload)
                low, high = self.c.get("drop_lateral_range_m", [0.12, 0.30])
                description += (" REQUIRED: choose bare tabletop strictly to the RIGHT of the red rack in the head image,"
                                f" roughly {low * 100:g}-{high * 100:g} cm away, at a similar front/back position."
                                f" The reference red rack bbox in original {size[0]:g}x{size[1]:g} pixels is {rack_box}."
                                f" Choose a box entirely beyond x={rack_box[2] + 5:g} pixels to its RIGHT"
                                f" (normalized x1 >= {(rack_box[2] + 5) / size[0] * 1000:.1f})."
                                " The red rack itself, its holes and caps are NOT empty tabletop."
                                " Exclude all previously dropped vials. Never choose the front, left, rack or gripper."
                                " Return found=false if no clear right-side area exists.")
                retries = self.c.get("drop_perception_retries", 1)
            feedback = ""
            for attempt in range(retries + 1):
                suffix = "" if attempt == 0 else f"_retry_{attempt}"
                conf = self.locator_config(f"drop_{index}{suffix}", "config_empty_table_place_vlm.yaml", description + feedback,
                                           object_name="empty tabletop drop area", refine=False)
                try:
                    source = self.locate(f"drop_area_{index}{suffix}", conf)
                    if self.c["demo"] == 4:
                        candidate = read_json(source)
                        validate_drop_image(candidate, rack_payload)
                        validate_right_drop(point(candidate, "bbox_center"), self.extract_rack_xyz, self.c, surface=True)
                    break
                except ValueError as exc:
                    if attempt >= retries:
                        raise
                    self.record(f"drop_area_rejected_{index}_{attempt + 1}", reason=str(exc),
                                perception_only_retry=True)
                    print(f"落料选点未通过检查，将重新识别一次：{exc}", flush=True)
                    feedback = (f" Previous candidate was REJECTED: {exc}. Choose a different bare-tabletop box"
                                " fully to the right of the rack; do not return the rack or reuse the rejected area.")
            args = ["--place-result-json", source, "--place-z-offset-m", self.c["drop_height_m"]]
        else:
            if self.c["demo"] == 4:
                validate_right_drop(xyz, self.extract_rack_xyz, self.c)
            args = ["--place-xyz", json.dumps(xyz)]
        # The extraction-only demo stays in Cartesian control between vials.
        # Its drop helper releases and retracts without a joint-home switch.
        home_args = [] if self.c["demo"] == 4 else ["--go-home-after-success"]
        if self.c["demo"] == 4:
            args += ["--keep-current-rotation", "--max-release-rise-m", "0.005", "--retract-distance-m", "0"]
            args += p2p_args(self.c, include_approach=False)
            args += ["--position-tolerance-m", self.c.get("drop_position_tolerance_m", 0.005),
                     "--rotation-tolerance-rad", self.c.get("drop_rotation_tolerance_rad", 0.035),
                     "--settle-time-sec", self.c.get("drop_settle_time_sec", 1.0)]
        self.legacy(f"drop_{index}_{side}", "place_held_object_on_table.py", "--side", side, *args,
                    *home_args, "--max-correction-iters", "10")

    def extract(self, index, receiver, position, *, pause_before_motion=False):
        configs = self.hole_configs(index, position, occupied=True)
        observation = self.out / f"extract_observe_{index}"
        rack_config = self.rack_config()
        if self.c["demo"] == 4 and self.extract_rack_reference is not None:
            rack = self.extract_rack_reference
            self.record(f"reuse_rack_reference_{index}", source="fixed_rack_reference", result_json=str(rack))
        else:
            rack = self.locate(f"rack_before_extract_{index}", rack_config)
        if self.c["demo"] == 4:
            self.extract_rack_xyz = validate_extract_rack(read_json(rack), self.c)
            self.extract_rack_reference = rack  # This batch assumes the rack stays fixed.
        if pause_before_motion:
            self.pause()
        orientation = ["--jaw-perpendicular-to-rack", "--jaw-opening-axis", self.c.get("jaw_opening_axis", "y"),
                       "--jaw-perpendicular-turn", self.c.get("jaw_perpendicular_turn", "ccw")]
        if self.c["demo"] == 4:
            orientation = ["--aligned-tcp-rotvec", json.dumps(extract_tcp_rotvec(
                self.c["gripper_opening_direction"], self.c.get("jaw_opening_axis", "y"))),
                "--observe-height-m", self.c.get("extract_observe_height_m", 0.12),
                "--settle-time-sec", self.c.get("extract_observe_settle_time_sec", 1.0),
                *p2p_args(self.c, include_approach=False)]
        self.legacy(f"extract_observe_{index}", "tube_insertion_skill.py", "--holder-side", receiver,
                    "--no-yield-non-holder-arm",
                    "--rack-config", rack_config, "--rack-result-json", rack, "--artifact-dir", observation,
                    "--roll-target-rad", math.pi, "--pitch-zero-rad", "0",
                    *orientation,
                    "--max-correction-iters", self.c.get("extract_observe_max_correction_iters", 10),
                    "--position-tolerance-m", self.c.get("extract_observe_position_tolerance_m", 0.003),
                    "--rotation-tolerance-rad", self.c.get("extract_observe_rotation_tolerance_rad", 0.03),
                    "--stop-after-observe")
        runtime = self.out / "configs" / f"cap_runtime_{index}.yaml"
        self.command(f"bind_wrist_calibration_{index}", [self.locator_python, OLD / "prepare_runtime_wrist_locator_config.py",
                     "--source-config", configs[receiver], "--runtime-calibration-dir", observation / "object_locator_runtime",
                     "--output-config", runtime])
        cap = self.locate(f"extract_cap_{index}", runtime)
        if self.c["demo"] == 4:
            validate_extract_cap(read_json(cap), self.extract_rack_xyz, self.c)
        self.command(f"open_for_extract_{index}", [self.python, GRIPPER, "open", "--side", receiver, "--execute"])
        self.legacy(f"extract_{index}_{receiver}", "grasp_right_arm_xyz.py", "--arm", receiver,
                    "--result-json", cap, "--result-grasp-point", "bbox_center", "--no-result-orientation",
                    "--keep-current-rotation", "--grasp-target-z-offset-m", self.c["grasp_z_offset_m"],
                    "--lift-after-grasp-m", self.c["extract_lift_m"], "--no-return-transition", "--max-correction-iters", "10",
                    *([*p2p_args(self.c),
                       "--settle-time-sec", self.c.get("extract_lift_settle_time_sec", 2.0),
                       "--approach-settle-time-sec", self.c.get("extract_approach_settle_time_sec", 2.0),
                       "--position-tolerance-m", self.c.get("extract_position_tolerance_m", 0.003),
                       "--rotation-tolerance-rad", self.c.get("extract_rotation_tolerance_rad", 0.03),
                       "--lift-position-tolerance-m", self.c.get("extract_lift_position_tolerance_m", self.c.get("extract_position_tolerance_m", 0.003)),
                       "--lift-rotation-tolerance-rad", self.c.get("extract_lift_rotation_tolerance_rad", self.c.get("extract_rotation_tolerance_rad", 0.03)),
                       "--pregrasp-xy-position-tolerance-m", self.c.get("extract_pregrasp_xy_position_tolerance_m", 0.003),
                       "--pregrasp-xy-rotation-tolerance-rad", self.c.get("extract_pregrasp_xy_rotation_tolerance_rad", 0.03),
                       "--pregrasp-xy-settle-time-sec", self.c.get("extract_pregrasp_xy_settle_time_sec", 2.0)]
                      if self.c["demo"] == 4 else []))
        self.drop(index, receiver)

    def plan(self):
        """No fake perception coordinates or live helper imports in dry_run."""
        if self.c["demo"] == 4:
            for i, position in enumerate(self.c["extract_order"], 1):
                configs = self.hole_configs(i, position, occupied=True)
                self.record(f"extract_observe_{i}", side=self.c["extract_arm"],
                            gripper_opening_direction=self.c["gripper_opening_direction"],
                            tcp_rotvec=extract_tcp_rotvec(self.c["gripper_opening_direction"], self.c.get("jaw_opening_axis", "y")),
                            frame="selected arm base", max_correction_iters=self.c.get("extract_observe_max_correction_iters", 10),
                            position_tolerance_m=self.c.get("extract_observe_position_tolerance_m", 0.003),
                            rotation_tolerance_rad=self.c.get("extract_observe_rotation_tolerance_rad", 0.03),
                            settle_time_sec=self.c.get("extract_observe_settle_time_sec", 1.0),
                            p2p=p2p_settings(self.c))
                self.record(f"extract_{i}", position=position, slot=slot(position), side=self.c["extract_arm"],
                            configs={k: str(v) for k, v in configs.items()}, lift_m=self.c["extract_lift_m"], handover=False,
                            pregrasp_xy_position_tolerance_m=self.c.get("extract_pregrasp_xy_position_tolerance_m", 0.003),
                            pregrasp_xy_rotation_tolerance_rad=self.c.get("extract_pregrasp_xy_rotation_tolerance_rad", 0.03),
                            pregrasp_xy_settle_time_sec=self.c.get("extract_pregrasp_xy_settle_time_sec", 2.0),
                            approach_settle_time_sec=self.c.get("extract_approach_settle_time_sec", 2.0),
                            lift_settle_time_sec=self.c.get("extract_lift_settle_time_sec", 2.0),
                            position_tolerance_m=self.c.get("extract_position_tolerance_m", 0.003),
                            rotation_tolerance_rad=self.c.get("extract_rotation_tolerance_rad", 0.03),
                            lift_position_tolerance_m=self.c.get("extract_lift_position_tolerance_m", self.c.get("extract_position_tolerance_m", 0.003)),
                            lift_rotation_tolerance_rad=self.c.get("extract_lift_rotation_tolerance_rad", self.c.get("extract_rotation_tolerance_rad", 0.03)))
                self.record(f"drop_{i}", xyz=self.c["drop_xyz"], table_height_offset_m=self.c["drop_height_m"],
                            direction="right (base -Y)", lateral_range_m=self.c.get("drop_lateral_range_m", [0.12, 0.30]),
                            retract_m=0, max_release_rise_m=0.005,
                            perception_retries=self.c.get("drop_perception_retries", 1),
                            image_right_of_rack_margin_px=5,
                            settle_time_sec=self.c.get("drop_settle_time_sec", 1.0),
                            position_tolerance_m=self.c.get("drop_position_tolerance_m", 0.005),
                            rotation_tolerance_rad=self.c.get("drop_rotation_tolerance_rad", 0.035))
            return
        self.record("inventory" if self.c["demo"] == 1 else "locate_elongated_object", count=self.c["count"])
        if self.stop_after_inventory:
            return
        for i in range(1, self.c["count"] + 1):
            self.record(f"grasp_handover_{i}", pick_arm=self.c["pick_arm"],
                        p2p=self.c["p2p"],
                        handover_receiver_stable_sec=self.c.get("handover_receiver_stable_sec", 0.5),
                        handover_release_delay_sec=self.c.get("handover_release_delay_sec", 2.0),
                        handover_close_timeout_sec=self.c.get("handover_close_timeout_sec", 5.0),
                        post_transfer_home=False, receiver_home_before_insert=False,
                        handover_retreat_distance_m=self.c.get("handover_retreat_distance_m", 0.20),
                        post_transfer_retreat="donor only: left +base Y, right -base Y; keep X/Z/rotation",
                        grasp_position=self.c.get("cycles", [{}] * self.c["count"])[i-1].get("grasp_position", self.c["grasp_position"]),
                        transition_json=str(self.transition_path()), geometry="由实时 head/tail 坐标计算；不回退到 bbox_center")
            if self.c["demo"] == 1:
                cycle = self.c["cycles"][i-1]
                paths = self.hole_configs(i, cycle["insert"])
                self.record(f"insert_{i}", slot=slot(cycle["insert"]), configs={k: str(v) for k, v in paths.items()},
                            helper=str(Path(__file__).with_name("demo_1_insert.py")), holder="本次 handover 接收手")
                if self.c["extract_timing"] == "after_each_insert":
                    self.plan_extract(i)
            else:
                self.record(f"{self.c['after_handover']}_{i}", side="接收手")
        if self.c["demo"] == 1 and self.c["extract_timing"] == "after_all_inserts":
            for i in range(1, 5):
                self.plan_extract(i)

    def plan_extract(self, i):
        position = self.c["cycles"][i-1]["extract"]
        configs = self.hole_configs(i, position, occupied=True)
        self.record(f"extract_{i}", slot=slot(position), side="第 i 次插入所用手", lift_m=self.c["extract_lift_m"],
                    configs={k: str(v) for k, v in configs.items()}, handover=False)
        self.record(f"drop_{i}", xyz=self.c["drop_xyz"], table_height_offset_m=self.c["drop_height_m"])

    def run(self):
        write_json(self.out / "config_snapshot.json", self.c)
        if self.mode == "dry_run":
            self.plan()
            return
        if self.c["demo"] == 4:
            if self.mode == "live":
                required = [Path(self.locator_python), GRIPPER, Path(__file__).with_name("demo_camera_locate.py"),
                            LOCATOR / "calibration/extrinsics.yaml"]
                required += [OLD / name for name in ("tube_insertion_skill.py", "prepare_runtime_wrist_locator_config.py",
                              "grasp_right_arm_xyz.py", "place_held_object_on_table.py")]
                required += [LOCATOR / name for name in ("config_rack_center_generic_vlm.yaml",
                              "config_rack_empty_hole_left_wrist_generic_vlm.yaml", "config_rack_empty_hole_right_wrist_generic_vlm.yaml",
                              "config_empty_table_place_vlm.yaml")]
                missing = [str(p) for p in required if not p.is_file()]
                if missing:
                    raise FileNotFoundError(f"missing demo dependencies: {missing}")
            for i, position in enumerate(self.c["extract_order"], 1):
                self.record(f"extract_selection_{i}", position=position, slot=slot(position), side=self.c["extract_arm"],
                            gripper_opening_direction=self.c["gripper_opening_direction"])
                self.extract(i, self.c["extract_arm"], position, pause_before_motion=i == 1)
            return
        if self.mode == "live":
            # Fail missing dependencies before any grasp can leave an object held.
            required = [Path(self.locator_python), self.transition_path(), GRIPPER,
                        Path(__file__).with_name("demo_camera_locate.py"),
                        OLD / "grasp_right_arm_xyz.py", OLD / "place_held_object_on_table.py",
                        LOCATOR / "calibration/extrinsics.yaml",
                        LOCATOR / "config_test_tube_cleanup_leftmost_vlm_head.yaml",
                        LOCATOR / "config_empty_table_place_vlm.yaml"]
            if self.c["demo"] == 1:
                required += [OLD / name for name in ("locate_all_tubes_once.py", "run_manual_grip_to_insert.py",
                              "tube_insertion_skill.py", "prepare_runtime_wrist_locator_config.py")]
                required += [LOCATOR / name for name in ("config_rack_center_generic_vlm.yaml",
                              "config_rack_empty_hole_left_wrist_generic_vlm.yaml", "config_rack_empty_hole_right_wrist_generic_vlm.yaml")]
            missing = [str(p) for p in required if not p.is_file()]
            if missing:
                raise FileNotFoundError(f"missing demo dependencies: {missing}")
        receivers = []
        if self.c["demo"] == 1:
            inventory = self.out / "inventory.json"
            self.command("inventory", [self.locator_python, Path(__file__).with_name("demo_1_inventory.py"),
                         "--config", LOCATOR / "config_test_tube_cleanup_leftmost_vlm_head.yaml", "--rack-config", self.rack_config(),
                         "--output-dir", self.out / "tubes", "--inventory-json", inventory,
                         "--rack-result-json", self.out / "initial_rack.json", "--save-rgb", self.out / "inventory_rgb.jpg"])
            if self.mode == "mock":
                sources = [self.locate(f"mock_tube_{i}", "mock_config", mock_key="head") for i in range(1, 5)]
            else:
                data = read_json(inventory)
                if not data.get("ok") or data.get("rejected_candidates") or len(data.get("tubes", [])) != 4:
                    raise ValueError("必须完整识别到四个散落 vial，且无被拒绝候选；未开始运动")
                sources = [Path(t["result_json"]) for t in sorted(data["tubes"], key=lambda t: t["bbox_center_px"]["x"])]
            # Validate every cached grasp BEFORE the first physical motion.
            for i, source in enumerate(sources):
                grasp_geometry(read_json(source), self.c["cycles"][i].get("grasp_position", self.c["grasp_position"]), self.c["pick_arm"])
            if self.stop_after_inventory:
                self.record("inventory_verified", count=len(sources), motion_started=False)
                return
            self.pause()
            for i, source in enumerate(sources, 1):
                cycle = self.c["cycles"][i-1]
                receiver = self.grasp(source, cycle.get("grasp_position", self.c["grasp_position"]), i)
                receivers.append(receiver)
                self.insert(i, receiver, cycle["insert"])
                if self.c["extract_timing"] == "after_each_insert":
                    self.extract(i, receiver, cycle["extract"])
            if self.c["extract_timing"] == "after_all_inserts":
                for i, receiver in enumerate(receivers, 1):
                    self.extract(i, receiver, self.c["cycles"][i-1]["extract"])
        else:
            conf = self.locator_config("elongated_object", "config_test_tube_cleanup_leftmost_vlm_head.yaml",
                self.c["object_description"] + " Return a tight whole-object box and head_px / tail_px at the two endpoints. "
                "head_px defines the FRONT and tail_px the TAIL. " + self.c["front_definition"],
                object_name="loose elongated object", refine=False)
            for i in range(1, self.c["count"] + 1):
                source = self.locate(f"object_{i}", conf, mock_key="head")
                grasp_geometry(read_json(source), self.c["grasp_position"], self.c["pick_arm"])
                if i == 1:
                    self.pause()
                receiver = self.grasp(source, self.c["grasp_position"], i)
                if self.c["after_handover"] == "drop":
                    self.drop(i, receiver)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config", type=Path, required=True)
    parser.add_argument("--mode", choices=("dry_run", "mock", "live"), default="dry_run")
    parser.add_argument("--dry-run", dest="mode", action="store_const", const="dry_run")
    parser.add_argument("--mock", dest="mode", action="store_const", const="mock")
    parser.add_argument("--execute", action="store_true")
    parser.add_argument("--artifact-dir", type=Path)
    parser.add_argument("--no-wait-before-motion", action="store_true")
    parser.add_argument("--stop-after-inventory", action="store_true",
                        help="Demo 1: capture and validate all four vials, then exit before any robot command.")
    args = parser.parse_args(argv)
    if args.execute != (args.mode == "live"):
        parser.error("live 必须同时指定 --mode live --execute；离线模式不能指定 --execute")
    config = yaml.safe_load(args.config.read_text(encoding="utf-8"))
    validate(config)
    if args.stop_after_inventory and config["demo"] != 1:
        parser.error("--stop-after-inventory 仅适用于 Demo 1")
    artifacts = (args.artifact_dir or Path("/tmp/agentic_skills_runs") / f"{DEMO_NAMES[config['demo']]}_{datetime.now():%Y%m%d_%H%M%S_%f}").resolve()
    if args.mode == "live" and artifacts.exists() and any(artifacts.iterdir()):
        parser.error("live artifact-dir 必须是新的空目录，避免读取前一次运行结果")
    demo = Demo(config, args.config.resolve(), artifacts, args.mode, not args.no_wait_before_motion,
                stop_after_inventory=args.stop_after_inventory)
    try:
        demo.run()
    except (Exception, KeyboardInterrupt) as exc:
        write_json(artifacts / "task_result.json", {"completion_flag": False, "physical_verified": False,
                   "mode": args.mode, "robot_stop_confirmed": False,
                   "stopped_reason": str(exc) or "interrupted", "last_stage": demo.events[-1] if demo.events else None})
        print(f"ERROR: {str(exc) or 'interrupted'}; no further commands; robot stop unconfirmed. "
              f"必要时使用现场急停。Artifacts: {artifacts}", file=sys.stderr)
        return 1
    write_json(artifacts / "task_result.json", {"completion_flag": args.mode != "dry_run" and not args.stop_after_inventory, "physical_verified": False,
               "inventory_only": args.stop_after_inventory,
               "inventory_complete": args.stop_after_inventory and args.mode != "dry_run",
               "mode": args.mode, "planned_only": args.mode == "dry_run", "mock": args.mode == "mock",
               "held_at_end": config["demo"] == 2 and config["after_handover"] == "hold"})
    print(f"Artifacts: {artifacts}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
