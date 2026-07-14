---
name: atomic-state-realsense-viewer
description: View the current dual-Franka state and capture RGB-D images from the head, left wrist, and right wrist RealSense cameras for quick inspection and debugging.
---

# Atomic State RealSense Viewer

Use this skill when the user asks to inspect the current robot state, current observation, or RealSense camera images on the dual-Franka setup.

Bundled script:

```bash
/home/deepcybo/agentic_skills/atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/scripts/view_state_realsense.py
```

Default RPC:

- `FRANKA_RPC_HOST` or `--server-host`, default `172.16.0.1`
- `FRANKA_RPC_PORT` or `--server-port`, default `4242`
- state source: existing `DualFrankaRobotiqRpcClient.get_full_state()`

Default RealSense cameras:

- `head`: `348522072761`
- `left_wrist`: `347622074336`
- `right_wrist`: `337322072568`

These serial numbers match the object locator configs under:

```bash
/home/deepcybo/agentic_skills/atomic_skills/object_locator/config_*grounded_sam*.yaml
```

## Common Commands

Capture current state and all three RealSense views:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/scripts/view_state_realsense.py
```

Capture only one camera:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/scripts/view_state_realsense.py \
  --camera right_wrist
```

Print the full state JSON to stdout in addition to saving it:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/scripts/view_state_realsense.py \
  --no-images \
  --print-state
```

List connected RealSense devices:

```bash
python3 /home/deepcybo/agentic_skills/atomic_skills/state_realsense_viewer/skills/atomic-state-realsense-viewer/scripts/view_state_realsense.py \
  --list-devices \
  --no-state \
  --no-images
```

## Output

Default output directory:

```bash
/home/deepcybo/agentic_skills/atomic_skills/state_realsense_viewer/runs/latest
```

Each run writes:

- `state.json` when state capture is enabled
- `<camera>_rgb.jpg`
- `<camera>_depth.jpg`
- `<camera>_panel.jpg`
- `summary.json`

Use `--output-dir` to write elsewhere. Use `--timestamp-dir` to create a unique timestamped subdirectory instead of overwriting `runs/latest`.

## Notes

- This skill is read-only for robot state and cameras. It does not command robot motion or grippers.
- If a RealSense pipeline times out, check whether another process is already using that serial number.
- If `pyrealsense2` or `cv2` is missing, install the object locator project dependencies.
