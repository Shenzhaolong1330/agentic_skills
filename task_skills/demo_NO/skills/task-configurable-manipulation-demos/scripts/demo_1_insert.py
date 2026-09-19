#!/usr/bin/env python3
"""Reuse the original insertion flow with a current-observation wrist calibration.

The original flow owns motion, force checks and completion handling. The adapter
keeps the donor at its post-handover retreat pose, binds the current wrist calibration,
and saves the final report separately from human-readable stdout diagnostics.
"""
import argparse
import json
from pathlib import Path
import subprocess
import sys

ROOT = Path(__file__).resolve().parents[5]
OLD = ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"


def option(argv, flag):
    return argv[argv.index(flag) + 1]


def calibrated_dispatch(original):
    observation_dir = None

    def run_stage(command):
        nonlocal observation_dir
        if command.name == "observe_rack":
            observation_dir = Path(option(command.argv, "--artifact-dir")) / "object_locator_runtime"
            # The donor has already retreated. Do not home it here;
            # the legacy home path also used to reset/reactivate the grippers.
            command = type(command)(name=command.name, argv=[*command.argv, "--no-yield-non-holder-arm"])
        if command.name == "move_above_hole_from_wrist" and "--execute" in command.argv:
            if observation_dir is None:
                raise RuntimeError("wrist localization requires a preceding observation stage")
            destination = Path(option(command.argv, "--artifact-dir")) / "current_wrist_hole.yaml"
            source = option(command.argv, "--hole-config")
            subprocess.run([sys.executable, str(OLD / "prepare_runtime_wrist_locator_config.py"),
                            "--source-config", source, "--runtime-calibration-dir", str(observation_dir),
                            "--output-config", str(destination)], check=True, stdout=subprocess.PIPE, text=True)
            argv = list(command.argv)
            argv[argv.index("--hole-config") + 1] = str(destination)
            command = type(command)(name=command.name, argv=argv)
        return original(command)

    return run_stage


def parse_adapter_args(argv=None):
    parser = argparse.ArgumentParser(add_help=False, allow_abbrev=False)
    parser.add_argument("--output-json", type=Path)
    return parser.parse_known_args(argv)


def main(argv=None):
    args, flow_argv = parse_adapter_args(argv)
    if args.output_json is not None:
        if args.output_json.exists():
            raise FileExistsError(f"Refusing to reuse insertion report: {args.output_json}")
        args.output_json.parent.mkdir(parents=True, exist_ok=True)
    sys.path.insert(0, str(OLD))
    import run_manual_grip_to_insert as flow
    original = flow.run_stage
    original_print_report = flow.print_report

    def print_report(report, *, compact):
        if args.output_json is not None:
            temporary = args.output_json.with_name(args.output_json.name + ".tmp")
            temporary.write_text(json.dumps(report, ensure_ascii=False, indent=2, default=str) + "\n", encoding="utf-8")
            temporary.replace(args.output_json)
        original_print_report(report, compact=compact)

    flow.run_stage = calibrated_dispatch(original)
    flow.print_report = print_report
    try:
        return flow.main(flow_argv)
    finally:
        flow.run_stage = original
        flow.print_report = original_print_report


if __name__ == "__main__":
    raise SystemExit(main())
