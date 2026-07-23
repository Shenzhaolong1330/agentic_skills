#!/usr/bin/env python3
"""Generate the local S10H config template; it does not contact hardware."""

from __future__ import annotations

import argparse
from pathlib import Path


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, default=Path.cwd())
    parser.add_argument("--output", type=Path)
    args = parser.parse_args(argv)
    repo = args.repo_root.resolve()
    source = repo / "config/templates/s10_hardware_acceptance.example.yaml"
    target = args.output.resolve() if args.output else repo / "config/local/s10_hardware_acceptance.yaml"
    if target != repo / "config/local/s10_hardware_acceptance.yaml" and target != source:
        raise SystemExit("output must be config/local/s10_hardware_acceptance.yaml or the tracked example template")
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(source.read_text(encoding="utf-8"), encoding="utf-8")
    print(target)
    return 0


if __name__ == "__main__": raise SystemExit(main())
