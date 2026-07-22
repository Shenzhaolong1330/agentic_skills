from __future__ import annotations

import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


ROOT = Path(__file__).resolve().parents[1]


class CompileTaskGraphCliTests(unittest.TestCase):
    def test_cli_writes_static_compiled_plan(self):
        with tempfile.TemporaryDirectory() as directory:
            result = subprocess.run([sys.executable, str(ROOT / "scripts/compile_task_graph.py"), "--goal", str(ROOT / "examples/planning/observe_object.goal.json"), "--envelope", str(ROOT / "examples/planning/dry_run.envelope.json"), "--graph", str(ROOT / "examples/planning/observe_object.graph.json"), "--manifest", str(ROOT / "skill_manifest.json"), "--output-dir", directory], cwd=ROOT, capture_output=True, text=True, check=False)
            self.assertEqual(result.returncode, 0, result.stderr)
            compiled = json.loads((Path(directory) / "compiled_task_graph.json").read_text())
            self.assertNotIn("executable", str(compiled))
            self.assertNotIn("argv", str(compiled))

