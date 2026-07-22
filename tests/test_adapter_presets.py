from __future__ import annotations

from pathlib import Path
import unittest

from agentic_skills_harness.dispatch.adapters.presets import default_preset_catalog


class AdapterPresetTests(unittest.TestCase):
    def test_known_preset_is_fixed(self):
        catalog = default_preset_catalog()
        preset = catalog.require("handover.transition.v1", "procedure.handover_transition")
        self.assertFalse(Path(preset.repo_relative_config_path).is_absolute())
        with self.assertRaises(ValueError):
            catalog.require("unknown.preset")

    def test_preset_cannot_escape(self):
        with self.assertRaises(ValueError):
            from agentic_skills_harness.dispatch.adapters.presets import PresetSpec
            PresetSpec("bad", "../outside.json", "x.y", "bad")
