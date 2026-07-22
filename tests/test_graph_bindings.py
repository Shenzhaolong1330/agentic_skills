from __future__ import annotations

import unittest

from agentic_skills_harness.planning import InputBinding


class GraphBindingTests(unittest.TestCase):
    def test_binding_sources_are_exclusive_and_pointer_based(self):
        binding = InputBinding("b", "/xyz_m", "WORLD_FACT", world_fact_selector={"predicate": "object.pose"}, constraints={"workspace_id": "w", "runtime_check": True})
        self.assertEqual(binding.source_type.value, "WORLD_FACT")
        with self.assertRaises(Exception):
            InputBinding("bad", "xyz_m", "LITERAL", literal=[0, 0, 0])
        with self.assertRaises(Exception):
            InputBinding("bad", "/x", "NODE_OUTPUT", source_node_id="n", source_output_path="output.value")

