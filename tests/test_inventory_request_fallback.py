"""Inventory HTTP fallback tests without cameras, networks or robot clients."""
import copy
from pathlib import Path
import sys
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts"))
import locate_all_tubes_once as inventory


class InventoryFallbackTests(unittest.TestCase):
    def setUp(self):
        self.client = inventory.OpenRouterVLMClient(api_key="sk-or-test-key", model="test-model")
        self.config = SimpleNamespace(target=SimpleNamespace(name="vial", description="loose vial"),
            openrouter=SimpleNamespace(model="test-model", timeout_s=1.0, temperature=0,
                jpeg_quality=90, max_tokens=4096, use_json_schema=True, retry_without_json_schema=True))
        self.success = {"choices": [{"message": {"content": '{"tubes":[]}'}, "finish_reason": "stop"}]}

    def detect(self):
        return inventory.detect_all_with_vlm(np.zeros((20, 20, 3), dtype=np.uint8), self.config,
            min_confidence=0.25, nms_iou_threshold=0.45, max_tubes=4, raw_response_path=None)

    def test_bad_schema_fallback_keeps_image_and_explicit_json_contract(self):
        calls = []
        def post(payload):
            calls.append(copy.deepcopy(payload))
            if len(calls) == 1:
                raise inventory.OpenRouterError("HTTP 400 invalid argument", status_code=400)
            return self.success
        with patch.object(inventory.OpenRouterVLMClient, "from_env", return_value=self.client), \
                patch.object(self.client, "_post_payload", side_effect=post):
            self.assertEqual(self.detect(), [])
        self.assertIn("response_format", calls[0])
        self.assertNotIn("response_format", calls[1])
        self.assertEqual(calls[0]["messages"], calls[1]["messages"])
        self.assertIn('"tubes":[]', calls[1]["messages"][1]["content"][0]["text"])

    def test_auth_credits_and_network_errors_do_not_trigger_schema_fallback(self):
        for code in (None, 401, 402):
            with self.subTest(code=code), patch.object(inventory.OpenRouterVLMClient, "from_env", return_value=self.client), \
                    patch.object(self.client, "_post_payload", side_effect=inventory.OpenRouterError("test error", status_code=code)) as post:
                with self.assertRaises(inventory.OpenRouterError):
                    self.detect()
                self.assertEqual(post.call_count, 1)

    def test_fallback_error_preserves_both_causes(self):
        errors = [inventory.OpenRouterError("HTTP 400 invalid argument", status_code=400),
                  inventory.OpenRouterError("TLS SSLEOFError after 3 attempts")]
        with patch.object(inventory.OpenRouterVLMClient, "from_env", return_value=self.client), \
                patch.object(self.client, "_post_payload", side_effect=errors):
            with self.assertRaisesRegex(inventory.OpenRouterError, "SSLEOFError.*HTTP 400"):
                self.detect()

    def test_json_schema_setting_is_respected(self):
        self.config.openrouter.use_json_schema = False
        with patch.object(inventory.OpenRouterVLMClient, "from_env", return_value=self.client), \
                patch.object(self.client, "_post_payload", return_value=self.success) as post:
            self.detect()
        self.assertNotIn("response_format", post.call_args.args[0])


if __name__ == "__main__":
    unittest.main()
