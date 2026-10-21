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
    def test_normalized_endpoints_do_not_shift_box_to_empty_table(self):
        raw = {'box_2d': {'y1':177, 'x1':288, 'y2':262, 'x2':362},
               'orientation': {'head_1000': {'x':349, 'y':242},
                               'tail_1000': {'x':301, 'y':198}}, 'confidence': .99}
        result = inventory.inventory_detection_from_normalized(raw, 640, 480, 'vial')
        self.assertAlmostEqual(result.bbox.x_min, 184.32)
        self.assertAlmostEqual(result.bbox.y_min, 84.96)
        self.assertAlmostEqual(result.head_px.x, 223.36)
        self.assertAlmostEqual(result.head_px.y, 116.16)
        self.assertAlmostEqual(result.tail_px.x, 192.64)
        self.assertAlmostEqual(result.tail_px.y, 95.04)

    def test_mixed_units_and_invalid_normalized_points_rejected(self):
        base = {'box_2d': {'y1':177, 'x1':288, 'y2':262, 'x2':362},
                'orientation': {'head_1000': {'x':349, 'y':242},
                                'tail_1000': {'x':301, 'y':198}}}
        for point in ({'x':223, 'y':116}, {'x':1001, 'y':242}, {'x':float('nan'), 'y':242}):
            raw = copy.deepcopy(base)
            raw['orientation']['head_1000'] = point
            with self.assertRaises(ValueError):
                inventory.inventory_detection_from_normalized(raw, 640, 480, 'vial')
        raw = copy.deepcopy(base)
        raw['orientation']['head_px'] = raw['orientation'].pop('head_1000')
        with self.assertRaisesRegex(ValueError, 'missing normalized'):
            inventory.inventory_detection_from_normalized(raw, 640, 480, 'vial')

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

    def test_length_retry_keeps_schema_and_image_and_increases_budget(self):
        calls = []
        def post(payload):
            calls.append(copy.deepcopy(payload))
            if len(calls) == 1:
                # Even parseable content is unacceptable when explicitly truncated.
                return {'choices': [{'message': {'content': '{"tubes":[]}'}, 'finish_reason': 'length'}]}
            return self.success
        with patch.object(inventory.OpenRouterVLMClient, 'from_env', return_value=self.client), \
                patch.object(self.client, '_post_payload', side_effect=post):
            self.assertEqual(self.detect(), [])
        self.assertEqual(len(calls), 2)
        self.assertEqual(calls[1]['max_tokens'], 8192)
        self.assertEqual(calls[1]['reasoning'], {'effort': 'low'})
        self.assertEqual(calls[0]['messages'], calls[1]['messages'])
        self.assertEqual(calls[0]['response_format'], calls[1]['response_format'])

    def test_invalid_fallback_json_retries_once(self):
        invalid = {'choices': [{'message': {'content': '```json\n{"tubes":['}, 'finish_reason': 'stop'}]}
        with patch.object(inventory.OpenRouterVLMClient, 'from_env', return_value=self.client), \
                patch.object(self.client, '_post_payload', side_effect=[
                    inventory.OpenRouterError('invalid schema', status_code=400), invalid, self.success]) as post:
            self.assertEqual(self.detect(), [])
            self.assertEqual(post.call_count, 3)

    def test_repeated_truncation_stops(self):
        truncated = {'choices': [{'message': {'content': '{"tubes":[]}'}, 'finish_reason': 'length'}]}
        with patch.object(inventory.OpenRouterVLMClient, 'from_env', return_value=self.client), \
                patch.object(self.client, '_post_payload', return_value=truncated) as post:
            with self.assertRaisesRegex(ValueError, 'after one same-image retry.*length'):
                self.detect()
            self.assertEqual(post.call_count, 2)


if __name__ == "__main__":
    unittest.main()
