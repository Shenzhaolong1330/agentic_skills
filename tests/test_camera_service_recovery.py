"""Persistent service releases before backoff and stops resetting a failed USB device."""
import contextlib
import importlib.util
import io
from pathlib import Path
import sys
import unittest
from unittest.mock import patch

PATH = Path(__file__).resolve().parents[1] / 'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts/wrist_camera_service.py'
spec = importlib.util.spec_from_file_location('camera_service_recovery_test', PATH)
service = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = service
spec.loader.exec_module(service)


class ServiceTests(unittest.TestCase):
    def run_failures(self, error):
        events, resets = [], []
        class Stop:
            waits = 0
            def is_set(self): return self.waits >= 6
            def wait(self, seconds):
                self.waits += 1
                events.append('wait')
        class Camera:
            def __init__(self, **kwargs): resets.append(kwargs['reset_on_start'])
            def start(self):
                events.append('start')
                raise error
            def stop(self): events.append('stop')
        state = service.CameraState('head', 'head')
        with patch.object(service, '_serial_is_available', return_value=True), \
             patch.object(service, 'RealSenseCamera', Camera), contextlib.redirect_stderr(io.StringIO()):
            service._camera_worker(state, Stop(), width=640, height=480, fps=30, warmup_frames=30, frame_timeout_ms=5000)
        return state, events, resets

    def test_persistent_fault_resets_once_then_waits_for_reconnect(self):
        state, events, resets = self.run_failures(RuntimeError('Protocol error'))
        self.assertEqual(resets, [False, False, True])
        self.assertEqual(state.status, 'reconnect_required')
        for index, event in enumerate(events):
            if event == 'start':
                self.assertEqual(events[index+1], 'stop')

    def test_other_owner_is_never_reset(self):
        for error in (service.CameraInUseError('already in use'), RuntimeError('get_xu: Device or resource busy')):
            state, events, resets = self.run_failures(error)
            self.assertFalse(any(resets))
            self.assertEqual(state.status, 'waiting_for_owner')

    def test_configuration_error_is_not_retried_or_reset(self):
        state, events, resets = self.run_failures(RuntimeError('unsupported visual_preset'))
        self.assertEqual(resets, [False])
        self.assertEqual(state.status, 'capture_error_requires_attention')


if __name__ == '__main__':
    unittest.main()
