"""Only camera capture failures may restart perception; never replay motion/inference."""
import contextlib
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import Mock, patch

ROOT=Path(__file__).resolve().parents[1]
SCRIPTS=ROOT/'task_skills/pick_tube_insert_rack/skills/task-pick-tube-insert-rack/scripts'
sys.path.insert(0,str(SCRIPTS))
import tube_insertion_skill as flow

BUSY='object-locator failed: ERROR: get_xu(ctrl=1) failed! Last Error: Device or resource busy'
TIMEOUT='object-locator failed: ERROR: set_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed: Connection timed out'

class CameraRecoveryTests(unittest.TestCase):
    def pending(self, signal):
        return SimpleNamespace(process=Mock(poll=Mock(return_value=0)), capture_ready_file=signal)

    def test_timeout_before_capture_recovers_with_same_config_and_output_args(self):
        with tempfile.TemporaryDirectory() as folder:
            config=Path(folder)/'head.yaml';first=self.pending(Path(folder)/'ready');second=self.pending(Path(folder)/'ready2')
            order=[]
            with patch.object(flow,'start_object_locator_after_capture_signal',return_value=first), \
                 patch.object(flow,'wait_for_object_locator_capture',side_effect=RuntimeError(TIMEOUT)), \
                 patch.object(flow,'cancel_pending_object_locator',side_effect=lambda p:order.append('cancel')), \
                 patch.object(flow,'recover_object_locator_after_camera_reset',side_effect=lambda *a,**k:(order.append('recover') or (second,{'wait_sec':1}))) as recovery, \
                 patch.object(flow,'finish_pending_object_locator',return_value={'found':True}),contextlib.redirect_stderr(io.StringIO()):
                result=flow.run_object_locator(Path(folder),config,extra_args=['--result-json','new.json'])
            self.assertEqual(order,['cancel','recover'])
            self.assertEqual(recovery.call_args.args[:2],(Path(folder),config))
            self.assertEqual(recovery.call_args.kwargs['extra_args'],['--result-json','new.json'])
            self.assertTrue(result['task_timings_sec']['reset_attempted'])

    def test_inference_failure_never_resets_or_repeats(self):
        pending=self.pending(Path('/tmp/no-file-camera-test'))
        error=RuntimeError('OpenRouter HTTP 402')
        with patch.object(flow,'start_object_locator_after_capture_signal',return_value=pending) as start, \
             patch.object(flow,'wait_for_object_locator_capture',return_value={'wait_sec':1}), \
             patch.object(flow,'finish_pending_object_locator',side_effect=error) as finish, \
             patch.object(flow,'recover_object_locator_after_camera_reset') as reset:
            with self.assertRaises(RuntimeError) as caught:flow.run_object_locator(Path('/tmp'),Path('/tmp/head.yaml'))
            self.assertIs(caught.exception,error)
            start.assert_called_once();finish.assert_called_once();reset.assert_not_called()

    def test_only_known_capture_errors_are_recoverable(self):
        for message in (TIMEOUT,'set_xu(...). xioctl(UVCIOC_CTRL_QUERY) failed: Protocol error','RealSense frame did not arrive within 5000 ms'):
            self.assertTrue(flow._recoverable_camera_capture_error(RuntimeError(message)))
        for message in (BUSY,'OpenRouter HTTP 402','invalid stream configuration','unsupported visual_preset','invalid capture-ready file',
                        'RealSense camera is already in use', 'RealSense reset cooldown active', 'Failed to release RealSense pipeline'):
            self.assertFalse(flow._recoverable_camera_capture_error(RuntimeError(message)))

    def test_failed_reset_capture_is_bounded_and_all_children_are_closed(self):
        pending=self.pending(Path('/tmp/no-file-camera-test'))
        with patch.dict(flow.os.environ,{'REALSENSE_RESET_MAX_ATTEMPTS':'3','REALSENSE_RESET_COOLDOWN_SEC':'0.1'}), \
             patch.object(flow.time,'sleep'), \
             patch.object(flow,'start_object_locator_after_capture_signal',return_value=pending) as start, \
             patch.object(flow,'wait_for_object_locator_capture',side_effect=RuntimeError(TIMEOUT)), \
             patch.object(flow,'cancel_pending_object_locator') as cancel,contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'after 3 forced reset'):
                flow.recover_object_locator_after_camera_reset(Path('/tmp'),Path('/tmp/head.yaml'),Path('/tmp/ready'))
            self.assertEqual(start.call_count,4);self.assertEqual(cancel.call_count,4)
            self.assertEqual(start.call_args_list[0].kwargs['extra_args'],[])
            self.assertEqual(start.call_args.kwargs['extra_args'],['--reset-realsense'])

    def test_clean_reopen_success_does_not_hardware_reset(self):
        pending=self.pending(Path('/tmp/no-file-camera-test'))
        with patch.object(flow.time,'sleep'), \
             patch.object(flow,'start_object_locator_after_capture_signal',return_value=pending) as start, \
             patch.object(flow,'wait_for_object_locator_capture',return_value={'wait_sec':1}),contextlib.redirect_stderr(io.StringIO()):
            _, status=flow.recover_object_locator_after_camera_reset(Path('/tmp'),Path('/tmp/head.yaml'),Path('/tmp/ready'))
        start.assert_called_once()
        self.assertNotIn('--reset-realsense',start.call_args.kwargs['extra_args'])
        self.assertFalse(status['reset_attempted'])

    def test_lock_conflict_during_recovery_does_not_reset_owner(self):
        pending=self.pending(Path('/tmp/no-file-camera-test'))
        with patch.object(flow.time,'sleep'), \
             patch.object(flow,'start_object_locator_after_capture_signal',return_value=pending) as start, \
             patch.object(flow,'wait_for_object_locator_capture',side_effect=RuntimeError('RealSense camera is already in use')), \
             patch.object(flow,'cancel_pending_object_locator'),contextlib.redirect_stderr(io.StringIO()):
            with self.assertRaisesRegex(RuntimeError,'already in use'):
                flow.recover_object_locator_after_camera_reset(Path('/tmp'),Path('/tmp/head.yaml'),Path('/tmp/ready'))
        start.assert_called_once()

if __name__=='__main__':unittest.main()
