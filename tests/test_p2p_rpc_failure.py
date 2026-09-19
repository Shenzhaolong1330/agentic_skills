"""Lost replies after an applied motion must never cause a replay. No hardware."""
import importlib.util
import io
import json
import struct
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[1]
PATH = ROOT / "atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/dual_franka_robotiq_rpc_client.py"
spec = importlib.util.spec_from_file_location("p2p_rpc_failure_test", PATH)
rpc = importlib.util.module_from_spec(spec)
spec.loader.exec_module(rpc)


class LostRemote(Exception):
    pass


class AppliedThenLostClient(rpc.DualFrankaRobotiqRpcClient):
    def __init__(self, fail_at):
        self.fail_at = fail_at
        self.applied = 0
        self.error = LostRemote("Lost remote after 10s heartbeat")

    def get_observation(self):
        return {side: {"end_pose": [0.] * 6} for side in ("left_arm", "right_arm")}

    def step(self, action=None):
        if action is None:
            return {"observation": self.get_observation()}
        self.applied += 1
        if self.applied == self.fail_at:
            raise self.error  # The simulated server applied it, but its reply was lost.
        return {"step_count": 100 + self.applied}


class RpcFailureTests(unittest.TestCase):
    def test_connection_monitor_records_disconnect_without_consuming_rpc_reply(self):
        import zmq
        class Monitor:
            def __init__(self):
                self.parts = [
                    [struct.pack('=hi', event, 17), b'tcp://test:4242']
                    for event in (zmq.EVENT_DISCONNECTED, zmq.EVENT_CONNECT_RETRIED)
                ]
            def recv_multipart(self, flags):
                self.flags = flags
                if not self.parts:
                    raise zmq.Again()
                return self.parts.pop(0)
            def close(self):
                self.closed = True
        monitor = Monitor()
        calls = []
        socket = SimpleNamespace(get_monitor_socket=lambda: monitor,
                                 disable_monitor=lambda: calls.append('disable'))
        events = SimpleNamespace(_socket=socket,
                                 emit_event=lambda event, timeout=None: calls.append('emit'),
                                 recv=lambda timeout=None: calls.append('recv'))
        trace = rpc._RpcEventTrace(SimpleNamespace(_events=events))
        trace.drain_monitor()
        self.assertEqual([e['event'] for e in trace.connections],
                         ['DISCONNECTED', 'CONNECT_RETRIED'])
        self.assertEqual(monitor.flags, zmq.NOBLOCK)
        self.assertEqual(calls, [])
        trace.close()
        self.assertTrue(monitor.closed)
        self.assertEqual(calls, ['disable'])

    def test_event_trace_is_bounded_and_does_not_record_payloads(self):
        events = SimpleNamespace(emit_event=lambda event, timeout=None: None, recv=lambda timeout=None: None)
        trace = rpc._RpcEventTrace(SimpleNamespace(_events=events))
        event = SimpleNamespace(name='step', header={'message_id': b'abc'}, args=['private payload'])
        for _ in range(100):
            events.emit_event(event)
        self.assertEqual(len(trace.records), 64)
        self.assertNotIn('private payload', json.dumps(list(trace.records)))

    def test_timeout_keeps_original_exception_and_does_not_retry_when_diagnostics_fail(self):
        calls = []
        error = LostRemote('Lost remote after 10s heartbeat')
        def step(action):
            calls.append(action)
            raise error
        def broken_snapshot():
            raise RuntimeError('diagnostic failure')
        client = object.__new__(rpc.DualFrankaRobotiqRpcClient)
        client._client = SimpleNamespace(step=step, _motion_event_trace=SimpleNamespace(snapshot=broken_snapshot))
        with self.assertRaises(LostRemote) as caught:
            client.step({'example': True})
        self.assertIs(caught.exception, error)
        self.assertEqual(len(calls), 1)

    def test_real_zerorpc_echo_and_timeout_trace_without_robot(self):
        import gevent
        import zerorpc
        class Echo:
            def ping(self):
                return {'ok': True}
            def delay(self):
                gevent.sleep(1.)
                return {'ok': True}
        with tempfile.TemporaryDirectory() as temp:
            endpoint = 'inproc://rpc-trace-' + Path(temp).name
            server = zerorpc.Server(Echo())
            server.bind(endpoint)
            job = gevent.spawn(server.run)
            client = rpc.DualFrankaRobotiqRpcClient(server=endpoint, timeout=.05)
            try:
                self.assertEqual(client.ping(), {'ok': True})
                output = io.StringIO()
                with redirect_stderr(output), self.assertRaises(zerorpc.exceptions.TimeoutExpired):
                    client._call('delay')
                report = json.loads(output.getvalue().removeprefix('zerorpc_transport_failure: '))
                self.assertEqual(report['method'], 'delay')
                self.assertTrue(any(e['direction'] == 'received' and e['event'] == 'OK' for e in report['events']))
                self.assertIn('receiver', report['workers'])
            finally:
                client.close()
                server.stop()
                server.close()
                job.kill()

    def test_initial_and_correction_failures_preserve_exception_without_replay(self):
        for fail_at, correction, waypoint in ((1, 0, 1), (2, 0, 2), (3, 1, 1)):
            with self.subTest(fail_at=fail_at):
                client = AppliedThenLostClient(fail_at)
                trajectory = [dict(index=i, left_delta=[0.] * 6, right_delta=[0.] * 6) for i in (1, 2)]
                def plan(*args, **kwargs):
                    return trajectory, dict(steps=2, duration_sec=1., period_sec=.5)
                output = io.StringIO()
                with patch.object(rpc, "_plan_smooth_absolute_trajectory", side_effect=plan), redirect_stderr(output):
                    with self.assertRaises(LostRemote) as caught:
                        client.dual_robot_move_to_ee_pose(
                            [.1, 0., 0., 0., 0., 0.], [0.] * 6,
                            delta=False, smooth=True, sleep=False, max_correction_iters=1,
                        )
                self.assertIs(caught.exception, client.error)
                self.assertEqual(client.applied, fail_at)
                record = json.loads(next(line.removeprefix("p2p_rpc_failure: ") for line in output.getvalue().splitlines() if line.startswith("p2p_rpc_failure: ")))
                self.assertEqual(record["correction_index"], correction)
                self.assertEqual(record["waypoint"], waypoint)
                self.assertEqual(record["last_confirmed_step_count"], 100 + fail_at - 1 if fail_at > 1 else None)
                self.assertEqual(record["motion_outcome"], "unconfirmed")


if __name__ == "__main__":
    unittest.main()
