"""Delayed, cross-coupled synthetic plant; NOT a fitted robot model or live proof."""
import io
import os
import unittest
from contextlib import redirect_stderr
from unittest.mock import patch

from test_external_tracking import adapter, g, Clock, FakeClient


class LagPlant(FakeClient):
    def __init__(self, clock, tau):
        super().__init__(clock)
        self.tau = tau
        self.physical = {s: adapter.base._pose6(self.measured[s]) for s in adapter.SIDES}
        self.last = clock.now
        self.joints = {s: [0.] * 7 for s in adapter.SIDES}

    def get_observation(self):
        dt = self.clock.now - self.last
        if dt > 0:
            for side in adapter.SIDES:
                target = adapter.base._pose6(adapter.base.legacy._pose_from_delta(
                    self.saved[side], [-v for v in self.bias[side]]))
                if side == 'left_arm':
                    target = g.Pose(target.position_m, g._integrate(target.quaternion_xyzw,
                        (0., 0., -self.physical[side].position_m[0]), True))
                error = g._error(self.physical[side], target)
                fraction = min(1., dt / self.tau)
                delta = g._scale(error['translation_vector_m'], fraction)
                rotation = g._scale(error['rotation_vector_rad'], fraction)
                self.physical[side] = g.Pose(g._add(self.physical[side].position_m, delta),
                    g._integrate(self.physical[side].quaternion_xyzw, rotation, True))
                # Synthetic velocity proxy drives arrival stationarity checks.
                # It is not the robot's Jacobian or a hardware speed model.
                self.joints[side] = [(3. * g._norm(delta) + g._norm(rotation)) / dt] + [0.] * 6
            self.last = self.clock.now
        self.measured = {s: adapter.base._six(p) for s, p in self.physical.items()}
        observation = super().get_observation()
        for side in adapter.SIDES:
            observation[side]['native_robot_state']['joint_velocities'] = self.joints[side]
        return observation


class LaggedTrackingTests(unittest.TestCase):
    def test_delay_and_cross_coupling_require_measured_arrival_not_five_second_timeout(self):
        for tau in (.1, .2, .4):
            with self.subTest(tau=tau):
                clock = Clock()
                client = LagPlant(clock, tau)
                with patch.object(adapter.base, 'time', clock), patch.dict(os.environ, clear=True), \
                     patch('socket.socket', side_effect=AssertionError('No hardware')), redirect_stderr(io.StringIO()):
                    os.environ['FRANKA_P2P_TRACKING_PROFILE'] = 'fast_tracking'
                    result = adapter.run(client, [.15, 0., 0., 0., 0., 0.], [0.] * 6,
                        max_translation_speed=.1, max_translation_step=.005, max_tracking_translation_bias_m=None,
                        position_tolerance_m=.005, rotation_tolerance_rad=.2)
                self.assertTrue(result['ok'])
                self.assertLessEqual(result['final_error']['max_translation_error_m'], .003)
                self.assertLessEqual(result['final_error']['max_rotation_error_rad'], .1)
                self.assertGreater(result['performance']['actual_call_s'], 5.)
                self.assertFalse(result['performance']['met_target'])
                self.assertGreaterEqual(result['hold_confirmation'][-1]['monotonic_s'] -
                                        result['hold_confirmation'][0]['monotonic_s'], .5)


if __name__ == '__main__':
    unittest.main()
