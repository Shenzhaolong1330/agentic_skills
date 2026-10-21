"""Recorded projection regression and synthetic plant checks; no hardware I/O."""
import json
import math
from pathlib import Path
import random
import unittest
from unittest.mock import patch
from test_external_tracking import adapter, g
from dataclasses import replace

class RotationCapTests(unittest.TestCase):
    def setUp(self):
        for target in ('socket.socket', 'subprocess.Popen'):
            guard=patch(target, side_effect=AssertionError('offline only'))
            guard.start(); self.addCleanup(guard.stop)
        self.fixture=json.loads((Path(__file__).parent/'fixtures/alignment-rotation-cap.json').read_text())
        progress=self.fixture['progress']; self.update=progress['servo']['left_arm']; self.telemetry=progress['tracking']['left_arm']
        self.opts=dict(dt=.05,speed=.12,step=.006,acceleration=.2,cap=.3)

    def bound(self,current,center,previous,desired):
        old=g._bounded_velocity(current,center,previous,desired,**self.opts,angular=True)
        return adapter.tracking.bounded_tangent_escape(current,center,previous,desired,old,**self.opts)

    def assert_bounds(self,current,center,previous,v):
        o=self.opts; dt=o['dt']; speed=g._norm(v)
        self.assertLessEqual(speed, min(o['speed'],o['step']/dt)+1e-12)
        self.assertLessEqual(g._norm(g._sub(v,previous)),o['acceleration']*dt+1e-12)
        end=g._integrate(current,g._scale(v,dt),True)
        stop=g._integrate(end,g._scale(v,speed/(2*o['acceleration'])),True)
        self.assertLessEqual(g._distance(end,center,True),o['cap']+4e-12)
        self.assertLessEqual(g._distance(stop,center,True),o['cap']+4e-12)
        return end

    def test_recorded_state_has_feasible_descent_without_relaxing_cap(self):
        cur=self.update['command']['quaternion_xyzw']; center=self.telemetry['reference']['quaternion_xyzw']
        error=self.telemetry['tracking_error']['rotation_vector_rad']; desired=g._scale(error,3*max(0,1-.01/g._norm(error)))
        old=g._bounded_velocity(cur,center,g._ZERO,desired,**self.opts,angular=True)
        self.assertEqual(old,g._ZERO)
        v=self.bound(cur,center,g._ZERO,desired)
        self.assertGreater(sum(a*b for a,b in zip(error,v)),0.)
        self.assert_bounds(cur,center,g._ZERO,v)

    def test_stationary_pure_outward_demand_stays_stopped(self):
        center=(0.,0.,0.,1.);cur=g._quat_from_rotvec((.3,0.,0.))
        self.assertLessEqual(g._norm(self.bound(cur,center,g._ZERO,(.08,0.,0.))),4*g._ROUNDING/self.opts['dt'])

    def test_nonstationary_and_outside_cap_do_not_get_fallback(self):
        center=(0.,0.,0.,1.)
        for cur,previous in [(g._quat_from_rotvec((.3,0.,0.)),(.001,0.,0.)),
                             (g._quat_from_rotvec((.3+1e-8,0.,0.)),g._ZERO)]:
            self.assertEqual(adapter.tracking.bounded_tangent_escape(cur,center,previous,(.06,.04,0.),g._ZERO,**self.opts),g._ZERO)

    def test_random_frames_preserve_step_speed_acceleration_and_braking(self):
        rng=random.Random(637)
        moved=0
        for _ in range(200):
            center=g._quat_from_rotvec(tuple(rng.uniform(-1,1) for _ in range(3)))
            axis=g._limit(tuple(rng.uniform(-1,1) for _ in range(3)),1)
            axis=g._scale(axis,1/g._norm(axis));cur=g._integrate(center,g._scale(axis,.3),True)
            d=tuple(rng.uniform(-.1,.1) for _ in range(3))
            v=self.bound(cur,center,g._ZERO,d);self.assert_bounds(cur,center,g._ZERO,v)
            moved+=g._norm(v)>1e-8
        self.assertGreater(moved,150)

    def test_synthetic_local_plant_reduces_recorded_error_with_same_limits(self):
        # Constant local disturbance is a test model, not a claim about hardware.
        t=self.telemetry;u=self.update
        ref=g.Pose(**t['reference']);saved=g.Pose(**u['command'])
        e=t['tracking_error'];measured=g.Pose(g._sub(ref.position_m,e['translation_vector_m']),g._integrate(ref.quaternion_xyzw,g._scale(e['rotation_vector_rad'],-1),True))
        config=g.ServoConfig(**self.fixture['config']['command_config'])
        servo=adapter.tracking.TrackingServo(measured,ref,saved,config,2.55,profile='fast_tracking',path_rotation_tolerance_rad=.035,nominal_position_tolerance_m=.001,confirmed_orientation_reference=ref.quaternion_xyzw,confirmed_rotation_tolerance_rad=.2)
        previous=g._ZERO
        first=g._error(measured,ref)['rotation_rad']
        for _ in range(240):
            proposal=servo.update(measured,.05)
            v=proposal.command_velocity['rotation_rad_s'];self.assert_bounds(servo.confirmed_command.quaternion_xyzw,servo._goal.quaternion_xyzw,previous,v)
            before=servo.confirmed_command;servo.confirm_command(proposal.command)
            delta=g._error(before,proposal.command)
            measured=g.Pose(g._add(measured.position_m,delta['translation_vector_m']),g._integrate(measured.quaternion_xyzw,delta['rotation_vector_rad'],True))
            previous=v
            if proposal.arrival_latched and proposal.command_stationary:break
        self.assertLess(g._error(measured,ref)['rotation_rad'],.035)
        self.assertLess(g._error(measured,ref)['rotation_rad'],first)
        self.assertTrue(proposal.arrival_latched)

if __name__=='__main__': unittest.main()
