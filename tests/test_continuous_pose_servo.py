"""Deterministic geometry and biased/lagged plants; no robot, clock or network."""
import ast
import dataclasses
import importlib.util
import json
import math
from pathlib import Path
import sys
import unittest

SKILLS = Path(__file__).resolve().parents[1]
ATOMIC_PATH = SKILLS / 'atomic_skills/dual_franka_p2p/skills/atomic-motion-franka-move-to-pose/continuous_pose_servo.py'


def load_module(name, path):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    sys.modules[name] = module
    spec.loader.exec_module(module)
    return module


servo = load_module('atomic_continuous_pose_servo_tests', ATOMIC_PATH)
Pose = servo.Pose


def pose(position=(0., 0., 0.), rotation=(0., 0., 0.)):
    return Pose(tuple(position), servo._quat_from_rotvec(rotation))


def shifted(value, translation=(0., 0., 0.), rotation=(0., 0., 0.)):
    return Pose(servo._add(value.position_m, translation),
                servo._integrate(value.quaternion_xyzw, rotation, True), value.frame)


class ContinuousPoseServoTests(unittest.TestCase):
    def commit(self, controller, measured, dt=.05):
        previous = controller.confirmed_command
        update = controller.update(measured, dt)
        self.assertEqual(controller.confirmed_command, previous)
        controller.confirm_command(update.command)
        self.assertEqual(controller.confirmed_command, update.command)
        return update

    def assert_bounds(self, update, previous_velocity, config):
        if config.max_translation_bias_m is not None:
            self.assertLessEqual(update.bias['translation_m'], config.max_translation_bias_m + 2e-12)
        else:
            self.assertFalse(update.at_cap['translation'])
        self.assertLessEqual(update.bias['rotation_rad'], config.max_rotation_bias_rad + 2e-12)
        for key, delta, speed, step, acceleration in (
            ('translation_m_s', update.proposed_delta[:3], config.max_translation_speed_m_s,
             config.max_translation_step_m, config.max_translation_acceleration_m_s2),
            ('rotation_rad_s', update.proposed_delta[3:], config.max_rotation_speed_rad_s,
             config.max_rotation_step_rad, config.max_rotation_acceleration_rad_s2),
        ):
            velocity = update.command_velocity[key]
            self.assertLessEqual(math.hypot(*velocity), speed + 2e-12)
            self.assertLessEqual(math.hypot(*delta), step + 2e-12)
            self.assertLessEqual(math.hypot(*servo._sub(velocity, previous_velocity[key])),
                                 acceleration * update.dt_s + 2e-10)
        return update.command_velocity

    def simulate(self, *, lag_seconds, strict=True, noncollinear=False, delay_steps=0):
        config = servo.ServoConfig(rotation_tolerance_rad=.03 if strict else .1)
        goal = pose((.3, -.2, .1), (2.4, -1.6, .4) if noncollinear else (0., 0., 0.))
        controller = servo.ContinuousPoseServo(goal, goal, config)
        bias_translation = (-.009, .006, -.003)
        bias_rotation = (-.10, .07, -.05) if noncollinear else (0., 0., -.16)
        measured = shifted(goal, bias_translation, bias_rotation)
        previous_velocity = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        history = []
        delayed_commands = [goal] * delay_steps
        for index in range(1600):
            update = self.commit(controller, measured)
            previous_velocity = self.assert_bounds(update, previous_velocity, config)
            history.append(update)
            delayed_commands.append(controller.confirmed_command)
            equilibrium = shifted(delayed_commands.pop(0), bias_translation, bias_rotation)
            fraction = 1. if not lag_seconds else 1. - math.exp(-.05 / lag_seconds)
            measured = shifted(measured,
                               servo._scale(servo._sub(equilibrium.position_m, measured.position_m), fraction),
                               servo._scale(servo._rotation_delta(measured.quaternion_xyzw,
                                                                 equilibrium.quaternion_xyzw), fraction))
            if update.arrival_latched and update.command_stationary:
                break
        self.assertLess(index, 1599, 'bounded simulated plant did not reach a held command')
        self.assertTrue(history[-1].within_tolerance)
        self.assertEqual(history[-1].status, 'holding')
        held = controller.confirmed_command
        for _ in range(20):
            delayed_commands.append(controller.confirmed_command)
            equilibrium = shifted(delayed_commands.pop(0), bias_translation, bias_rotation)
            fraction = 1. if not lag_seconds else 1. - math.exp(-.05 / lag_seconds)
            measured = shifted(measured,
                               servo._scale(servo._sub(equilibrium.position_m, measured.position_m), fraction),
                               servo._scale(servo._rotation_delta(measured.quaternion_xyzw,
                                                                 equilibrium.quaternion_xyzw), fraction))
            update = self.commit(controller, measured)
            self.assertTrue(update.within_tolerance)
            self.assertEqual(update.proposed_delta, (0.,) * 6)
            self.assertEqual(controller.confirmed_command, held)
        self.assertGreater(servo._error(goal, held)['translation_m'], .003)
        self.assertGreater(servo._error(goal, held)['rotation_rad'], .03)
        return history

    def test_constant_pose_bias_converges_without_resets_or_segment_settling(self):
        self.simulate(lag_seconds=0.)

    def test_first_order_lag_and_constant_bias_converge_at_conservative_gain(self):
        self.simulate(lag_seconds=.25)

    def test_noncollinear_rotation_bias_with_lag_converges_on_so3(self):
        self.simulate(lag_seconds=.2, noncollinear=True)

    def test_transport_deadtime_and_first_order_lag_converge_in_bounded_simulation(self):
        self.simulate(lag_seconds=.2, delay_steps=3)

    def test_user_rotation_tolerance_is_separate_from_strict_experiment(self):
        loose = self.simulate(lag_seconds=.15, strict=False)
        strict = self.simulate(lag_seconds=.15, strict=True)
        self.assertLess(len(loose), len(strict))
        self.assertLessEqual(loose[-1].errors['rotation_rad'], .1)
        self.assertLessEqual(strict[-1].errors['rotation_rad'], .03)

    def test_initial_saved_command_is_never_reset_to_the_nominal_goal(self):
        goal = pose()
        saved = pose((.01, -.003, 0.), (0., .12, 0.))
        controller = servo.ContinuousPoseServo(goal, saved)
        update = controller.update(goal, .05)
        self.assertTrue(update.command_stationary)
        self.assertTrue(update.within_tolerance)
        self.assertEqual(update.command, saved)
        self.assertEqual(update.proposed_delta, (0.,) * 6)
        controller.confirm_command(saved)
        self.assertEqual(controller.confirmed_command, saved)

    def test_invalid_initial_bias_is_rejected_instead_of_reanchored(self):
        for saved in (pose((.02501, 0., 0.)), pose(rotation=(0., 0., .20001))):
            with self.subTest(saved=saved), self.assertRaisesRegex(ValueError, 'bias cap'):
                servo.ContinuousPoseServo(pose(), saved)
        servo.ContinuousPoseServo(pose(), pose((.025, 0., 0.), (0., 0., .2)))

    def test_fixed_goal_has_no_recenter_api(self):
        controller = servo.ContinuousPoseServo(pose(), pose())
        with self.assertRaises(AttributeError):
            controller.goal = pose((.1, 0., 0.))

    def test_pending_command_must_be_confirmed_before_next_update(self):
        controller = servo.ContinuousPoseServo(pose(), pose())
        update = controller.update(pose((-.01, 0., 0.)), .05)
        with self.assertRaisesRegex(RuntimeError, 'pending'):
            controller.update(pose((-.01, 0., 0.)), .05)
        self.assertEqual(controller.confirmed_command, pose())
        controller.confirm_command(update.command)
        self.assertIsNone(controller.pending)
        with self.assertRaisesRegex(RuntimeError, 'No pending'):
            controller.confirm_command(update.command)

    def test_bad_ack_keeps_last_confirmed_command_and_blocks_another_proposal(self):
        controller = servo.ContinuousPoseServo(pose(), pose())
        update = controller.update(pose((-.01, 0., 0.)), .05)
        for bad in (pose((.01, 0., 0.)), Pose((0., 0., 0.), (0., 0., 0., 1.), 'left_base')):
            with self.subTest(bad=bad), self.assertRaises(ValueError):
                controller.confirm_command(bad)
            self.assertEqual(controller.confirmed_command, pose())
            self.assertIs(controller.pending, update)

    def test_close_ack_must_also_preserve_the_braking_reserve(self):
        saved = pose((.02497, 0., 0.))
        controller = servo.ContinuousPoseServo(pose(), saved)
        update = controller.update(pose((-.1, 0., 0.)), .05)
        # 0.1 micrometre is inside the acknowledgement match tolerance and the
        # position cap, but consumes the exactly reserved outward braking room.
        biased_ack = shifted(update.command, (.0000001, 0., 0.))
        with self.assertRaisesRegex(ValueError, 'reserved braking distance'):
            controller.confirm_command(biased_ack)
        self.assertEqual(controller.confirmed_command, saved)
        self.assertIs(controller.pending, update)

    def test_nonfinite_or_late_dt_does_not_integrate_or_create_pending(self):
        controller = servo.ContinuousPoseServo(pose(), pose())
        for dt in (0., -.1, .2000001, math.inf, math.nan, True, '0.05'):
            with self.subTest(dt=dt), self.assertRaises(ValueError):
                controller.update(pose((-.01, 0., 0.)), dt)
            self.assertEqual(controller.confirmed_command, pose())
            self.assertIsNone(controller.pending)
        self.commit(controller, pose((-.01, 0., 0.)), .2)

    def test_wrong_frame_or_nonfinite_measurement_cannot_advance_integrator(self):
        controller = servo.ContinuousPoseServo(pose(), pose())
        for measured in (None, [0.] * 6, Pose((0., 0., 0.), (0., 0., 0., 1.), 'left_base')):
            with self.subTest(measured=measured), self.assertRaises(ValueError):
                controller.update(measured, .05)
            self.assertIsNone(controller.pending)
        with self.assertRaises(ValueError):
            Pose((math.nan, 0., 0.), (0., 0., 0., 1.))

    def test_configuration_requires_finite_positive_values(self):
        for field in dataclasses.fields(servo.ServoConfig):
            for bad in (0., -1., math.inf, math.nan, True):
                with self.subTest(field=field.name, bad=bad), self.assertRaises(ValueError):
                    servo.ServoConfig(**{field.name: bad})
        with self.assertRaises(ValueError):
            servo.ServoConfig(max_rotation_bias_rad=math.pi / 2.)

    def test_only_translation_radius_accepts_none_and_serializes_as_json_null(self):
        config = servo.ServoConfig(max_translation_bias_m=None)
        self.assertIsNone(config.max_translation_bias_m)
        self.assertEqual(servo.ServoConfig().max_translation_bias_m, .025)
        record = json.loads(json.dumps(dataclasses.asdict(config), allow_nan=False))
        self.assertIsNone(record['max_translation_bias_m'])
        for field in dataclasses.fields(servo.ServoConfig):
            if field.name != 'max_translation_bias_m':
                with self.subTest(field=field.name), self.assertRaises(ValueError):
                    servo.ServoConfig(**{field.name: None})

    def test_no_translation_radius_keeps_existing_large_saved_compensation(self):
        config = servo.ServoConfig(max_translation_bias_m=None)
        saved = pose((.040, -.010, .005), (0., .12, 0.))
        controller = servo.ContinuousPoseServo(pose(), saved, config)
        update = self.commit(controller, pose())
        self.assertEqual(update.command, saved)
        self.assertEqual(update.proposed_delta, (0.,) * 6)
        self.assertFalse(update.at_cap['translation'])
        self.assertTrue(update.within_tolerance)
        self.assertGreater(update.bias['translation_m'], .025)
        json.dumps(update.to_dict(), allow_nan=False)

    def test_no_translation_radius_converges_beyond_25mm_and_brakes_without_reset(self):
        config = servo.ServoConfig(max_translation_bias_m=None)
        controller = servo.ContinuousPoseServo(pose(), pose(), config)
        plant_bias = (-.035, .008, -.005)
        measured = shifted(pose(), plant_bias)
        previous = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        crossed_25mm = False
        for index in range(600):
            update = self.commit(controller, measured)
            previous = self.assert_bounds(update, previous, config)
            crossed_25mm |= update.bias['translation_m'] > .025
            equilibrium = shifted(controller.confirmed_command, plant_bias)
            # A first-order plant adds measurable lag without any settle call.
            measured = shifted(measured, servo._scale(
                servo._sub(equilibrium.position_m, measured.position_m), 1. - math.exp(-.05 / .2)))
            if update.arrival_latched and update.command_stationary:
                break
        self.assertLess(index, 599)
        self.assertTrue(crossed_25mm)
        self.assertTrue(update.within_tolerance)
        self.assertEqual(update.status, 'holding')
        held = controller.confirmed_command
        self.assertGreater(update.bias['translation_m'], .030)
        for _ in range(20):
            update = self.commit(controller, shifted(held, plant_bias))
            self.assertTrue(update.within_tolerance)
            self.assertEqual(update.command, held)
            self.assertEqual(update.proposed_delta, (0.,) * 6)

    def test_no_translation_radius_does_not_disable_rotation_cap_or_braking_reserve(self):
        config = servo.ServoConfig(max_translation_bias_m=None)
        with self.assertRaisesRegex(ValueError, 'bias cap'):
            servo.ContinuousPoseServo(pose(), pose((.04, 0., 0.), (0., 0., .20001)), config)
        controller = servo.ContinuousPoseServo(pose(), pose((.04, 0., 0.)), config)
        previous = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        for _ in range(300):
            update = self.commit(controller, pose((-.1, 0., 0.), (0., 0., -.8)))
            previous = self.assert_bounds(update, previous, config)
        self.assertTrue(update.at_cap['rotation'])
        self.assertFalse(update.at_cap['translation'])
        self.assertEqual(update.command_velocity['rotation_rad_s'], (0.,) * 3)
        self.assertGreater(update.command_velocity['translation_m_s'][0], 0.)
        self.assertGreater(update.bias['translation_m'], .09)
        self.assertFalse(update.within_tolerance)
        saved = pose((.04, 0., 0.), (0., 0., .19997))
        controller = servo.ContinuousPoseServo(pose(), saved, config)
        update = controller.update(pose((-.1, 0., 0.), (0., 0., -.8)), .05)
        with self.assertRaisesRegex(ValueError, 'reserved braking distance'):
            controller.confirm_command(shifted(update.command, rotation=(0., 0., 1e-7)))
        self.assertEqual(controller.confirmed_command, saved)
        self.assertIs(controller.pending, update)

    def test_no_translation_radius_preserves_slew_limits_during_reversal(self):
        config = servo.ServoConfig(max_translation_bias_m=None)
        controller = servo.ContinuousPoseServo(pose(), pose((.04, 0., 0.)), config)
        previous = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        for index in range(100):
            measured = pose((-.1 if index < 50 else .1, 0., 0.))
            update = self.commit(controller, measured)
            previous = self.assert_bounds(update, previous, config)
        self.assertLess(update.command_velocity['translation_m_s'][0], 0.)

    def test_no_translation_radius_rejects_infeasible_step_and_late_feedback(self):
        config = servo.ServoConfig(max_translation_bias_m=None)
        controller = servo.ContinuousPoseServo(pose(), pose((.04, 0., 0.)), config)
        for _ in range(20):
            self.commit(controller, pose((-.1, 0., 0.)))
        before = controller.confirmed_command
        for dt, error in ((.2, servo.NoFeasibleStep), (.201, ValueError),
                          (math.inf, ValueError), (math.nan, ValueError)):
            with self.subTest(dt=dt), self.assertRaises(error):
                controller.update(pose((-.1, 0., 0.)), dt)
            self.assertEqual(controller.confirmed_command, before)
            self.assertIsNone(controller.pending)

    def test_no_translation_radius_checks_actual_ack_speed_and_acceleration(self):
        config = servo.ServoConfig(max_translation_bias_m=None)
        for preceding_steps in (0, 20):
            with self.subTest(preceding_steps=preceding_steps):
                controller = servo.ContinuousPoseServo(pose(), pose((.04, 0., 0.)), config)
                for _ in range(preceding_steps):
                    self.commit(controller, pose((-.1, 0., 0.)))
                before = controller.confirmed_command
                update = controller.update(pose((-.1, 0., 0.)), .05)
                # Inside the 1-micrometre ACK match tolerance, but outside the
                # acceleration limit on the first step or speed limit at cruise.
                with self.assertRaisesRegex(ValueError, 'velocity or acceleration'):
                    controller.confirm_command(shifted(update.command, (1e-7, 0., 0.)))
                self.assertEqual(controller.confirmed_command, before)
                self.assertIs(controller.pending, update)
                with self.assertRaisesRegex(RuntimeError, 'pending'):
                    controller.update(pose((-.1, 0., 0.)), .05)

    def test_speed_acceleration_step_caps_and_smooth_direction_reversal(self):
        config = servo.ServoConfig(position_tolerance_m=.0001, rotation_tolerance_rad=.001)
        controller = servo.ContinuousPoseServo(pose(), pose(), config)
        previous = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        for index in range(100):
            sign = -1 if index < 50 else 1
            update = self.commit(controller, pose((sign * .1, 0., 0.), (0., sign * .5, 0.)))
            previous = self.assert_bounds(update, previous, config)
        self.assertLess(update.command_velocity['translation_m_s'][0], 0.)
        self.assertLess(update.command_velocity['rotation_rad_s'][1], 0.)

    def test_small_real_dt_variation_keeps_actual_velocity_and_acceleration_bounded(self):
        config = servo.ServoConfig()
        controller = servo.ContinuousPoseServo(pose(), pose(), config)
        previous = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        for index in range(200):
            dt = (.04, .05, .06)[index % 3]
            update = self.commit(controller, pose((-.05, .03, -.01), (.3, -.2, -.4)), dt)
            previous = self.assert_bounds(update, previous, config)

    def test_bias_cap_brakes_without_windup_or_false_arrival(self):
        config = servo.ServoConfig()
        controller = servo.ContinuousPoseServo(pose(), pose(), config)
        previous = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        for _ in range(700):
            update = self.commit(controller, pose((-.1, 0., 0.), (0., 0., -.8)))
            previous = self.assert_bounds(update, previous, config)
            self.assertFalse(update.within_tolerance)
            self.assertFalse(update.arrival_latched)
        self.assertTrue(all(update.at_cap.values()))
        self.assertEqual(update.status, 'at_cap')
        self.assertTrue(update.command_stationary)
        saturated = controller.confirmed_command
        for _ in range(20):
            update = self.commit(controller, pose((-.1, 0., 0.), (0., 0., -.8)))
            self.assertEqual(update.command, saturated)
        # No hidden windup: reversing the error immediately starts a bounded
        # inward step, rather than spending time unwinding a discarded integral.
        inward = self.commit(controller, pose((.1, 0., 0.), (0., 0., .8)))
        self.assertLess(inward.proposed_delta[0], 0.)
        self.assertLess(inward.proposed_delta[5], 0.)

    def test_acceleration_and_cap_hold_for_noncollinear_boundary_motion(self):
        config = servo.ServoConfig(position_tolerance_m=.00001, rotation_tolerance_rad=.0001)
        goal = pose(rotation=(2.5, -1.7, .3))
        initial = shifted(goal, (.024, 0., 0.), (.19, 0., 0.))
        controller = servo.ContinuousPoseServo(goal, initial, config)
        previous = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        for index in range(350):
            angle = index * .025
            measured = shifted(goal, (-.05 * math.cos(angle), -.05 * math.sin(angle), -.02),
                               (-.4 * math.cos(angle), -.4 * math.sin(angle), -.1))
            update = self.commit(controller, measured)
            previous = self.assert_bounds(update, previous, config)

    def test_arrival_latches_then_brakes_within_bounds_and_keeps_compensation(self):
        config = servo.ServoConfig()
        controller = servo.ContinuousPoseServo(pose(), pose(), config)
        previous = {'translation_m_s': (0., 0., 0.), 'rotation_rad_s': (0., 0., 0.)}
        for _ in range(20):
            update = self.commit(controller, pose((-.01, 0., 0.), (0., 0., -.15)))
            previous = self.assert_bounds(update, previous, config)
        before_braking = controller.confirmed_command
        first = self.commit(controller, pose())
        previous = self.assert_bounds(first, previous, config)
        self.assertTrue(first.arrival_latched)
        self.assertEqual(first.status, 'braking')
        # Feedback may leave tolerance while decelerating. The arrival latch
        # never reaccelerates; only the driver can decide final dwell/failure.
        for _ in range(20):
            update = self.commit(controller, pose((-.01, 0., 0.), (0., 0., -.15)))
            previous = self.assert_bounds(update, previous, config)
            if update.command_stationary:
                break
        self.assertEqual(update.status, 'holding')
        self.assertFalse(update.within_tolerance)
        braking = servo._error(before_braking, update.command)
        self.assertLessEqual(braking['translation_m'], .004 ** 2 / (2 * .01) + 1e-12)
        self.assertLessEqual(braking['rotation_rad'], .03 ** 2 / (2 * .08) + 1e-12)
        held = update.command
        self.assertNotEqual(held, pose())
        for _ in range(10):
            self.assertEqual(self.commit(controller, pose()).command, held)

    def test_infeasible_large_dt_rejects_instead_of_breaking_step_or_acceleration_limit(self):
        controller = servo.ContinuousPoseServo(pose(), pose())
        for _ in range(20):
            self.commit(controller, pose((-.01, 0., 0.), (0., 0., -.15)))
        before = controller.confirmed_command
        # At v=.004, a=.01, dt=.2 the next speed cannot fall below .002,
        # but the .25 mm per-step cap requires <=.00125 m/s.
        with self.assertRaises(servo.NoFeasibleStep):
            controller.update(pose((-.01, 0., 0.), (0., 0., -.15)), .2)
        self.assertEqual(controller.confirmed_command, before)
        self.assertIsNone(controller.pending)

    def test_relative_quaternion_delta_handles_principal_rotvec_branch_crossing(self):
        axis = servo._scale((2.6, -1.75, .28), 1. / math.hypot(2.6, -1.75, .28))
        saved = pose(rotation=servo._scale(axis, math.pi - .04))
        # Quaternion representation of the goal crosses pi; subtracting absolute
        # principal rotvec coordinates here would spuriously request ~2*pi.
        goal = shifted(saved, rotation=servo._scale(axis, .08))
        measured = shifted(saved, rotation=servo._scale(axis, -.08))
        controller = servo.ContinuousPoseServo(goal, saved, servo.ServoConfig(rotation_tolerance_rad=.03))
        update = self.commit(controller, measured)
        self.assertAlmostEqual(update.errors['rotation_rad'], .16)
        self.assertLess(math.hypot(*update.proposed_delta[3:]), .002)
        self.assertGreater(sum(a*b for a, b in zip(update.proposed_delta[3:], axis)), 0.)
        same_rotation_opposite_quaternion = Pose(measured.position_m,
                                                 tuple(-q for q in measured.quaternion_xyzw))
        other = servo.ContinuousPoseServo(goal, saved, servo.ServoConfig(rotation_tolerance_rad=.03))
        identical = self.commit(other, same_rotation_opposite_quaternion)
        for a, b in zip(update.proposed_delta, identical.proposed_delta):
            self.assertAlmostEqual(a, b)

    def test_update_payload_serializes_without_hidden_runtime_objects(self):
        controller = servo.ContinuousPoseServo(pose(), pose())
        update = controller.update(pose((-.01, 0., 0.)), .05)
        record = json.loads(json.dumps(update.to_dict(), allow_nan=False))
        self.assertEqual(record['command']['frame'], 'torso')
        self.assertEqual(len(record['proposed_delta']), 6)
        self.assertIn('rotation_vector_rad', record['errors'])
        self.assertFalse(record['within_tolerance'])


class AtomicResearchParityTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        # The comparison is test-only. Production imports neither this source
        # nor its geometry module, even when the workspace research folder exists.
        study_dir = SKILLS.parent / 'robot_agent_lab/precision_research'
        cls.study_path = study_dir / 'continuous_pose_servo.py'
        study_geometry = load_module('study_servo_geometry_parity', study_dir / 'live_benchmark_math.py')
        old = sys.modules.get('live_benchmark_math')
        sys.modules['live_benchmark_math'] = study_geometry
        try:
            cls.study = load_module('study_servo_parity', cls.study_path)
        finally:
            if old is None:
                sys.modules.pop('live_benchmark_math', None)
            else:
                sys.modules['live_benchmark_math'] = old

    def test_atomic_module_has_only_standard_library_imports(self):
        tree = ast.parse(ATOMIC_PATH.read_text())
        imported = set()
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                imported.update(alias.name.split('.')[0] for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                imported.add(node.module.split('.')[0])
        self.assertEqual(imported, {'__future__', 'dataclasses', 'math', 'typing'})

    def test_default_limits_and_invalid_initial_states_retain_research_semantics(self):
        self.assertEqual(dataclasses.asdict(servo.ServoConfig()),
                         dataclasses.asdict(self.study.ServoConfig()))
        for translation, rotation in (((.02501, 0., 0.), (0., 0., 0.)),
                                      ((0., 0., 0.), (0., 0., .20001))):
            outcomes = []
            for module in (servo, self.study):
                goal = module.Pose((0., 0., 0.), (0., 0., 0., 1.))
                saved = module.Pose(translation, module._quat_from_rotvec(rotation))
                with self.assertRaises(ValueError) as caught:
                    module.ContinuousPoseServo(goal, saved)
                outcomes.append(str(caught.exception))
            self.assertEqual(outcomes[0], outcomes[1])

    def test_embedded_pose_validation_is_equivalent_to_research_geometry(self):
        cases = [
            ((1., 2., 3.), (0., 0., 0., 1.), 'torso'),
            ([1., 2., 3.], [0., 0., 0., 1.0000001], 'torso'),
            ((True, 0., 0.), (0., 0., 0., 1.), 'torso'),
            ((math.nan, 0., 0.), (0., 0., 0., 1.), 'torso'),
            ((0., 0.), (0., 0., 0., 1.), 'torso'),
            ((0., 0., 0.), (0., 0., 0., .5), 'torso'),
            ((0., 0., 0.), (0., 0., 0., 0.), 'torso'),
            ((0., 0., 0.), (0., 0., 0., 1.), ' torso'),
            ((0., 0., 0.), (0., 0., 0., 1.), ''),
        ]
        for values in cases:
            with self.subTest(values=values):
                outcomes = []
                for module in (servo, self.study):
                    try:
                        value = module.Pose(*values)
                        outcomes.append(('valid', value.position_m, value.quaternion_xyzw, value.frame))
                    except ValueError as error:
                        outcomes.append(('invalid', str(error)))
                self.assertEqual(outcomes[0], outcomes[1])

    def test_identical_inputs_produce_identical_proposals_and_confirmed_states(self):
        axis = servo._scale((2.6, -1.75, .28), 1. / math.hypot(2.6, -1.75, .28))
        scenarios = (
            ('biased_lag', (0., 0., 0., 1.), (0., 0., 0., 1.), {}, 280),
            ('cap', (0., 0., 0., 1.), (0., 0., 0., 1.), {}, 700),
            ('cross_pi', servo._quat_from_rotvec(servo._scale(axis, math.pi + .04)),
             servo._quat_from_rotvec(servo._scale(axis, math.pi - .04)), {}, 420),
            ('conservative', (0., 0., 0., 1.), (0., 0., 0., 1.), {
                'translation_integral_gain_per_s': .3, 'rotation_integral_gain_per_s': .3,
                'max_translation_speed_m_s': .002, 'max_rotation_speed_rad_s': .01,
                'max_rotation_acceleration_rad_s2': .02, 'rotation_tolerance_rad': .1,
            }, 500),
        )
        for name, goal_quat, saved_quat, configuration, count in scenarios:
            with self.subTest(scenario=name):
                options = {'rotation_tolerance_rad': .03, **configuration}
                controllers = [module.ContinuousPoseServo(
                    module.Pose((0., 0., 0.), goal_quat), module.Pose((0., 0., 0.), saved_quat),
                    module.ServoConfig(**options)) for module in (servo, self.study)]
                measured_position = (-.009, .006, -.003)
                measured_quaternion = servo._integrate(saved_quat, (-.10, .07, -.05), True)
                for _ in range(count):
                    updates = [controller.update(module.Pose(measured_position, measured_quaternion), .05)
                               for module, controller in zip((servo, self.study), controllers)]
                    self.assertEqual(updates[0].to_dict(), updates[1].to_dict())
                    for controller, update in zip(controllers, updates):
                        controller.confirm_command(update.command)
                    self.assertEqual(controllers[0].confirmed_command.position_m,
                                     controllers[1].confirmed_command.position_m)
                    self.assertEqual(controllers[0].confirmed_command.quaternion_xyzw,
                                     controllers[1].confirmed_command.quaternion_xyzw)
                    if name != 'cap':
                        command = controllers[0].confirmed_command
                        equilibrium_position = servo._add(command.position_m, (-.009, .006, -.003))
                        equilibrium_rotation = servo._integrate(command.quaternion_xyzw, (-.10, .07, -.05), True)
                        fraction = 1. - math.exp(-.05 / .25)
                        measured_position = servo._add(measured_position, servo._scale(
                            servo._sub(equilibrium_position, measured_position), fraction))
                        measured_quaternion = servo._integrate(measured_quaternion, servo._scale(
                            servo._rotation_delta(measured_quaternion, equilibrium_rotation), fraction), True)


if __name__ == '__main__':
    unittest.main()
