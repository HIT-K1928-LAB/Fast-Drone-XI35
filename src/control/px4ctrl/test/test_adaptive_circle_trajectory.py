#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import unittest


SCRIPT = pathlib.Path(__file__).parents[1] / "scripts" / "adaptive_circle_trajectory.py"


def load_module():
    spec = importlib.util.spec_from_file_location("adaptive_circle_trajectory", str(SCRIPT))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class CircleKinematicsTest(unittest.TestCase):
    def test_counterclockwise_command_contains_consistent_derivatives(self):
        module = load_module()

        command = module.circle_command(
            center=(1.0, 2.0), altitude=3.0, radius=2.0, phase=0.0,
            speed=4.0, clockwise=False)

        self.assertEqual(command.position, (3.0, 2.0, 3.0))
        self.assertAlmostEqual(command.velocity[0], 0.0)
        self.assertAlmostEqual(command.velocity[1], 4.0)
        self.assertAlmostEqual(command.velocity[2], 0.0)
        self.assertAlmostEqual(command.acceleration[0], -8.0)
        self.assertAlmostEqual(command.acceleration[1], 0.0)
        self.assertAlmostEqual(command.jerk[0], 0.0)
        self.assertAlmostEqual(command.jerk[1], -16.0)
        self.assertAlmostEqual(command.yaw, math.pi / 2.0)
        self.assertAlmostEqual(command.yaw_rate, 2.0)

    def test_speed_envelope_uses_centripetal_acceleration_limit(self):
        module = load_module()

        limit = module.feasible_speed_limit(
            radius=2.0, max_velocity=20.0, max_acceleration=10.0,
            hard_max_speed=0.0)

        self.assertAlmostEqual(limit, math.sqrt(20.0))

    def test_speed_ramp_adds_tangential_acceleration_and_consistent_jerk(self):
        module = load_module()

        command = module.circle_command(
            center=(0.0, 0.0), altitude=1.0, radius=2.0, phase=0.0,
            speed=2.0, clockwise=False, tangential_acceleration=1.0)

        self.assertAlmostEqual(command.acceleration[0], -2.0)
        self.assertAlmostEqual(command.acceleration[1], 1.0)
        self.assertAlmostEqual(command.jerk[0], -3.0)
        self.assertAlmostEqual(command.jerk[1], -2.0)

    def test_initial_geometry_preserves_position_and_heading(self):
        module = load_module()
        center, phase = module.initial_circle_geometry(
            position=(4.0, -2.0), yaw=0.3, radius=2.0, clockwise=False)

        command = module.circle_command(
            center=center, altitude=1.0, radius=2.0, phase=phase,
            speed=1.0, clockwise=False)

        self.assertAlmostEqual(command.position[0], 4.0)
        self.assertAlmostEqual(command.position[1], -2.0)
        self.assertAlmostEqual(command.yaw, 0.3)

    def test_zero_speed_command_keeps_the_circle_tangent_heading(self):
        module = load_module()
        center, phase = module.initial_circle_geometry(
            position=(4.0, -2.0), yaw=0.3, radius=2.0, clockwise=False)

        command = module.circle_command(
            center=center, altitude=1.0, radius=2.0, phase=phase,
            speed=0.0, clockwise=False)

        self.assertAlmostEqual(command.yaw, 0.3)


class AdaptiveSpeedSearchTest(unittest.TestCase):
    def test_stable_tracking_promotes_the_next_speed_level(self):
        module = load_module()
        search = module.AdaptiveSpeedSearch(
            initial_speed=1.0, speed_step=0.5, settle_time=1.0, window_time=2.0,
            pass_rms_error=0.10, pass_p95_error=0.15, fail_rms_error=0.20,
            fail_peak_error=0.30, emergency_error=0.50,
            emergency_hold_time=0.30, speed_limit=3.0)

        search.observe(now=0.0, error=0.05, at_target=True)
        search.observe(now=1.0, error=0.05, at_target=True)
        search.observe(now=2.0, error=0.08, at_target=True)
        decision = search.observe(now=3.1, error=0.06, at_target=True)

        self.assertEqual(decision.state, "RAMPING")
        self.assertAlmostEqual(decision.target_speed, 1.5)
        self.assertAlmostEqual(decision.last_safe_speed, 1.0)
        self.assertFalse(decision.emergency)

    def test_failed_level_returns_to_last_validated_speed_and_stops_search(self):
        module = load_module()
        search = module.AdaptiveSpeedSearch(
            initial_speed=1.0, speed_step=0.5, settle_time=1.0, window_time=2.0,
            pass_rms_error=0.10, pass_p95_error=0.15, fail_rms_error=0.20,
            fail_peak_error=0.30, emergency_error=0.50,
            emergency_hold_time=0.30, speed_limit=3.0)

        for now in (0.0, 1.0, 2.0, 3.1):
            search.observe(now=now, error=0.05, at_target=True)
        search.observe(now=4.0, error=0.25, at_target=True)
        search.observe(now=5.0, error=0.25, at_target=True)
        search.observe(now=6.0, error=0.25, at_target=True)
        decision = search.observe(now=7.1, error=0.25, at_target=True)

        self.assertEqual(decision.state, "COMPLETE")
        self.assertAlmostEqual(decision.target_speed, 1.0)
        self.assertAlmostEqual(decision.last_safe_speed, 1.0)
        self.assertGreater(decision.rms_error, 0.20)

    def test_sustained_critical_error_requests_emergency_hover(self):
        module = load_module()
        search = module.AdaptiveSpeedSearch(
            initial_speed=1.0, speed_step=0.5, settle_time=1.0, window_time=2.0,
            pass_rms_error=0.10, pass_p95_error=0.15, fail_rms_error=0.20,
            fail_peak_error=0.30, emergency_error=0.50,
            emergency_hold_time=0.30, speed_limit=3.0)

        self.assertFalse(search.observe(0.0, 0.60, False).emergency)
        self.assertFalse(search.observe(0.20, 0.60, False).emergency)
        decision = search.observe(0.31, 0.60, False)

        self.assertTrue(decision.emergency)
        self.assertEqual(decision.state, "EMERGENCY")

    def test_search_reports_when_the_fsm_envelope_is_reached(self):
        module = load_module()
        search = module.AdaptiveSpeedSearch(
            initial_speed=1.0, speed_step=0.5, settle_time=0.0, window_time=1.0,
            pass_rms_error=0.10, pass_p95_error=0.15, fail_rms_error=0.20,
            fail_peak_error=0.30, emergency_error=0.50,
            emergency_hold_time=0.30, speed_limit=1.25)

        search.observe(0.0, 0.05, True)
        search.observe(1.1, 0.05, True)
        self.assertAlmostEqual(search.target_speed, 1.25)
        search.observe(2.0, 0.05, True)
        decision = search.observe(3.1, 0.05, True)

        self.assertEqual(decision.state, "LIMIT_REACHED")
        self.assertAlmostEqual(decision.target_speed, 1.25)
        self.assertAlmostEqual(decision.last_safe_speed, 1.25)


if __name__ == "__main__":
    unittest.main()
