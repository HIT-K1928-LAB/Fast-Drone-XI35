"""Offline state-machine tests: no node initialization, subscriptions or commands."""
import threading
import unittest
from types import SimpleNamespace as NS
from unittest.mock import patch

import run_yopo_px4ctrl_recovery as m


def stamp(t):
    return NS(to_sec=lambda: t)


def odom(t, x=0.0, speed=0.0, z=1.1):
    return NS(header=NS(stamp=stamp(t), frame_id="world"),
              pose=NS(pose=NS(position=NS(x=x, y=7.5, z=z),
                              orientation=NS(x=0, y=0, z=0, w=1))),
              twist=NS(twist=NS(linear=NS(x=speed, y=0, z=0))))


class RecoveryTest(unittest.TestCase):
    def setUp(self):
        self.sim = 10.0
        self.wall = 100.0
        self.clock = patch.object(m.rospy.Time, "now", side_effect=lambda: stamp(self.sim))
        self.clock.start()
        self.monotonic = patch.object(m.time, "monotonic", side_effect=lambda: self.wall)
        self.monotonic.start()
        self.logs = patch.object(m.rospy, "logwarn")
        self.logs.start()
        self.a = a = m.RecoveryYopo.__new__(m.RecoveryYopo)
        a.guard = threading.RLock()
        a.phase, a.active = "TRACK", True
        a.execute, a.auto_resume = True, True
        a.recoveries, a.max_recoveries = 0, 3
        a.cmd_timeout = 0.5
        a.last_odom_stamp = a.last_replan_stamp = -float("inf")
        a.last_report = -float("inf")
        a.odom, a.odom_init = odom(self.sim), True
        a.goal = m.np.array([20., 7.5, 1.1])
        a.input_problem = lambda: None
        a.exclusive_output = lambda: True
        a.events = []
        a.emit = lambda reason: a.events.append((a.phase, reason))
        a.mu_lo = None
        a.desire_init = True
        a.ctrl_time = 1.0
        a.brake = False
        self.pipeline = patch.object(m.upstream.YopoNet, "callback_depth",
                                     lambda a, msg: self.plan(a, msg))
        self.pipeline.start()
        self.plan_calls = 0
        self.accept = True
        self.floor = 1.1
        self.seeds = []

    def tearDown(self):
        self.pipeline.stop()
        self.logs.stop()
        self.monotonic.stop()
        self.clock.stop()

    def plan(self, a, msg):
        self.plan_calls += 1
        self.seeds.append(a.desire_pos.copy())
        a.brake = not self.accept
        a.best_total_time = 2.0
        a.ctrl_time = 0.0
        def pos(t):
            v = m.np.tile(a._odom_pos(), (len(t), 1))
            v[:, 2] = self.floor
            return v
        a.optimal_traj = NS(position=pos,
                            velocity=lambda t: m.np.zeros((len(t), 3)),
                            acceleration=lambda t: m.np.zeros((len(t), 3)))

    def brake(self):
        self.a.stop("upstream corridor admission requested braking")

    def hover(self):
        self.brake()
        self.a.callback_odometry(odom(11.01))
        self.a.callback_odometry(odom(12.02))
        self.assertEqual(self.a.phase, "HOVER_REPLAN")

    def depth(self, t):
        self.sim = t
        self.a.callback_odometry(odom(t))
        self.a.callback_depth(NS(header=NS(stamp=stamp(t)), width=640, height=370))

    def test_brake_suppresses_control_and_retains_goal(self):
        self.brake()
        self.assertTrue(self.a.active)
        self.assertEqual(self.a.phase, "BRAKE")
        self.assertIsNone(self.a.ctrl_time)
        with patch.object(m.GazeboYopo, "control_pub") as forward:
            self.a.control_pub(None)
            forward.assert_not_called()

    def test_waits_for_timeout_and_continuous_stability(self):
        self.brake()
        self.a.callback_odometry(odom(10.9))
        self.assertIsNone(self.a.stable_since)
        self.a.callback_odometry(odom(11.1))
        self.a.callback_odometry(odom(11.9, speed=.2))
        self.assertIsNone(self.a.stable_since)
        self.a.callback_odometry(odom(12.0))
        self.a.callback_odometry(odom(12.8))
        self.assertEqual(self.a.phase, "BRAKE")
        self.a.callback_odometry(odom(13.1))
        self.assertEqual(self.a.phase, "HOVER_REPLAN")

    def test_resume_uses_fresh_reference_after_five_distinct_frames(self):
        self.hover()
        self.a.desire_pos = m.np.array([999., 999., 999.])
        for t in [12.1, 12.21, 12.32, 12.43]:
            self.depth(t)
            self.assertEqual(self.a.phase, "HOVER_REPLAN")
            self.assertIsNone(self.a.ctrl_time)
        self.depth(12.54)
        self.assertEqual(self.a.phase, "TRACK")
        self.assertEqual(self.a.ctrl_time, 0.0)
        for p in self.seeds:
            m.np.testing.assert_allclose(p, [0, 7.5, 1.1])
        m.np.testing.assert_allclose(self.a.desire_acc, [0, 0, 0])

    def test_preview_never_resumes_or_forwards_control(self):
        self.a.auto_resume = False
        self.hover()
        with patch.object(m.GazeboYopo, "control_pub") as forward:
            for t in m.np.arange(12.1, 13.3, .11):
                self.depth(t)
                self.a.control_pub(None)
            forward.assert_not_called()
        self.assertEqual(self.a.phase, "HOVER_REPLAN")
        self.assertIsNone(self.a.ctrl_time)

    def test_rejected_candidate_resets_streak(self):
        self.hover()
        self.depth(12.1)
        self.depth(12.21)
        self.accept = False
        self.depth(12.32)
        self.assertEqual(self.a.good_frames, 0)
        self.accept = True
        self.depth(12.43)
        self.assertEqual(self.a.good_frames, 1)

    def test_low_height_plan_not_admitted(self):
        self.hover()
        self.floor = .4
        self.depth(12.1)
        self.assertEqual(self.a.good_frames, 0)

    def test_nonfinite_plan_not_admitted(self):
        self.hover()
        self.floor = float("nan")
        self.depth(12.1)
        self.assertEqual(self.a.good_frames, 0)

    def test_invalid_odometry_stops(self):
        self.brake()
        self.a.callback_odometry(odom(11.1, x=float("nan")))
        self.assertEqual(self.a.phase, "STOPPED")

    def test_backwards_odometry_stops(self):
        self.hover()
        self.a.callback_odometry(odom(11.9))
        self.assertEqual(self.a.phase, "STOPPED")

    def test_duplicate_images_do_not_count(self):
        self.hover()
        for _ in range(10):
            self.depth(12.1)
        self.assertEqual(self.a.good_frames, 1)

    def test_motion_interrupts_replanning(self):
        self.hover()
        self.depth(12.1)
        self.a.callback_odometry(odom(12.2, speed=.2))
        self.assertEqual(self.a.phase, "BRAKE")
        self.assertEqual(self.a.good_frames, 0)
        self.assertIsNone(self.a.ctrl_time)

    def test_user_stop_cannot_auto_resume(self):
        self.hover()
        self.a.receive_stop(None)
        self.assertFalse(self.a.active)
        self.depth(12.1)
        self.assertEqual(self.plan_calls, 0)
        self.assertEqual(self.a.phase, "STOPPED")

    def test_stale_inputs_terminal(self):
        self.brake()
        self.a.input_problem = lambda: "odometry/depth missing or stale"
        self.a.control_pub(None)
        self.assertEqual(self.a.phase, "STOPPED")

    def test_recovery_timeout_terminal(self):
        self.brake()
        self.wall += 31
        self.a.control_pub(None)
        self.assertEqual(self.a.phase, "STOPPED")

    def test_three_recovery_limit(self):
        self.a.recoveries = 3
        self.brake()
        self.assertEqual(self.a.phase, "STOPPED")
        self.assertFalse(self.a.active)

    def test_competing_publisher_prevents_resume(self):
        self.hover()
        self.a.exclusive_output = lambda: False
        for t in [12.1, 12.21, 12.32, 12.43, 12.54]:
            self.depth(t)
        self.assertEqual(self.a.phase, "STOPPED")

    def test_arrival_remains_terminal_during_recovery(self):
        self.hover()
        self.a.goal = m.np.array([0., 7.5, 1.1])
        self.a.callback_odometry(odom(12.1))
        self.assertEqual(self.a.phase, "STOPPED")


if __name__ == "__main__":
    unittest.main()
