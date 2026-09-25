#!/usr/bin/env python3

"""Regression test for FAST-LIVO2 IMU callback scheduling."""

from pathlib import Path
import unittest


ROOT = Path(__file__).resolve().parents[1]
HEADER = (ROOT / "include" / "LIVMapper.h").read_text()
SOURCE = (ROOT / "src" / "LIVMapper.cpp").read_text()


class ImuCallbackQueueTest(unittest.TestCase):
    def test_imu_has_an_ordered_dedicated_callback_queue(self):
        self.assertIn("ros::CallbackQueue imu_callback_queue;", HEADER)
        self.assertIn(
            "std::unique_ptr<ros::AsyncSpinner> imu_spinner;", HEADER
        )
        self.assertIn("imu_nh.setCallbackQueue(&imu_callback_queue);", SOURCE)
        self.assertIn(
            "new ros::AsyncSpinner(1, &imu_callback_queue)", SOURCE
        )

    def test_body_pose_extrinsic_is_independent_of_heading_source(self):
        self.assertIn('nh.param<bool>("body_pose/enabled"', SOURCE)
        self.assertIn('"body_pose/zero_initial_position"', SOURCE)
        self.assertIn("if (body_pose_output_en)", SOURCE)
        self.assertIn("tryInitializeBodyPoseAlignment();", SOURCE)
        self.assertIn("worldRotationToZeroInitialBodyYaw", SOURCE)

    def test_fresh_imu_drives_propagated_odometry(self):
        self.assertIn("void propagateImuOdometry();", HEADER)
        self.assertNotIn("ros::Timer imu_prop_timer;", HEADER)
        self.assertIn("bool new_imu = false", HEADER)
        self.assertIn("sensor_msgs::Imu newest_imu;", HEADER)
        self.assertIn(
            "void LIVMapper::propagateImuOdometry()", SOURCE
        )

    def test_imu_callback_propagates_each_fresh_sample(self):
        callback_start = SOURCE.index("void LIVMapper::imu_cbk")
        callback_end = SOURCE.index(
            "cv::Mat LIVMapper::getImageFromMsg", callback_start
        )
        callback = SOURCE[callback_start:callback_end]

        self.assertIn("newest_imu = *msg;", callback)
        self.assertIn("new_imu = true;", callback)
        self.assertIn("propagateImuOdometry();", callback)


if __name__ == "__main__":
    unittest.main()
