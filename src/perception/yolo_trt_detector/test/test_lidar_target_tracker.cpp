#include <gtest/gtest.h>

#include <algorithm>
#include <cmath>

#include <yolo_trt_detector/lidar_target_tracker.h>

namespace {

std::vector<yolo_trt_detector::LidarPoint> targetCloud(
    double target_x, bool include_wall = true) {
  std::vector<yolo_trt_detector::LidarPoint> points;
  if (include_wall) {
    for (int y = -20; y <= 20; ++y) {
      for (int z = 0; z <= 10; ++z) {
        points.push_back({6.0, y * 0.1, z * 0.1, 0.0f});
      }
    }
  }
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      points.push_back({target_x + x * 0.04, y * 0.04, 1.0, 1.0f});
    }
  }
  return points;
}

}  // namespace

TEST(LidarTargetTracker, YoloConfirmsIdentityThenLidarContinues) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.background_confirm_frames = 3;
  config.cluster_min_points = 2;
  config.cluster_voxel_size = 0.15;
  config.track_gate_distance = 0.8;
  config.lidar_lost_time = 1.0;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  tracker.processCloud(targetCloud(3.0), sensor, 0.1);
  ASSERT_FALSE(tracker.candidates().empty());
  const auto target = *std::min_element(
      tracker.candidates().begin(), tracker.candidates().end(),
      [](const auto& a, const auto& b) {
        return std::abs(a.centroid.x - 3.0) < std::abs(b.centroid.x - 3.0);
      });
  ASSERT_TRUE(tracker.confirmIdentity(target, 0.9, 0.1));

  auto estimate = tracker.processCloud(targetCloud(3.1), sensor, 0.2);
  ASSERT_TRUE(estimate.valid);
  EXPECT_EQ(estimate.state, yolo_trt_detector::LidarTrackState::kConfirmed);

  // No further image confirmation: the LiDAR-only updates remain valid.
  estimate = tracker.processCloud(targetCloud(3.2), sensor, 0.3);
  ASSERT_TRUE(estimate.valid);
  EXPECT_GT(estimate.last_visual_confirmation_age, 0.0);
  EXPECT_NEAR(estimate.position.x, 3.2, 0.35);
}

TEST(LidarTargetTracker, StaticBackgroundIsRemovedButProtectedTargetRemains) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.background_confirm_frames = 2;
  config.cluster_min_points = 2;
  config.track_protection_radius = 0.6;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);
  tracker.processCloud(targetCloud(3.0), sensor, 0.1);
  const auto initial_candidates = tracker.candidates();
  ASSERT_FALSE(initial_candidates.empty());
  const auto target = *std::min_element(
      initial_candidates.begin(), initial_candidates.end(),
      [](const auto& a, const auto& b) {
        return std::abs(a.centroid.x - 3.0) < std::abs(b.centroid.x - 3.0);
      });
  ASSERT_TRUE(tracker.confirmIdentity(target, 0.9, 0.1));
  tracker.processCloud(targetCloud(3.0), sensor, 0.2);
  const auto estimate = tracker.processCloud(targetCloud(3.0), sensor, 0.3);
  EXPECT_TRUE(estimate.valid);
}

TEST(LidarTargetTracker, StationaryTargetCanBeAcquiredAfterBackgroundWarmup) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.background_confirm_frames = 2;
  config.cluster_min_points = 2;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);
  for (int frame = 1; frame <= 5; ++frame) {
    tracker.processCloud(targetCloud(3.0), sensor, frame * 0.1);
  }
  const auto& candidates = tracker.candidates();
  ASSERT_FALSE(candidates.empty());
  const auto target = *std::min_element(
      candidates.begin(), candidates.end(), [](const auto& a, const auto& b) {
        return std::abs(a.centroid.x - 3.0) < std::abs(b.centroid.x - 3.0);
      });
  EXPECT_TRUE(tracker.confirmIdentity(target, 0.9, 0.5));
}

TEST(LidarTargetTracker, RequiresVisualReconfirmationAfterLidarLoss) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.lidar_lost_time = 0.25;
  config.cluster_min_points = 2;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);
  tracker.processCloud(targetCloud(3.0, false), sensor, 0.1);
  ASSERT_FALSE(tracker.candidates().empty());
  ASSERT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.9, 0.1));
  EXPECT_TRUE(tracker.processCloud(targetCloud(3.1, false), sensor, 0.2).valid);

  const std::vector<yolo_trt_detector::LidarPoint> no_points;
  const auto lost = tracker.processCloud(no_points, sensor, 0.6);
  EXPECT_FALSE(lost.valid);
  EXPECT_FALSE(tracker.estimate(0.6).identity_confirmed);
}

TEST(LidarTargetTracker, RequiresFiveConsistentVisualFramesBeforeConfirmation) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 5;
  config.visual_confirm_window = 7;
  config.visual_confirm_max_gap = 0.25;
  config.visual_tentative_gate = 0.60;
  config.cluster_min_points = 2;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  for (int frame = 1; frame <= 4; ++frame) {
    const double stamp = frame * 0.1;
    tracker.processCloud(targetCloud(3.0 + 0.02 * frame, false), sensor, stamp);
    ASSERT_FALSE(tracker.candidates().empty());
    EXPECT_FALSE(tracker.confirmIdentity(tracker.candidates().front(), 0.92, stamp));
    const auto tentative = tracker.estimate(stamp);
    EXPECT_EQ(tentative.state, yolo_trt_detector::LidarTrackState::kTentative);
    EXPECT_FALSE(tentative.valid);
  }

  tracker.processCloud(targetCloud(3.10, false), sensor, 0.5);
  ASSERT_FALSE(tracker.candidates().empty());
  EXPECT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.94, 0.5));
  EXPECT_TRUE(tracker.estimate(0.5).identity_confirmed);
}

TEST(LidarTargetTracker, VisualConfirmationHistoryExpiresAcrossLargeGap) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 3;
  config.visual_confirm_window = 5;
  config.visual_confirm_max_gap = 0.25;
  config.cluster_min_points = 2;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  for (double stamp : {0.1, 0.2}) {
    tracker.processCloud(targetCloud(3.0, false), sensor, stamp);
    ASSERT_FALSE(tracker.candidates().empty());
    EXPECT_FALSE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, stamp));
  }
  tracker.processCloud(targetCloud(3.0, false), sensor, 0.8);
  ASSERT_FALSE(tracker.candidates().empty());
  EXPECT_FALSE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, 0.8));
  EXPECT_FALSE(tracker.estimate(0.8).identity_confirmed);
}

TEST(LidarTargetTracker, ConfirmationRequiresEnoughHitsInsideFrameWindow) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 5;
  config.visual_confirm_window = 7;
  config.visual_confirm_max_gap = 0.25;
  config.cluster_min_points = 2;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  int hits = 0;
  for (int frame = 1; frame <= 8; ++frame) {
    const double stamp = frame * 0.1;
    tracker.processCloud(targetCloud(3.0, false), sensor, stamp);
    if (frame == 2 || frame == 4 || frame == 6) {
      tracker.noteVisualFrameWithoutIdentity(stamp);
      continue;
    }
    ASSERT_FALSE(tracker.candidates().empty());
    ++hits;
    EXPECT_FALSE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, stamp));
  }
  EXPECT_EQ(hits, 5);
  EXPECT_FALSE(tracker.estimate(0.8).identity_confirmed);
}

TEST(LidarTargetTracker, RejectsImplausibleLidarTeleportInsteadOfSwitchingObject) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.cluster_min_points = 2;
  config.track_gate_distance = 10.0;
  config.max_target_speed = 3.0;
  config.max_target_acceleration = 8.0;
  config.lidar_lost_time = 0.25;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  tracker.processCloud(targetCloud(3.0, false), sensor, 0.1);
  ASSERT_FALSE(tracker.candidates().empty());
  ASSERT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, 0.1));
  ASSERT_TRUE(tracker.processCloud(targetCloud(3.1, false), sensor, 0.2).valid);

  const auto after_jump = tracker.processCloud(targetCloud(5.0, false), sensor, 0.3);
  EXPECT_TRUE(after_jump.valid);
  EXPECT_LT(after_jump.position.x, 4.0);
  const std::vector<yolo_trt_detector::LidarPoint> no_points;
  EXPECT_FALSE(tracker.processCloud(no_points, sensor, 0.6).valid);
}

TEST(LidarTargetTracker, RejectsRecorded141221WrongObjectJump) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.cluster_min_points = 2;
  config.track_gate_distance = 2.0;
  config.track_gate_max_distance = 0.50;
  config.measurement_max_update_distance = 0.45;
  config.max_target_speed = 4.0;
  config.lidar_lost_time = 0.60;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  tracker.processCloud(targetCloud(1.4836, false), sensor, 143.7586);
  ASSERT_FALSE(tracker.candidates().empty());
  ASSERT_TRUE(tracker.confirmIdentity(
      tracker.candidates().front(), 0.83, 143.7586));

  // The recorded track switched to a side object roughly 0.97 m away in
  // 0.192 s. It must be treated as a missing update, never a valid target.
  const auto after_jump = tracker.processCloud(
      targetCloud(0.7697, false), sensor, 143.9507);
  EXPECT_TRUE(after_jump.valid);
  EXPECT_GT(after_jump.last_lidar_update_age, 0.15);
  EXPECT_GT(after_jump.position.x, 1.0);
}

TEST(LidarTargetTracker, RecentVisualFilterPreventsSwitchToOutsideCandidate) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.cluster_min_points = 2;
  config.track_gate_distance = 1.0;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);
  tracker.processCloud(targetCloud(3.0, false), sensor, 0.1);
  ASSERT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.9, 0.1));

  const auto estimate = tracker.processCloud(
      targetCloud(3.2, false), sensor, 0.2,
      [](const yolo_trt_detector::LidarCluster& candidate) {
        return candidate.centroid.x < 2.0;
      });
  EXPECT_TRUE(estimate.valid);
  EXPECT_GT(estimate.last_lidar_update_age, 0.05);
  EXPECT_NEAR(estimate.position.x, 3.0, 0.25);
}

TEST(LidarTargetTracker, AcceptsSmallCentroidJitterWithoutFalseAccelerationLoss) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.cluster_min_points = 2;
  config.max_target_speed = 3.0;
  config.max_target_acceleration = 8.0;
  config.measurement_velocity_slack = 0.8;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  tracker.processCloud(targetCloud(3.0, false), sensor, 0.1);
  ASSERT_FALSE(tracker.candidates().empty());
  ASSERT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, 0.1));
  ASSERT_TRUE(tracker.processCloud(targetCloud(3.1, false), sensor, 0.2).valid);
  const auto jittered = tracker.processCloud(targetCloud(3.05, false), sensor, 0.3);
  EXPECT_TRUE(jittered.valid);
  EXPECT_NEAR(jittered.last_lidar_update_age, 0.0, 1e-9);
}

TEST(LidarTargetTracker, StopsPublishingWhenVisualIdentityIsTooOld) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.visual_max_age = 0.50;
  config.cluster_min_points = 2;
  config.lidar_lost_time = 2.0;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  tracker.processCloud(targetCloud(3.0, false), sensor, 0.1);
  ASSERT_FALSE(tracker.candidates().empty());
  ASSERT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, 0.1));
  EXPECT_TRUE(tracker.processCloud(targetCloud(3.1, false), sensor, 0.4).valid);
  const auto expired = tracker.processCloud(targetCloud(3.2, false), sensor, 0.7);
  EXPECT_FALSE(expired.valid);
  EXPECT_EQ(expired.state, yolo_trt_detector::LidarTrackState::kLost);
}

TEST(LidarTargetTracker, ContinuousLidarDoesNotExpireWithoutYolo) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.visual_max_age = 0.20;
  config.require_recent_visual_confirmation = false;
  config.cluster_min_points = 2;
  config.lidar_lost_time = 0.25;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  tracker.processCloud(targetCloud(3.0, false), sensor, 0.1);
  ASSERT_FALSE(tracker.candidates().empty());
  ASSERT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, 0.1));
  yolo_trt_detector::LidarTrackEstimate estimate;
  for (int frame = 1; frame <= 14; ++frame) {
    const double stamp = 0.1 + frame * 0.1;
    estimate = tracker.processCloud(
        targetCloud(3.0 + frame * 0.02, false), sensor, stamp);
    ASSERT_TRUE(estimate.valid) << "frame=" << frame;
  }
  EXPECT_GT(estimate.last_visual_confirmation_age, 1.0);
  EXPECT_NEAR(estimate.last_lidar_update_age, 0.0, 1e-9);
}

TEST(LidarTargetTracker, MissingLidarStillStopsWithoutVisualTimeout) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.require_recent_visual_confirmation = false;
  config.cluster_min_points = 2;
  config.lidar_lost_time = 0.25;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  tracker.processCloud(targetCloud(3.0, false), sensor, 0.1);
  ASSERT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, 0.1));
  ASSERT_TRUE(tracker.processCloud(targetCloud(3.05, false), sensor, 0.2).valid);
  const std::vector<yolo_trt_detector::LidarPoint> no_points;
  EXPECT_FALSE(tracker.processCloud(no_points, sensor, 0.5).valid);
}

TEST(LidarTargetTracker, ExceedingMaximumSensorRangeStopsLidarOnlyTrack) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.require_recent_visual_confirmation = false;
  config.cluster_min_points = 2;
  config.max_range = 4.0;
  config.visual_acquire_max_range = 4.0;
  yolo_trt_detector::LidarTargetTracker tracker(config);

  const cv::Point3d initial_sensor(0.0, 0.0, 0.0);
  tracker.processCloud(targetCloud(3.0, false), initial_sensor, 0.1);
  ASSERT_TRUE(tracker.confirmIdentity(tracker.candidates().front(), 0.95, 0.1));
  const cv::Point3d far_sensor(-2.0, 0.0, 0.0);
  EXPECT_FALSE(tracker.processCloud(
      targetCloud(3.0, false), far_sensor, 0.2).valid);
}

TEST(LidarTargetTracker, PointCountMustMatchTargetSizeAndRange) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.cluster_min_points = 2;
  config.enforce_range_point_count = true;
  config.target_physical_size = 0.55;
  config.point_density_at_1m = 300.0;
  config.point_count_min_ratio = 0.15;
  config.point_count_max_ratio = 5.0;
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  yolo_trt_detector::LidarTargetTracker at_three_meters(config);
  at_three_meters.processCloud(targetCloud(3.0, false), sensor, 0.1);
  EXPECT_FALSE(at_three_meters.candidates().empty());

  // The same dense 25-point return at 8 m is the wrong angular/physical
  // scale for a 0.55 m aircraft and is more likely a merged background object.
  yolo_trt_detector::LidarTargetTracker at_eight_meters(config);
  at_eight_meters.processCloud(targetCloud(8.0, false), sensor, 0.1);
  EXPECT_TRUE(at_eight_meters.candidates().empty());
}

TEST(LidarTargetTracker, RejectsVisualAcquisitionBeyondSafeRange) {
  yolo_trt_detector::LidarTrackerConfig config;
  config.visual_confirm_hits = 1;
  config.visual_acquire_max_range = 8.0;
  config.max_range = 18.0;
  config.cluster_min_points = 2;
  yolo_trt_detector::LidarTargetTracker tracker(config);
  const cv::Point3d sensor(0.0, 0.0, 0.0);

  tracker.processCloud(targetCloud(12.0, false), sensor, 0.1);
  ASSERT_FALSE(tracker.candidates().empty());
  EXPECT_FALSE(tracker.confirmIdentity(tracker.candidates().front(), 0.99, 0.1));
  EXPECT_FALSE(tracker.estimate(0.1).identity_confirmed);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
