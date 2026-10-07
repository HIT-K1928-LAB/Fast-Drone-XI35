#include <gtest/gtest.h>

#include <vector>

#include <yolo_trt_detector/visual_lidar_match.h>

TEST(VisualLidarMatch, AcceptsOneClearCenteredCandidate) {
  yolo_trt_detector::VisualLidarMatchConfig config;
  const std::vector<yolo_trt_detector::VisualLidarMatchCandidate> candidates{
      {0, 4, 0.12, 0.10},
      {0, 7, 0.48, 0.46},
  };

  EXPECT_EQ(yolo_trt_detector::selectVisualLidarMatch(
                candidates, 1, false, config),
            0);
}

TEST(VisualLidarMatch, RejectsMultipleDetectionsDuringIdentityAcquisition) {
  yolo_trt_detector::VisualLidarMatchConfig config;
  const std::vector<yolo_trt_detector::VisualLidarMatchCandidate> candidates{
      {0, 4, 0.10, 0.08},
      {1, 9, 0.08, 0.07},
  };

  EXPECT_EQ(yolo_trt_detector::selectVisualLidarMatch(
                candidates, 2, false, config),
            -1);
}

TEST(VisualLidarMatch, RejectsCandidateOutsideDetectionCentralRegion) {
  yolo_trt_detector::VisualLidarMatchConfig config;
  config.max_normalized_center_distance = 0.35;
  const std::vector<yolo_trt_detector::VisualLidarMatchCandidate> candidates{
      {0, 4, 0.42, 0.15},
  };

  EXPECT_EQ(yolo_trt_detector::selectVisualLidarMatch(
                candidates, 1, false, config),
            -1);
}

TEST(VisualLidarMatch, RejectsTwoNearlyEqualClustersAsAmbiguous) {
  yolo_trt_detector::VisualLidarMatchConfig config;
  config.minimum_rank_separation = 0.12;
  const std::vector<yolo_trt_detector::VisualLidarMatchCandidate> candidates{
      {0, 4, 0.10, 0.08},
      {0, 5, 0.13, 0.16},
  };

  EXPECT_EQ(yolo_trt_detector::selectVisualLidarMatch(
                candidates, 1, false, config),
            -1);
}

TEST(VisualLidarMatch, AllowsMultipleImageDetectionsAfterIdentityIsKnown) {
  yolo_trt_detector::VisualLidarMatchConfig config;
  const std::vector<yolo_trt_detector::VisualLidarMatchCandidate> candidates{
      {1, 6, 0.09, 0.05},
      {0, 3, 0.18, 0.30},
  };

  EXPECT_EQ(yolo_trt_detector::selectVisualLidarMatch(
                candidates, 2, true, config),
            0);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
