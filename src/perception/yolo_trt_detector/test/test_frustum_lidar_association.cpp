#include <gtest/gtest.h>

#include <yolo_trt_detector/frustum_lidar_association.h>

namespace {

yolo_trt_detector::ProjectedLidarPoint point(
    double x, double y, double z, double u, double v, double depth) {
  yolo_trt_detector::ProjectedLidarPoint result;
  result.position = cv::Point3d(x, y, z);
  result.pixel = cv::Point2d(u, v);
  result.optical_depth = depth;
  return result;
}

void addPatch(
    std::vector<yolo_trt_detector::ProjectedLidarPoint>* points,
    double depth, double center_u, double center_v, double world_x) {
  for (int x = -2; x <= 2; ++x) {
    for (int y = -2; y <= 2; ++y) {
      points->push_back(point(
          world_x, x * 0.025, 0.5 + y * 0.025,
          center_u + x * 3.0, center_v + y * 3.0, depth + x * 0.005));
    }
  }
}

}  // namespace

TEST(FrustumLidarAssociation, SelectsForegroundTargetInsteadOfBackgroundWall) {
  std::vector<yolo_trt_detector::ProjectedLidarPoint> points;
  addPatch(&points, 3.0, 320.0, 240.0, 3.0);
  addPatch(&points, 6.0, 320.0, 240.0, 6.0);

  yolo_trt_detector::FrustumLidarConfig config;
  config.cluster_radius = 0.12;
  config.min_points = 3;
  const auto candidates = yolo_trt_detector::clusterPointsInsideDetection(
      points, cv::Rect(280, 200, 80, 80), config);
  ASSERT_EQ(candidates.size(), 2u);
  const int selected = yolo_trt_detector::selectForegroundFrustumCluster(
      candidates, cv::Rect(280, 200, 80, 80), config);
  ASSERT_GE(selected, 0);
  EXPECT_NEAR(candidates[selected].cluster.centroid.x, 3.0, 0.1);
}

TEST(FrustumLidarAssociation, IgnoresLargeObjectOutsideImageDetection) {
  std::vector<yolo_trt_detector::ProjectedLidarPoint> points;
  addPatch(&points, 3.0, 320.0, 240.0, 3.0);
  for (int i = 0; i < 50; ++i) {
    points.push_back(point(1.0, -1.0 + i * 0.02, 0.5, 80.0, 240.0, 1.0));
  }
  yolo_trt_detector::FrustumLidarConfig config;
  const auto candidates = yolo_trt_detector::clusterPointsInsideDetection(
      points, cv::Rect(280, 200, 80, 80), config);
  ASSERT_EQ(candidates.size(), 1u);
  EXPECT_NEAR(candidates.front().cluster.centroid.x, 3.0, 0.1);
}

TEST(FrustumLidarAssociation, RejectsDepthAmbiguityInsteadOfGuessing) {
  std::vector<yolo_trt_detector::ProjectedLidarPoint> points;
  addPatch(&points, 3.00, 316.0, 240.0, 3.00);
  addPatch(&points, 3.08, 324.0, 240.0, 3.08);
  yolo_trt_detector::FrustumLidarConfig config;
  config.cluster_radius = 0.06;
  config.minimum_rank_separation = 0.15;
  const auto candidates = yolo_trt_detector::clusterPointsInsideDetection(
      points, cv::Rect(280, 200, 80, 80), config);
  ASSERT_EQ(candidates.size(), 2u);
  EXPECT_EQ(yolo_trt_detector::selectForegroundFrustumCluster(
                candidates, cv::Rect(280, 200, 80, 80), config),
            -1);
}

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
