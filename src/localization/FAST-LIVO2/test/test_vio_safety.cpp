#include <gtest/gtest.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cmath>
#include <limits>

#include "image_patch_utils.h"
#include "vio.h"

TEST(ImagePatchBounds, AcceptsTheExactMaximumRawAccessExtent)
{
  const cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC1);

  EXPECT_TRUE(fast_livo::isPatchAccessInFrame(
      image, Eigen::Vector2d(160.0, 160.0), 8, 4, 32));
  EXPECT_TRUE(fast_livo::isPatchAccessInFrame(
      image, Eigen::Vector2d(320.0, 240.0), 8, 4, 32));
}

TEST(ImagePatchBounds, RejectsPointsWhoseGradientTapsCrossTheImageEdge)
{
  const cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC1);

  EXPECT_FALSE(fast_livo::isPatchAccessInFrame(
      image, Eigen::Vector2d(159.0, 240.0), 8, 4, 32));
  EXPECT_FALSE(fast_livo::isPatchAccessInFrame(
      image, Eigen::Vector2d(320.0, 320.0), 8, 4, 32));
}

TEST(ImagePatchBounds, RejectsInvalidImagesCoordinatesAndScale)
{
  const cv::Mat gray = cv::Mat::zeros(480, 640, CV_8UC1);
  const cv::Mat color = cv::Mat::zeros(480, 640, CV_8UC3);
  const Eigen::Vector2d center(320.0, 240.0);

  EXPECT_FALSE(fast_livo::isPatchAccessInFrame(
      cv::Mat(), center, 8, 4, 1));
  EXPECT_FALSE(fast_livo::isPatchAccessInFrame(
      color, center, 8, 4, 1));
  EXPECT_FALSE(fast_livo::isPatchAccessInFrame(
      gray, Eigen::Vector2d(std::numeric_limits<double>::quiet_NaN(), 240.0), 8, 4, 1));
  EXPECT_FALSE(fast_livo::isPatchAccessInFrame(
      gray, center, 8, 4, 0));
}

TEST(ImagePatchBounds, ReturnsTheExactAlignedOriginUsedForRawAddressing)
{
  const cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC1);
  int aligned_u = -1;
  int aligned_v = -1;

  ASSERT_TRUE(fast_livo::getPatchAccessOrigin(
      image, Eigen::Vector2d(634.99999999, 240.0), 8, 4, 1,
      aligned_u, aligned_v));
  EXPECT_EQ(634, aligned_u);
  EXPECT_EQ(240, aligned_v);
}

TEST(VioImagePatch, UsesTheCvMatRowStrideAndReportsInvalidAccess)
{
  cv::Mat storage(40, 40, CV_8UC1);
  for (int row = 0; row < storage.rows; ++row)
  {
    for (int col = 0; col < storage.cols; ++col)
      storage.at<uint8_t>(row, col) = static_cast<uint8_t>((row * 7 + col * 3) % 251);
  }
  const cv::Mat image = storage(cv::Rect(4, 4, 32, 32));
  ASSERT_NE(image.step[0], static_cast<size_t>(image.cols));

  VIOManager vio;
  vio.patch_size = 8;
  vio.patch_size_half = 4;
  vio.patch_size_total = 64;
  vio.width = image.cols;
  std::vector<float> patch(64, -1.0f);

  ASSERT_TRUE(vio.getImagePatch(image, Eigen::Vector2d(16.0, 16.0), patch.data(), 0));
  for (int row = 0; row < 8; ++row)
  {
    for (int col = 0; col < 8; ++col)
      EXPECT_FLOAT_EQ(image.at<uint8_t>(12 + row, 12 + col), patch[row * 8 + col]);
  }

  std::fill(patch.begin(), patch.end(), -1.0f);
  EXPECT_FALSE(vio.getImagePatch(image, Eigen::Vector2d(2.0, 2.0), patch.data(), 0));
  EXPECT_TRUE(std::all_of(patch.begin(), patch.end(), [](float value) { return value == -1.0f; }));
}

TEST(VioEkfUpdate, LeavesStateAndCovarianceUntouchedWhenAllProjectionsAreInvalid)
{
  vk::PinholeCamera camera(640, 480, 1.0, 320, 320, 320, 240);
  StatesGroup state;
  StatesGroup propagated = state;
  VIOManager vio;
  vio.cam = &camera;
  vio.state = &state;
  vio.state_propagat = &propagated;
  vio.grid_size = 20;
  vio.raycast_en = false;
  vio.colmap_output_en = false;
  vio.patch_size = 8;
  vio.patch_pyrimid_level = 1;
  vio.Rcl.setIdentity();
  vio.Rli.setIdentity();
  vio.Pcl.setZero();
  vio.Pli.setZero();
  vio.inverse_composition_en = false;
  vio.exposure_estimate_en = true;
  vio.max_iterations = 2;
  vio.img_point_cov = 100.0;
  vio.initializeVIO();

  const cv::Mat image = cv::Mat::zeros(480, 640, CV_8UC1);
  vio.new_frame_.reset(new Frame(&camera, image));
  std::unique_ptr<VisualPoint> point(new VisualPoint(Eigen::Vector3d(0.0, 100.0, 1.0)));
  vio.visual_submap->voxel_points.push_back(point.get());
  vio.visual_submap->search_levels.push_back(0);
  vio.visual_submap->warp_patch.push_back(std::vector<float>(64, 0.0f));
  vio.visual_submap->inv_expo_list.push_back(1.0);
  vio.visual_submap->errors.push_back(0.0f);
  vio.visual_submap->propa_errors.push_back(0.0f);
  vio.total_points = 1;
  vio.G.setOnes();

  const StatesGroup state_before = state;
  EXPECT_FALSE(vio.computeJacobianAndUpdateEKF(image));
  EXPECT_TRUE((state - state_before).isZero());
  EXPECT_TRUE(state.cov.isApprox(state_before.cov));
  EXPECT_TRUE(vio.G.isZero());

  vio.inverse_composition_en = true;
  vio.G.setOnes();
  EXPECT_FALSE(vio.computeJacobianAndUpdateEKF(image));
  EXPECT_TRUE((state - state_before).isZero());
  EXPECT_TRUE(state.cov.isApprox(state_before.cov));
  EXPECT_TRUE(vio.G.isZero());

  vio.visual_submap->voxel_points.clear();
}

TEST(VioEkfUpdate, ReturnsWithoutChangingGainWhenThereAreNoPoints)
{
  VIOManager vio;
  vio.total_points = 0;
  vio.G.setOnes();

  EXPECT_FALSE(vio.computeJacobianAndUpdateEKF(cv::Mat()));
  EXPECT_TRUE(vio.G.isZero());
}

int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
