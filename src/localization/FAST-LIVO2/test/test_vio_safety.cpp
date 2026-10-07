#include <gtest/gtest.h>

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cmath>
#include <limits>

#include "body_pose_extrinsic.h"
#include "aligned_point_cloud.h"
#include "image_patch_utils.h"
#include "initial_heading_alignment.h"
#include "output_path.h"
#include "pointcloud_publication.h"
#include "vio.h"

namespace
{

fast_livo::BodyPoseExtrinsic makeFcuFromLivoxCalibration()
{
  Eigen::Matrix3d rotation_fcu_livox;
  rotation_fcu_livox <<
      0.928719158327664, 0.028186162369961323, 0.3697110563751235,
     -0.025166576674429137, 0.9995988842847866, -0.012988993613829738,
     -0.36992886934334507, 0.0027587655689106172, 0.9290559836946805;
  const Eigen::Vector3d translation_fcu_livox(
      -0.030591146664829438,
      -0.036910287242114814,
      -0.03524249759870778);
  return fast_livo::BodyPoseExtrinsic::FromTargetReference(
      rotation_fcu_livox, translation_fcu_livox);
}

}  // namespace

TEST(PcdOutputPath, UsesTmuxSessionLogRootWhenAvailable)
{
  EXPECT_EQ(
      std::filesystem::path(
          "/root/Fast-Drone-XI35/log/2026-09-19_10-25-42_orin-lidar-01_lidar/fastlivo2/pcd"),
      fast_livo::resolvePcdOutputDirectory(
          "/root/Fast-Drone-XI35/log/2026-09-19_10-25-42_orin-lidar-01_lidar",
          "/root/Fast-Drone-XI35/src/localization/FAST-LIVO2"));
}

TEST(PcdOutputPath, FallsBackToPackageLogDirectoryWithoutTmuxEnvironment)
{
  const std::filesystem::path package_root(
      "/root/Fast-Drone-XI35/src/localization/FAST-LIVO2");

  EXPECT_EQ(
      package_root / "Log" / "pcd",
      fast_livo::resolvePcdOutputDirectory(nullptr, package_root));
  EXPECT_EQ(
      package_root / "Log" / "pcd",
      fast_livo::resolvePcdOutputDirectory("", package_root));
}

TEST(BodyPoseExtrinsic, InvertsKalibrTargetFromReferenceConvention)
{
  const fast_livo::BodyPoseExtrinsic extrinsic =
      makeFcuFromLivoxCalibration();

  Eigen::Matrix3d rotation_fcu_livox;
  rotation_fcu_livox <<
      0.928719158327664, 0.028186162369961323, 0.3697110563751235,
     -0.025166576674429137, 0.9995988842847866, -0.012988993613829738,
     -0.36992886934334507, 0.0027587655689106172, 0.9290559836946805;
  const Eigen::Vector3d translation_fcu_livox(
      -0.030591146664829438,
      -0.036910287242114814,
      -0.03524249759870778);

  EXPECT_TRUE((rotation_fcu_livox * extrinsic.rotationReferenceTarget())
                  .isApprox(Eigen::Matrix3d::Identity(), 1e-9));
  EXPECT_TRUE((rotation_fcu_livox * extrinsic.translationReferenceTarget() +
               translation_fcu_livox)
                  .isZero(1e-9));
}

TEST(BodyPoseExtrinsic, TransformsPoseAndLeverArmVelocityToTargetFrame)
{
  const fast_livo::BodyPoseExtrinsic extrinsic =
      makeFcuFromLivoxCalibration();
  fast_livo::RigidBodyState reference_state;
  reference_state.rotation =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  reference_state.position = Eigen::Vector3d(1.0, 2.0, 3.0);
  reference_state.velocity = Eigen::Vector3d(4.0, 5.0, 6.0);
  const Eigen::Vector3d angular_velocity_reference(0.0, 0.0, 2.0);

  const fast_livo::RigidBodyState target_state = extrinsic.transform(
      reference_state, angular_velocity_reference);

  EXPECT_TRUE(target_state.rotation.isApprox(
      reference_state.rotation * extrinsic.rotationReferenceTarget(), 1e-9));
  EXPECT_TRUE(target_state.position.isApprox(
      reference_state.position +
          reference_state.rotation * extrinsic.translationReferenceTarget(),
      1e-9));
  EXPECT_TRUE(target_state.velocity.isApprox(
      reference_state.velocity + reference_state.rotation *
          angular_velocity_reference.cross(
              extrinsic.translationReferenceTarget()),
      1e-9));
}

TEST(InitialHeadingAlignment, ZerosInitialBodyYawAndRotatesWorldVectors)
{
  fast_livo::RigidBodyState body_state;
  body_state.rotation =
      Eigen::AngleAxisd(M_PI / 3.0, Eigen::Vector3d::UnitZ())
          .toRotationMatrix();
  body_state.position = Eigen::Vector3d(1.0, 0.0, 2.0);
  body_state.velocity = Eigen::Vector3d(0.0, 2.0, 0.0);

  const Eigen::Matrix3d world_alignment =
      fast_livo::worldRotationToZeroInitialBodyYaw(body_state.rotation);
  const fast_livo::RigidBodyState aligned =
      fast_livo::rotateWorldFrame(body_state, world_alignment);

  EXPECT_NEAR(fast_livo::yawFromRotation(aligned.rotation), 0.0, 1e-12);
  EXPECT_TRUE(aligned.position.isApprox(
      Eigen::Vector3d(0.5, -std::sqrt(3.0) / 2.0, 2.0), 1e-12));
  EXPECT_TRUE(aligned.velocity.isApprox(
      Eigen::Vector3d(std::sqrt(3.0), 1.0, 0.0), 1e-12));
}

TEST(InitialHeadingAlignment, ZerosInitialBodyPositionAndYaw)
{
  fast_livo::RigidBodyState initial_state;
  initial_state.rotation =
      (Eigen::AngleAxisd(40.0 * M_PI / 180.0, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-8.0 * M_PI / 180.0, Eigen::Vector3d::UnitY()) *
       Eigen::AngleAxisd(5.0 * M_PI / 180.0, Eigen::Vector3d::UnitX()))
          .toRotationMatrix();
  initial_state.position = Eigen::Vector3d(1.2, -0.7, 0.3);
  initial_state.velocity = Eigen::Vector3d(0.4, -0.2, 0.1);

  const Eigen::Matrix3d world_alignment =
      fast_livo::worldRotationToZeroInitialBodyYaw(initial_state.rotation);
  const fast_livo::RigidBodyState aligned =
      fast_livo::alignWorldFrameAtInitialPosition(
          initial_state, world_alignment, initial_state.position);

  EXPECT_NEAR(fast_livo::yawFromRotation(aligned.rotation), 0.0, 1e-12);
  EXPECT_TRUE(aligned.position.isZero(1e-12));
  EXPECT_NEAR(
      aligned.rotation.col(2).z(), initial_state.rotation.col(2).z(), 1e-12);
  EXPECT_TRUE(aligned.velocity.isApprox(
      world_alignment * initial_state.velocity, 1e-12));
}

TEST(InitialHeadingAlignment, UsesFcuBodyAfterLivoxExtrinsic)
{
  const fast_livo::BodyPoseExtrinsic extrinsic =
      makeFcuFromLivoxCalibration();
  fast_livo::RigidBodyState livox_state;
  livox_state.rotation =
      (Eigen::AngleAxisd(0.8, Eigen::Vector3d::UnitZ()) *
       Eigen::AngleAxisd(-0.2, Eigen::Vector3d::UnitY()))
          .toRotationMatrix();
  livox_state.position = Eigen::Vector3d(2.0, -1.0, 0.4);
  livox_state.velocity = Eigen::Vector3d(0.5, -0.3, 0.1);

  const fast_livo::RigidBodyState initial_fcu_state =
      extrinsic.transform(livox_state);
  const Eigen::Matrix3d world_alignment =
      fast_livo::worldRotationToZeroInitialBodyYaw(
          initial_fcu_state.rotation);
  const fast_livo::RigidBodyState aligned_livox_state =
      fast_livo::rotateWorldFrame(livox_state, world_alignment);
  const fast_livo::RigidBodyState aligned_fcu_state =
      extrinsic.transform(aligned_livox_state);

  EXPECT_NEAR(
      fast_livo::yawFromRotation(aligned_fcu_state.rotation), 0.0, 1e-12);
}

TEST(AlignedPointCloud, MatchesBodyOdometryWorldAndPreservesOtherFields)
{
  sensor_msgs::PointCloud2 cloud;
  cloud.header.frame_id = "camera_init";
  cloud.header.stamp.fromSec(12.5);
  sensor_msgs::PointCloud2Modifier modifier(cloud);
  modifier.setPointCloud2Fields(
      4, "x", 1, sensor_msgs::PointField::FLOAT32,
      "y", 1, sensor_msgs::PointField::FLOAT32,
      "z", 1, sensor_msgs::PointField::FLOAT32,
      "intensity", 1, sensor_msgs::PointField::FLOAT32);
  modifier.resize(2);

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
  *x = 2.0f; *y = 2.0f; *z = 3.0f; *intensity = 42.0f;
  ++x; ++y; ++z; ++intensity;
  *x = 1.0f; *y = 4.0f; *z = 4.0f; *intensity = 7.0f;

  const Eigen::Matrix3d rotation =
      Eigen::AngleAxisd(M_PI / 2.0, Eigen::Vector3d::UnitZ()).toRotationMatrix();
  const Eigen::Vector3d initial_body_position(1.0, 2.0, 3.0);
  ASSERT_TRUE(fast_livo::alignWorldPointCloudInPlace(
      cloud, rotation, initial_body_position));

  sensor_msgs::PointCloud2ConstIterator<float> ax(cloud, "x");
  sensor_msgs::PointCloud2ConstIterator<float> ay(cloud, "y");
  sensor_msgs::PointCloud2ConstIterator<float> az(cloud, "z");
  sensor_msgs::PointCloud2ConstIterator<float> ai(cloud, "intensity");
  EXPECT_NEAR(*ax, 0.0, 1e-6);
  EXPECT_NEAR(*ay, 1.0, 1e-6);
  EXPECT_NEAR(*az, 0.0, 1e-6);
  EXPECT_FLOAT_EQ(*ai, 42.0f);
  ++ax; ++ay; ++az; ++ai;
  EXPECT_NEAR(*ax, -2.0, 1e-6);
  EXPECT_NEAR(*ay, 0.0, 1e-6);
  EXPECT_NEAR(*az, 1.0, 1e-6);
  EXPECT_FLOAT_EQ(*ai, 7.0f);
  EXPECT_EQ(cloud.header.frame_id, "camera_init");
  EXPECT_DOUBLE_EQ(cloud.header.stamp.toSec(), 12.5);

  fast_livo::RigidBodyState raw_body;
  raw_body.position = Eigen::Vector3d(2.0, 2.0, 3.0);
  const auto aligned_body = fast_livo::alignWorldFrameAtInitialPosition(
      raw_body, rotation, initial_body_position);
  EXPECT_NEAR(aligned_body.position.x(), 0.0, 1e-12);
  EXPECT_NEAR(aligned_body.position.y(), 1.0, 1e-12);
}

TEST(AlignedPointCloud, RejectsCloudWithoutFloat32XYZ)
{
  sensor_msgs::PointCloud2 cloud;
  EXPECT_FALSE(fast_livo::alignWorldPointCloudInPlace(
      cloud, Eigen::Matrix3d::Identity(), Eigen::Vector3d::Zero()));
}

TEST(PointCloudPublication, RejectsMessagesWithoutPoints)
{
  sensor_msgs::PointCloud2 cloud;

  EXPECT_FALSE(fast_livo::hasPublishablePoints(cloud));

  cloud.width = 1;
  EXPECT_FALSE(fast_livo::hasPublishablePoints(cloud));
}

TEST(PointCloudPublication, AcceptsMessagesWithPointsAndPayload)
{
  sensor_msgs::PointCloud2 cloud;
  cloud.width = 1;
  cloud.height = 1;
  cloud.point_step = 16;
  cloud.row_step = 16;
  cloud.data.resize(16);

  EXPECT_TRUE(fast_livo::hasPublishablePoints(cloud));
}

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
