#include <gtest/gtest.h>

#include <array>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

#include "yopo_planner/yopo_planner.h"

namespace {

nav_msgs::Odometry makeOdom(double x, double vx, double stamp) {
    nav_msgs::Odometry odom;
    odom.header.stamp.fromSec(stamp);
    odom.pose.pose.position.x = x;
    odom.pose.pose.position.z = 1.0;
    odom.pose.pose.orientation.w = 1.0;
    odom.twist.twist.linear.x = vx;
    return odom;
}

yopo_planner::YopoParams trackingParams() {
    yopo_planner::YopoParams params;
    params.goal_yaw_alignment_enabled = true;
    params.terminal_approach_enabled = false;
    params.ctrl_dt = 0.02;
    params.dynamic_target_standoff_distance = 0.0;
    return params;
}

void installTrajectory(yopo_planner::YopoPlanner* planner) {
    std::array<float, 1 * 9 * 3 * 5> endstate{};
    std::array<float, 1 * 3 * 5> scores{};
    planner->prepareObsInput();
    planner->updateTrajectory(endstate, scores, false);
    ASSERT_TRUE(planner->hasTrajectory());
}

TEST(YopoDynamicTarget, RollingUpdatePreservesActiveReference) {
    yopo_planner::YopoPlanner planner(trackingParams());
    planner.updateOdometry(makeOdom(0.0, 1.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(10.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    installTrajectory(&planner);

    quadrotor_msgs::PositionCommand cmd;
    ASSERT_TRUE(planner.fillControlCommand(&cmd));
    const double control_time_before = planner.controlTime();

    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(10.5, 0.2, 1.0), Eigen::Vector3d(0.5, 0.0, 0.0)));
    EXPECT_TRUE(planner.hasTrajectory());
    EXPECT_DOUBLE_EQ(planner.controlTime(), control_time_before);
    EXPECT_FALSE(planner.goalAlignmentActive());
    EXPECT_LT((planner.goal() - Eigen::Vector3d(10.5, 0.2, 1.0)).norm(), 1e-12);
}

TEST(YopoDynamicTarget, RearwardUpdatesDoNotRestartPerGoalYawAlignment) {
    yopo_planner::YopoPlanner planner(trackingParams());
    planner.updateOdometry(makeOdom(0.0, 0.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    EXPECT_FALSE(planner.goalAlignmentActive());

    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(-10.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    EXPECT_FALSE(planner.goalAlignmentActive());

    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(-9.0, 1.0, 1.0), Eigen::Vector3d::Zero()));
    EXPECT_FALSE(planner.goalAlignmentActive());
}

TEST(YopoDynamicTarget, LossCreatesOneSmoothTerminalHold) {
    yopo_planner::YopoPlanner planner(trackingParams());
    planner.updateOdometry(makeOdom(0.0, 1.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(10.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    installTrajectory(&planner);

    quadrotor_msgs::PositionCommand before;
    ASSERT_TRUE(planner.fillControlCommand(&before));
    EXPECT_TRUE(planner.handleDynamicTargetLost());
    EXPECT_FALSE(planner.handleDynamicTargetLost());
    EXPECT_TRUE(planner.dynamicTargetLost());
    EXPECT_TRUE(planner.hasTrajectory());

    quadrotor_msgs::PositionCommand braking;
    ASSERT_TRUE(planner.fillControlCommand(&braking));
    EXPECT_TRUE(std::isfinite(braking.position.x));
    EXPECT_TRUE(std::isfinite(braking.velocity.x));
    EXPECT_LT(std::abs(braking.position.x - before.position.x), 0.2);
}

TEST(YopoDynamicTarget, YawFacesTargetWithRateAndAccelerationLimits) {
    auto params = trackingParams();
    params.dynamic_target_yaw_rate = 1.2;
    params.dynamic_target_yaw_accel = 2.5;
    params.dynamic_target_yaw_deadband_deg = 0.0;
    yopo_planner::YopoPlanner planner(params);
    planner.updateOdometry(makeOdom(0.0, 0.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(10.0, 10.0, 1.0), Eigen::Vector3d::Zero()));
    installTrajectory(&planner);

    double previous_rate = 0.0;
    quadrotor_msgs::PositionCommand cmd;
    for (int i = 0; i < 50; ++i) {
        ASSERT_TRUE(planner.fillControlCommand(&cmd));
        EXPECT_LE(std::abs(cmd.yaw_dot), params.dynamic_target_yaw_rate + 1e-9);
        EXPECT_LE(
            std::abs(cmd.yaw_dot - previous_rate),
            params.dynamic_target_yaw_accel * params.ctrl_dt + 1e-9);
        previous_rate = cmd.yaw_dot;
    }
    EXPECT_GT(cmd.yaw, 0.3);
}

TEST(YopoDynamicTarget, CoastingScaleShortensSelectedPrimitive) {
    yopo_planner::YopoPlanner full_speed(trackingParams());
    yopo_planner::YopoPlanner coasting(trackingParams());
    const auto odom = makeOdom(0.0, 0.0, 1.0);
    full_speed.updateOdometry(odom);
    coasting.updateOdometry(odom);
    full_speed.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    coasting.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    ASSERT_TRUE(full_speed.updateDynamicTarget(
        Eigen::Vector3d(10.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    ASSERT_TRUE(coasting.updateDynamicTarget(
        Eigen::Vector3d(10.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    coasting.setDynamicTargetTrackingScale(0.35);
    installTrajectory(&full_speed);
    installTrajectory(&coasting);

    quadrotor_msgs::PositionCommand full_cmd;
    quadrotor_msgs::PositionCommand coast_cmd;
    for (int i = 0; i < 50; ++i) {
        ASSERT_TRUE(full_speed.fillControlCommand(&full_cmd));
        ASSERT_TRUE(coasting.fillControlCommand(&coast_cmd));
    }
    const double full_distance = std::hypot(full_cmd.position.x, full_cmd.position.y);
    const double coast_distance = std::hypot(coast_cmd.position.x, coast_cmd.position.y);
    EXPECT_LT(coast_distance, full_distance);
}

TEST(YopoDynamicTarget, TargetYawCompensatesCameraForwardOffset) {
    auto params = trackingParams();
    params.velocity = 1.0;
    params.dynamic_target_yaw_deadband_deg = 0.0;
    yopo_planner::YopoPlanner planner(params);
    planner.setCameraToBodyExtrinsic(
        Eigen::AngleAxisd(-0.1, Eigen::Vector3d::UnitZ()).toRotationMatrix());
    planner.updateOdometry(makeOdom(0.0, 0.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(10.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    installTrajectory(&planner);

    quadrotor_msgs::PositionCommand cmd;
    for (int i = 0; i < 100; ++i) {
        ASSERT_TRUE(planner.fillControlCommand(&cmd));
    }
    EXPECT_NEAR(cmd.yaw, 0.1, 0.02);
}

TEST(YopoDynamicTarget, UsesStandoffPointAsNetworkGoal) {
    auto params = trackingParams();
    params.dynamic_target_standoff_distance = 1.2;
    yopo_planner::YopoPlanner planner(params);
    planner.updateOdometry(makeOdom(0.0, 0.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(10.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    EXPECT_NEAR(planner.goal().x(), 8.8, 1e-9);
    EXPECT_NEAR(planner.goal().y(), 0.0, 1e-9);
    EXPECT_FALSE(planner.dynamicStandoffHoldActive());
}

TEST(YopoDynamicTarget, BrakesAndHoldsInsideStandoffRadius) {
    auto params = trackingParams();
    params.dynamic_target_standoff_distance = 1.2;
    params.dynamic_target_standoff_hysteresis = 0.25;
    yopo_planner::YopoPlanner planner(params);
    planner.updateOdometry(makeOdom(0.0, 0.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));

    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(1.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    EXPECT_TRUE(planner.dynamicStandoffHoldActive());
    EXPECT_TRUE(planner.terminalApproachActive());
    quadrotor_msgs::PositionCommand hold;
    ASSERT_TRUE(planner.fillControlCommand(&hold));
    EXPECT_NEAR(hold.position.x, 0.0, 1e-6);

    ASSERT_TRUE(planner.updateDynamicTarget(
        Eigen::Vector3d(2.0, 0.0, 1.0), Eigen::Vector3d::Zero()));
    EXPECT_FALSE(planner.dynamicStandoffHoldActive());
    EXPECT_FALSE(planner.terminalApproachActive());
    EXPECT_NEAR(planner.goal().x(), 0.8, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
