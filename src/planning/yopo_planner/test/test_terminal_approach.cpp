#include <gtest/gtest.h>

#include <cmath>
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

yopo_planner::YopoParams terminalParams() {
    yopo_planner::YopoParams params;
    params.goal_yaw_alignment_enabled = false;
    params.terminal_approach_enabled = true;
    params.terminal_brake_accel = 3.0;
    params.terminal_brake_margin = 0.5;
    params.arrive_distance = 1.5;
    params.terminal_position_tolerance = 0.25;
    params.terminal_speed_tolerance = 0.2;
    params.terminal_stable_time = 0.3;
    params.ctrl_dt = 0.02;
    return params;
}

TEST(YopoTerminalApproach, FastEntryDoesNotLatchEntryPoint) {
    yopo_planner::YopoPlanner planner(terminalParams());
    planner.updateOdometry(makeOdom(3.0, 5.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    EXPECT_FALSE(planner.terminalApproachActive());

    planner.updateOdometry(makeOdom(3.4, 5.0, 1.02));
    ASSERT_TRUE(planner.terminalApproachActive());
    EXPECT_FALSE(planner.arrived());
    quadrotor_msgs::PositionCommand cmd;
    ASSERT_TRUE(planner.fillControlCommand(&cmd));
    EXPECT_GT(cmd.velocity.x, 0.0);
    EXPECT_LT(cmd.position.x, 10.0);

    planner.updateOdometry(makeOdom(8.8, 4.0, 1.04));
    EXPECT_FALSE(planner.arrived());
    for (int i = 0; i < 500; ++i) {
        ASSERT_TRUE(planner.fillControlCommand(&cmd));
    }
    EXPECT_NEAR(cmd.position.x, 10.0, 1e-6);
    EXPECT_NEAR(cmd.velocity.x, 0.0, 1e-6);
    EXPECT_FALSE(planner.arrived());

    for (int i = 0; i < 20; ++i) {
        planner.updateOdometry(makeOdom(10.05, 0.05, 1.06 + i * 0.02));
    }
    ASSERT_TRUE(planner.arrived());
    ASSERT_TRUE(planner.fillControlCommand(&cmd));
    EXPECT_NEAR(cmd.position.x, 10.0, 1e-9);
    EXPECT_DOUBLE_EQ(cmd.velocity.x, 0.0);
}

TEST(YopoTerminalApproach, NewGoalAndFsmResyncClearOldTerminalReference) {
    yopo_planner::YopoPlanner planner(terminalParams());
    planner.updateOdometry(makeOdom(3.4, 5.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    planner.updateOdometry(makeOdom(3.5, 5.0, 1.02));
    ASSERT_TRUE(planner.terminalApproachActive());

    planner.updateOdometry(makeOdom(4.0, 0.0, 1.04));
    ASSERT_TRUE(planner.syncReferenceFromCurrentOdom());
    EXPECT_FALSE(planner.terminalApproachActive());
    EXPECT_FALSE(planner.hasTrajectory());

    planner.setGoal(Eigen::Vector3d(20.0, 0.0, 1.0));
    EXPECT_FALSE(planner.arrived());
    EXPECT_FALSE(planner.terminalApproachActive());
}

TEST(YopoTerminalApproach, LegacyModeKeepsExistingRadiusEntryHold) {
    auto params = terminalParams();
    params.terminal_approach_enabled = false;
    yopo_planner::YopoPlanner planner(params);
    planner.updateOdometry(makeOdom(8.8, 4.0, 1.0));
    planner.setGoal(Eigen::Vector3d(10.0, 0.0, 1.0));
    planner.updateOdometry(makeOdom(8.8, 4.0, 1.02));
    ASSERT_TRUE(planner.arrived());
    quadrotor_msgs::PositionCommand cmd;
    ASSERT_TRUE(planner.fillControlCommand(&cmd));
    EXPECT_NEAR(cmd.position.x, 8.8, 1e-9);
}

}  // namespace

int main(int argc, char** argv) {
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
