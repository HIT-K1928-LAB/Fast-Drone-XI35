#include <gtest/gtest.h>

#include <cmath>
#include <nav_msgs/Odometry.h>
#include <ros/time.h>

#include "yopo_planner/yopo_planner.h"

namespace {

nav_msgs::Odometry makeOdom(double yaw, double vx = 0.0, double vy = 0.0) {
    nav_msgs::Odometry odom;
    odom.pose.pose.position.z    = 1.0;
    odom.pose.pose.orientation.z = std::sin(0.5 * yaw);
    odom.pose.pose.orientation.w = std::cos(0.5 * yaw);
    odom.twist.twist.linear.x    = vx;
    odom.twist.twist.linear.y    = vy;
    return odom;
}

yopo_planner::YopoParams alignmentParams() {
    yopo_planner::YopoParams params;
    params.ctrl_dt                          = 0.02;
    params.goal_yaw_alignment_enabled      = true;
    params.goal_yaw_align_enter_deg        = 35.0;
    params.goal_yaw_align_rate             = 0.8;
    params.goal_yaw_brake_speed_tolerance  = 0.15;
    params.goal_yaw_brake_stable_time      = 0.04;
    params.goal_yaw_align_min_distance     = 0.5;
    return params;
}

TEST(YopoGoalYawAlignment, FrontGoalStartsPlanningWithoutAlignment) {
    yopo_planner::YopoPlanner planner(alignmentParams());
    planner.updateOdometry(makeOdom(0.0));
    planner.setGoal(Eigen::Vector3d(5.0, 0.0, 1.0));

    EXPECT_FALSE(planner.goalAlignmentActive());
}

TEST(YopoGoalYawAlignment, MovingVehicleBrakesBeforeTurning) {
    yopo_planner::YopoPlanner planner(alignmentParams());
    planner.updateOdometry(makeOdom(0.0, 0.5, 0.0));
    planner.setGoal(Eigen::Vector3d(-5.0, 0.0, 1.0));
    ASSERT_TRUE(planner.goalAlignmentActive());

    for (int i = 0; i < 20; ++i) {
        quadrotor_msgs::PositionCommand cmd;
        ASSERT_TRUE(planner.fillControlCommand(&cmd));
        EXPECT_DOUBLE_EQ(cmd.velocity.x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.velocity.y, 0.0);
        EXPECT_NEAR(cmd.yaw, 0.0, 1e-12);
    }
    EXPECT_TRUE(planner.goalAlignmentActive());
}

TEST(YopoGoalYawAlignment, RearGoalHoldsPositionUntilItEntersForwardSector) {
    yopo_planner::YopoPlanner planner(alignmentParams());
    planner.updateOdometry(makeOdom(0.0));
    planner.setGoal(Eigen::Vector3d(-5.0, 0.0, 1.0));
    ASSERT_TRUE(planner.goalAlignmentActive());

    double simulated_yaw = 0.0;
    quadrotor_msgs::PositionCommand cmd;
    int iterations = 0;
    while (planner.goalAlignmentActive() && iterations++ < 400) {
        ASSERT_TRUE(planner.fillControlCommand(&cmd));
        EXPECT_NEAR(cmd.position.x, 0.0, 1e-12);
        EXPECT_NEAR(cmd.position.y, 0.0, 1e-12);
        EXPECT_NEAR(cmd.position.z, 1.0, 1e-12);
        EXPECT_DOUBLE_EQ(cmd.velocity.x, 0.0);
        EXPECT_DOUBLE_EQ(cmd.velocity.y, 0.0);
        EXPECT_DOUBLE_EQ(cmd.velocity.z, 0.0);

        simulated_yaw = cmd.yaw;
        planner.updateOdometry(makeOdom(simulated_yaw));
    }

    EXPECT_LT(iterations, 400);
    EXPECT_FALSE(planner.goalAlignmentActive());
    EXPECT_NEAR(std::abs(simulated_yaw), M_PI, 35.0 * M_PI / 180.0 + 1e-6);
}

TEST(YopoGoalYawAlignment, PlanningReferenceUsesActualPositionAfterTurn) {
    auto params = alignmentParams();
    params.plan_from_reference = true;
    yopo_planner::YopoPlanner planner(params);
    planner.updateOdometry(makeOdom(0.0));
    planner.setGoal(Eigen::Vector3d(-5.0, 0.0, 1.0));
    ASSERT_TRUE(planner.goalAlignmentActive());

    quadrotor_msgs::PositionCommand cmd;
    int iterations = 0;
    while (planner.goalAlignmentActive() && iterations++ < 400) {
        ASSERT_TRUE(planner.fillControlCommand(&cmd));
        auto odom = makeOdom(cmd.yaw);
        if (iterations > 100) odom.pose.pose.position.x = 0.3;
        planner.updateOdometry(odom);
    }
    ASSERT_FALSE(planner.goalAlignmentActive());

    std::array<float, 1 * 9 * 3 * 5> endstate{};
    std::array<float, 1 * 3 * 5> scores{};
    planner.updateTrajectory(endstate, scores, false);
    ASSERT_TRUE(planner.fillControlCommand(&cmd));
    EXPECT_NEAR(cmd.position.x, 0.3, 0.01);
}

}  // namespace

int main(int argc, char** argv) {
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
