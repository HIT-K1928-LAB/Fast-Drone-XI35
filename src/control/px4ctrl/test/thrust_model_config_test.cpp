#include "PX4CtrlFSM.h"
#include "controller.h"

#include <cmath>
#include <gtest/gtest.h>
#include <ros/ros.h>

namespace {

Parameter_t makeParameter(bool use_battery_feedback, bool online_estimation) {
    Parameter_t param;
    param.gra                                  = 9.81;
    param.ctrl_freq_max                        = 200.0;
    param.max_angle                            = 0.5;
    param.msg_timeout.bat                      = 0.5;
    param.thr_map.hover_percentage             = 0.45;
    param.thr_map.imu_acc_lpf_freq_cutoff      = 40.0;
    param.thr_map.use_battery_feedback         = use_battery_feedback;
    param.thr_map.online_estimation             = online_estimation;
    param.gain.Kp0 = param.gain.Kp1 = param.gain.Kp2 = 0.0;
    param.gain.Ki0 = param.gain.Ki1 = param.gain.Ki2 = 0.0;
    param.gain.Kd0 = param.gain.Kd1 = param.gain.Kd2 = 0.0;
    return param;
}

TEST(ThrustModelConfig, DisabledBatteryFeedbackDoesNotRequireBatteryMessages) {
    Parameter_t param = makeParameter(false, true);
    LinearControl controller(param);
    PX4CtrlFSM fsm(param, controller);

    EXPECT_TRUE(fsm.bat_is_received(ros::Time::now()));
}

TEST(ThrustModelConfig, EnabledBatteryFeedbackRequiresFreshBatteryMessages) {
    Parameter_t param = makeParameter(true, true);
    LinearControl controller(param);
    PX4CtrlFSM fsm(param, controller);

    EXPECT_FALSE(fsm.bat_is_received(ros::Time::now()));
}

TEST(ThrustModelConfig, DisabledOnlineEstimationKeepsTheThrustMappingFixed) {
    Parameter_t param = makeParameter(false, false);
    LinearControl controller(param);
    Desired_State_t desired;
    Odom_Data_t odom;
    Imu_Data_t imu;
    Controller_Output_t output;

    desired.p.setZero();
    desired.v.setZero();
    desired.a.setZero();
    desired.yaw = 0.0;
    odom.p.setZero();
    odom.v.setZero();
    odom.q = Eigen::Quaterniond::Identity();
    imu.q  = Eigen::Quaterniond::Identity();

    for (int sample = 0; sample < 60; ++sample) {
        controller.calculateControl(desired, odom, imu, output);
        ros::WallDuration(0.001).sleep();
    }

    EXPECT_FALSE(controller.estimateThrustModel(Eigen::Vector3d(0.0, 0.0, 15.0), param));
}

TEST(TuningDebug, IsDisabledByDefault) {
    Parameter_t param;
    EXPECT_FALSE(param.tuning_debug.enable);
    EXPECT_EQ("/px4ctrl/tune_debug", param.tuning_debug.topic);
}

TEST(TuningDebug, SnapshotContainsSynchronizedControllerTerms) {
    Parameter_t param = makeParameter(false, false);
    param.tuning_debug.enable = true;
    param.max_angle = 0.5;
    param.gain.Kp0 = 2.0;

    LinearControl controller(param);
    Desired_State_t desired;
    Odom_Data_t odom;
    Imu_Data_t imu;
    Controller_Output_t output;

    desired.p = Eigen::Vector3d(0.1, 0.0, 1.0);
    desired.v.setZero();
    desired.a.setZero();
    desired.yaw = 0.0;
    odom.p = Eigen::Vector3d(0.0, 0.0, 1.0);
    odom.v.setZero();
    odom.q = Eigen::Quaterniond::Identity();
    imu.q = Eigen::Quaterniond::Identity();
    imu.w = Eigen::Vector3d(0.01, 0.02, 0.03);
    imu.a = Eigen::Vector3d(0.1, 0.2, 9.7);

    controller.calculateControl(desired, odom, imu, output);
    const quadrotor_msgs::Px4ctrlTuneDebug &debug = controller.getTuneDebug();

    EXPECT_DOUBLE_EQ(0.1, debug.position_error.x);
    EXPECT_DOUBLE_EQ(0.2, debug.acceleration_proportional.x);
    EXPECT_DOUBLE_EQ(9.81, debug.gravity_compensation.z);
    EXPECT_DOUBLE_EQ(0.01, debug.body_rate_meas.x);
    EXPECT_DOUBLE_EQ(9.7, debug.specific_force_body_meas.z);
    EXPECT_DOUBLE_EQ(2.0, debug.kp.x);
    EXPECT_NEAR(0.45, debug.thrust_command, 1e-3);
    EXPECT_FALSE(debug.tilt_limit_exceeded);
    EXPECT_FALSE(debug.thrust_limit_exceeded);
}

TEST(ControlLimits, ConstrainsAccelerationTiltAndThrust) {
    Parameter_t param = makeParameter(false, false);
    param.tuning_debug.enable = true;
    param.max_angle = 0.15;
    param.control_limits.enable = true;
    param.control_limits.max_horizontal_acceleration = 5.0;
    param.control_limits.max_vertical_acceleration_up = 1.0;
    param.control_limits.max_vertical_acceleration_down = 1.0;
    param.control_limits.min_thrust = 0.2;
    param.control_limits.max_thrust = 0.45;
    param.gain.Kp0 = 20.0;
    param.gain.Kp2 = 20.0;

    LinearControl controller(param);
    Desired_State_t desired;
    Odom_Data_t odom;
    Imu_Data_t imu;
    Controller_Output_t output;

    desired.p = Eigen::Vector3d(10.0, 0.0, 10.0);
    desired.v.setZero();
    desired.a.setZero();
    desired.yaw = 0.0;
    odom.p.setZero();
    odom.v.setZero();
    odom.q = Eigen::Quaterniond::Identity();
    odom.msg.header.stamp = ros::Time(1.0);
    imu.q = Eigen::Quaterniond::Identity();

    controller.calculateControl(desired, odom, imu, output);
    const quadrotor_msgs::Px4ctrlTuneDebug &debug = controller.getTuneDebug();

    EXPECT_NEAR(5.0, debug.acceleration_command.x, 1e-9);
    EXPECT_NEAR(10.81, debug.acceleration_command.z, 1e-9);
    EXPECT_LE(debug.tilt_command, param.max_angle + 1e-9);
    EXPECT_DOUBLE_EQ(0.45, output.thrust);
    EXPECT_TRUE(debug.acceleration_limit_exceeded);
    EXPECT_TRUE(debug.tilt_limit_exceeded);
    EXPECT_TRUE(debug.thrust_limit_exceeded);
}

TEST(VelocityFilter, UpdatesOnlyForANewOdometryTimestamp) {
    Parameter_t param = makeParameter(false, false);
    param.tuning_debug.enable = true;
    param.velocity_filter.enable = true;
    param.velocity_filter.cutoff_frequency = 1.0;
    param.velocity_filter.reset_after_gap = 1.0;

    LinearControl controller(param);
    Desired_State_t desired;
    Odom_Data_t odom;
    Imu_Data_t imu;
    Controller_Output_t output;

    desired.p.setZero();
    desired.v.setZero();
    desired.a.setZero();
    desired.yaw = 0.0;
    odom.p.setZero();
    odom.v.setZero();
    odom.q = Eigen::Quaterniond::Identity();
    odom.msg.header.stamp = ros::Time(1.0);
    imu.q = Eigen::Quaterniond::Identity();

    controller.resetControlState(odom);
    controller.calculateControl(desired, odom, imu, output);

    odom.v.x() = 1.0;
    odom.msg.header.stamp = ros::Time(1.1);
    controller.calculateControl(desired, odom, imu, output);
    const double filtered_new_sample = controller.getTuneDebug().velocity_filtered.x;
    const double expected_alpha = 1.0 - std::exp(-2.0 * M_PI * 0.1);
    EXPECT_NEAR(expected_alpha, filtered_new_sample, 1e-9);

    odom.v.x() = 10.0;
    controller.calculateControl(desired, odom, imu, output);
    EXPECT_DOUBLE_EQ(filtered_new_sample, controller.getTuneDebug().velocity_filtered.x);

    controller.resetControlState(odom);
    controller.calculateControl(desired, odom, imu, output);
    EXPECT_DOUBLE_EQ(10.0, controller.getTuneDebug().velocity_filtered.x);
    EXPECT_DOUBLE_EQ(0.0, controller.getTuneDebug().velocity_error_integral.x);
}

}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "px4ctrl_thrust_model_test");
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
