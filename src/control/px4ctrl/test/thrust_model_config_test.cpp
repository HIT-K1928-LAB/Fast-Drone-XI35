#include "PX4CtrlFSM.h"
#include "controller.h"

#include <gtest/gtest.h>
#include <ros/ros.h>

namespace {

Parameter_t makeParameter(bool use_battery_feedback, bool online_estimation) {
    Parameter_t param;
    param.gra                                  = 9.81;
    param.ctrl_freq_max                        = 200.0;
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

}  // namespace

int main(int argc, char **argv) {
    ros::init(argc, argv, "px4ctrl_thrust_model_test");
    ros::Time::init();
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
