#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>
#include <string>

class Parameter_t {
  public:
    struct Gain {
        double Kp0, Kp1, Kp2;
        double Ki0, Ki1, Ki2;
        double Kd0, Kd1, Kd2;
    };

    struct RotorDrag {
        double x, y, z;
        double k_thrust_horz;
    };

    struct MsgTimeout {
        double odom;
        double rc;
        double cmd;
        double imu;
        double bat;
    };

    struct ThrustMapping {
        bool print_val;
        double K1;
        double K2;
        double K3;
        bool accurate_thrust_model;
        bool use_battery_feedback{true};
        bool online_estimation{true};
        double hover_percentage;
        double imu_acc_lpf_freq_cutoff;
    };

    struct RCReverse {
        bool roll;
        bool pitch;
        bool yaw;
        bool throttle;
    };

    struct AutoTakeoffLand {
        bool enable;
        bool enable_auto_arm;
        bool no_RC;
        double height;
        double speed;
    };

    struct TuningDebug {
        bool enable{false};
        std::string topic{"/px4ctrl/tune_debug"};
    };

    struct VelocityFilter {
        bool enable{true};
        double cutoff_frequency{4.0};
        double reset_after_gap{0.2};
    };

    struct ControlLimits {
        bool enable{true};
        double max_horizontal_acceleration{2.0};
        double max_vertical_acceleration_up{2.0};
        double max_vertical_acceleration_down{2.0};
        double min_thrust{0.15};
        double max_thrust{0.90};
    };

    Gain gain;
    RotorDrag rt_drag;
    MsgTimeout msg_timeout;
    RCReverse rc_reverse;
    ThrustMapping thr_map;
    AutoTakeoffLand takeoff_land;
    TuningDebug tuning_debug;
    VelocityFilter velocity_filter;
    ControlLimits control_limits;

    int pose_solver;
    double mass;
    double gra;
    double max_angle;
    double ctrl_freq_max;
    double max_manual_vel;
    double low_voltage;

    bool use_bodyrate_ctrl;
    // bool print_dbg;

    // mavros params
    std::string mavros_ns;
    int mavros_battery_id;
    float mavros_bat_msg_freq;
    int mavros_attitude_id;
    float mavros_attitude_msg_freq;
    int mavros_attitude_quaternion_id;
    float mavros_attitude_quaternion_msg_freq;
    int mavros_highres_imu_id;
    float mavros_highres_imu_msg_freq;

    Parameter_t();
    void config_from_ros_handle(const ros::NodeHandle &nh);
    void config_full_thrust(double hov);

  private:
    template <typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val) {
        if (nh.getParam(name, val)) {
            // pass
        } else {
            ROS_ERROR_STREAM("Read param: " << name << " failed.");
            ROS_BREAK();
        }
    };
};

#endif
