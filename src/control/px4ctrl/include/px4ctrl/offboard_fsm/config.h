#pragma once

#include <cstdint>
#include <ros/ros.h>
#include <string>
#include <vector>

namespace px4ctrl {

struct RcSwitchConfig {
    int channel{0};
    bool reversed{false};
    int trigger_pwm{1700};
    int release_pwm{1300};
};

struct MessageIntervalConfig {
    std::string name;
    uint32_t id{0};
    double rate{0.0};
    bool enabled{true};
    bool required{false};
};

struct Config {
    int schema_version{1};

    struct Node {
        double control_rate{50.0};
        double status_rate{10.0};
        bool start_enabled{true};
        bool prestream_when_ready{true};
        double prestream_min_time{1.2};
        bool request_offboard_automatically{false};
    } node;

    struct Topics {
        std::string mavros_state{"/mavros/state"};
        std::string extended_state{"/mavros/extended_state"};
        std::string px4_odometry{"/mavros/local_position/odom"};
        std::string livo_odometry{"/LIVO2/imu_propagate"};
        std::string rc_input{"/mavros/rc/in"};
        std::string planner_command{"/position_cmd"};
        std::string planner_emergency_hover{"/planning/Emergency_hover"};
        std::string fsm_command{"/offboard_fsm/command"};
        std::string planner_lifecycle{"/px4ctrl/takeoff_land"};
        std::string traj_start_trigger{"/traj_start_trigger"};
        std::string setpoint{"/mavros/setpoint_raw/local"};
        std::string status{"/offboard_fsm/status"};
        std::string reference{"/offboard_fsm/reference"};
        std::string diagnostics{"/diagnostics"};
    } topics;

    struct Services {
        std::string arming{"/mavros/cmd/arming"};
        std::string set_mode{"/mavros/set_mode"};
        std::string message_interval{"/mavros/set_message_interval"};
    } services;

    struct Timeouts {
        double mavros_state{0.5};
        double extended_state{1.0};
        double px4_odometry{0.30};
        double livo_odometry{0.30};
        double planner_command{0.25};
        double rc_input{0.5};
        double arm_request{5.0};
        double disarm_request{5.0};
        double takeoff{10.0};
        double landing{30.0};
    } timeouts;

    struct Rc {
        bool enabled{false};
        int index_base{1};
        double debounce_time{0.15};
        bool require_release_before_trigger{true};
        RcSwitchConfig takeoff_land;
    } rc;

    struct Takeoff {
        double relative_height{0.7};
        double max_velocity{0.4};
        double max_acceleration{0.5};
        double max_jerk{1.0};
        double position_tolerance{0.08};
        double velocity_tolerance{0.12};
        double stable_time{1.0};
    } takeoff;

    struct Planner {
        int activation_messages{3};
        double max_position_jump{0.5};
        double max_yaw_jump_deg{30.0};
        double max_velocity{3.0};
        double max_acceleration{4.0};
        double brake_acceleration{1.0};
        double brake_velocity_tolerance{0.15};
        double brake_stable_time{0.5};
    } planner;

    struct Landing {
        std::string strategy{"offboard_descent"};
        double max_velocity{0.30};
        double max_acceleration{0.40};
        double flare_height{0.30};
        double flare_velocity{0.15};
        double touchdown_confirm_time{1.0};
        bool auto_disarm{true};
        double disarm_delay{1.0};
        double max_descent_distance{10.0};
    } landing;

    struct Frames {
        bool align_planner_on_offboard_entry{true};
    } frames;

    struct Limits {
        double max_horizontal_distance{10.0};
        double max_height_above_origin{3.0};
        double min_height_below_origin{-0.30};
    } limits;

    struct Failsafe {
        bool stop_setpoints_on_critical_failure{true};
    } failsafe;

    struct Requests {
        double retry_interval{1.0};
    } requests;

    struct MessageIntervals {
        bool enabled{true};
        bool reapply_on_reconnect{true};
        int retry_count{3};
        double retry_delay{0.5};
        std::vector<MessageIntervalConfig> messages;
    } message_intervals;

    static Config load(const ros::NodeHandle& private_nh);
    void validate() const;
};

}  // namespace px4ctrl
