#pragma once

#include <cstdint>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <limits>
#include <quadrotor_msgs/PositionCommand.h>
#include <string>

namespace px4ctrl {

enum class FsmState : uint8_t {
    WAIT_FCU  = 0,
    PRESTREAM = 1,
    GROUND    = 2,
    ARMING    = 3,
    TAKEOFF   = 4,
    HOVER     = 5,
    EXTERNAL  = 6,
    BRAKE     = 7,
    LANDING   = 8,
    DISARMING = 9,
    FAILSAFE  = 10,
};

struct Events {
    bool takeoff{false};
    bool land{false};
    bool hover{false};
    bool abort{false};
    bool reset{false};
    double takeoff_height{0.0};
};

struct InputSnapshot {
    double now{0.0};
    bool enabled{true};
    bool state_fresh{false};
    bool extended_state_fresh{false};
    bool odometry_fresh{false};
    bool livo_odometry_fresh{false};
    bool planner_fresh{false};
    bool connected{false};
    bool armed{false};
    std::string mode;
    uint8_t landed_state{0};
    double odometry_age{std::numeric_limits<double>::infinity()};
    double planner_age{std::numeric_limits<double>::infinity()};

    geometry_msgs::Point position;
    geometry_msgs::Vector3 velocity_world;
    double yaw{0.0};

    geometry_msgs::Point livo_position;
    double livo_yaw{0.0};

    quadrotor_msgs::PositionCommand planner;
    uint64_t planner_sequence{0};
    double planner_received_at{0.0};
    Events events;
};

struct Reference {
    geometry_msgs::Point position;
    geometry_msgs::Vector3 velocity;
    geometry_msgs::Vector3 acceleration;
    double yaw{0.0};
    double yaw_rate{0.0};
    bool use_velocity_acceleration{false};
};

struct FsmOutput {
    bool publish_setpoint{false};
    bool publish_traj_start_trigger{false};
    bool request_arm{false};
    bool request_disarm{false};
    bool request_offboard{false};
    Reference reference;
};

const char* stateName(FsmState state);

}  // namespace px4ctrl
