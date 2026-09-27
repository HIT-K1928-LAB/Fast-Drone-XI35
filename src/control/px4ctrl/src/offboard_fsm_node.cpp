#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/KeyValue.h>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/ExtendedState.h>
#include <mavros_msgs/MessageInterval.h>
#include <mavros_msgs/PositionTarget.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <px4ctrl/FsmCommand.h>
#include <px4ctrl/FsmStatus.h>
#include <px4ctrl/offboard_fsm/config.h>
#include <px4ctrl/offboard_fsm/freshness.h>
#include <px4ctrl/offboard_fsm/fsm_core.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_srvs/SetBool.h>
#include <std_srvs/Trigger.h>
#include <string>
#include <thread>
#include <vector>

namespace px4ctrl {
namespace {

double steadyNow() { return ros::SteadyTime::now().toSec(); }

double yawFromQuaternion(const geometry_msgs::Quaternion& q) {
    const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
    const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
    return std::atan2(siny_cosp, cosy_cosp);
}

geometry_msgs::Quaternion quaternionFromYaw(const double yaw) {
    geometry_msgs::Quaternion output;
    output.z = std::sin(0.5 * yaw);
    output.w = std::cos(0.5 * yaw);
    return output;
}

geometry_msgs::Vector3 rotateBodyToWorld(
    const geometry_msgs::Quaternion& q, const geometry_msgs::Vector3& value) {
    const double tx = 2.0 * (q.y * value.z - q.z * value.y);
    const double ty = 2.0 * (q.z * value.x - q.x * value.z);
    const double tz = 2.0 * (q.x * value.y - q.y * value.x);
    geometry_msgs::Vector3 output;
    output.x = value.x + q.w * tx + (q.y * tz - q.z * ty);
    output.y = value.y + q.w * ty + (q.z * tx - q.x * tz);
    output.z = value.z + q.w * tz + (q.x * ty - q.y * tx);
    return output;
}

class DebouncedSwitch {
  public:
    DebouncedSwitch() = default;

    bool update(
        const int pwm, const RcSwitchConfig& config, const double debounce_time,
        const bool require_release, const double now) {
        bool raw_active = candidate_active_;
        if (!initialized_) {
            raw_active   = config.reversed ? pwm <= config.trigger_pwm : pwm >= config.trigger_pwm;
            initialized_ = true;
            candidate_active_ = raw_active;
            stable_active_    = raw_active;
            candidate_since_  = now;
            trigger_armed_    = !raw_active || !require_release;
            return !require_release && raw_active;
        }

        if (!config.reversed) {
            if (pwm >= config.trigger_pwm) {
                raw_active = true;
            } else if (pwm <= config.release_pwm) {
                raw_active = false;
            }
        } else {
            if (pwm <= config.trigger_pwm) {
                raw_active = true;
            } else if (pwm >= config.release_pwm) {
                raw_active = false;
            }
        }

        if (raw_active != candidate_active_) {
            candidate_active_ = raw_active;
            candidate_since_  = now;
            return false;
        }
        if (candidate_active_ == stable_active_ || now - candidate_since_ < debounce_time) {
            return false;
        }

        stable_active_ = candidate_active_;
        if (!stable_active_) {
            trigger_armed_ = true;
            return false;
        }
        if (trigger_armed_) {
            trigger_armed_ = false;
            return true;
        }
        return false;
    }

  private:
    bool initialized_{false};
    bool candidate_active_{false};
    bool stable_active_{false};
    bool trigger_armed_{false};
    double candidate_since_{0.0};
};

}  // namespace

class OffboardFsmNode {
  public:
    OffboardFsmNode() : nh_(), pnh_("~"), config_(Config::load(pnh_)), core_(config_) {
        enabled_ = config_.node.start_enabled;

        setpoint_publisher_ =
            nh_.advertise<mavros_msgs::PositionTarget>(config_.topics.setpoint, 10);
        status_publisher_    = nh_.advertise<px4ctrl::FsmStatus>(config_.topics.status, 10, true);
        reference_publisher_ = nh_.advertise<nav_msgs::Odometry>(config_.topics.reference, 10);
        planner_lifecycle_publisher_ =
            nh_.advertise<quadrotor_msgs::TakeoffLand>(config_.topics.planner_lifecycle, 2, true);
        traj_start_trigger_publisher_ =
            nh_.advertise<geometry_msgs::PoseStamped>(config_.topics.traj_start_trigger, 1, false);
        diagnostics_publisher_ =
            nh_.advertise<diagnostic_msgs::DiagnosticArray>(config_.topics.diagnostics, 10);

        state_subscriber_ =
            nh_.subscribe(config_.topics.mavros_state, 10, &OffboardFsmNode::stateCallback, this);
        extended_state_subscriber_ = nh_.subscribe(
            config_.topics.extended_state, 10, &OffboardFsmNode::extendedStateCallback, this);
        px4_odom_subscriber_ =
            nh_.subscribe(config_.topics.px4_odometry, 10, &OffboardFsmNode::px4OdomCallback, this);
        livo_odom_subscriber_ = nh_.subscribe(
            config_.topics.livo_odometry, 10, &OffboardFsmNode::livoOdomCallback, this);
        planner_subscriber_ = nh_.subscribe(
            config_.topics.planner_command, 10, &OffboardFsmNode::plannerCallback, this);
        planner_emergency_subscriber_ = nh_.subscribe(
            config_.topics.planner_emergency_hover, 10, &OffboardFsmNode::plannerEmergencyCallback,
            this);
        command_subscriber_ =
            nh_.subscribe(config_.topics.fsm_command, 10, &OffboardFsmNode::commandCallback, this);
        if (config_.rc.enabled) {
            rc_subscriber_ =
                nh_.subscribe(config_.topics.rc_input, 10, &OffboardFsmNode::rcCallback, this);
        }

        arming_client_ = nh_.serviceClient<mavros_msgs::CommandBool>(config_.services.arming);
        mode_client_   = nh_.serviceClient<mavros_msgs::SetMode>(config_.services.set_mode);
        message_interval_client_ =
            nh_.serviceClient<mavros_msgs::MessageInterval>(config_.services.message_interval);

        enable_service_ = pnh_.advertiseService("enable", &OffboardFsmNode::enableCallback, this);
        reset_service_  = pnh_.advertiseService("reset", &OffboardFsmNode::resetCallback, this);

        control_timer_ = nh_.createWallTimer(
            ros::WallDuration(1.0 / config_.node.control_rate), &OffboardFsmNode::controlCallback,
            this);
        status_timer_ = nh_.createWallTimer(
            ros::WallDuration(1.0 / config_.node.status_rate), &OffboardFsmNode::statusCallback,
            this);

        running_.store(true);
        interval_reset_requested_.store(true);
        service_request_thread_ = std::thread(&OffboardFsmNode::serviceRequestWorker, this);
        message_interval_thread_ = std::thread(&OffboardFsmNode::messageIntervalWorker, this);

        ROS_WARN(
            "PX4 native OFFBOARD FSM started %s. It never changes mode unless "
            "node/request_offboard_automatically is enabled.",
            enabled_ ? "ENABLED" : "DISABLED");
    }

    ~OffboardFsmNode() {
        running_.store(false);
        if (service_request_thread_.joinable()) {
            service_request_thread_.join();
        }
        if (message_interval_thread_.joinable()) {
            message_interval_thread_.join();
        }
    }

  private:
    struct ServiceRequests {
        bool arm{false};
        bool disarm{false};
        bool offboard{false};
    };

    void stateCallback(const mavros_msgs::State::ConstPtr& message) {
        const bool was_connected = connected_.exchange(message->connected);
        if (message->connected && !was_connected) {
            const bool first_connection = !ever_connected_.exchange(true);
            if (first_connection || config_.message_intervals.reapply_on_reconnect) {
                interval_reset_requested_.store(true);
            }
        }
        std::lock_guard<std::mutex> lock(data_mutex_);
        state_             = *message;
        state_received_at_ = steadyNow();
        have_state_        = true;
    }

    void extendedStateCallback(const mavros_msgs::ExtendedState::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        extended_state_             = *message;
        extended_state_received_at_ = steadyNow();
        have_extended_state_        = true;
    }

    void px4OdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        const double received_at = steadyNow();
        if (have_px4_odom_) {
            px4_odom_last_gap_ = received_at - px4_odom_received_at_;
            px4_odom_max_gap_  = std::max(px4_odom_max_gap_, px4_odom_last_gap_);
        }
        px4_odom_             = *message;
        px4_odom_received_at_ = received_at;
        have_px4_odom_        = true;
    }

    void livoOdomCallback(const nav_msgs::Odometry::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        livo_odom_             = *message;
        livo_odom_received_at_ = steadyNow();
        have_livo_odom_        = true;
    }

    void plannerCallback(const quadrotor_msgs::PositionCommand::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        planner_command_     = *message;
        planner_received_at_ = steadyNow();
        ++planner_sequence_;
        have_planner_command_ = true;
    }

    void commandCallback(const px4ctrl::FsmCommand::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        switch (message->command) {
            case px4ctrl::FsmCommand::TAKEOFF:
                pending_events_.takeoff        = true;
                pending_events_.takeoff_height = message->takeoff_height;
                break;
            case px4ctrl::FsmCommand::LAND:
                pending_events_.land = true;
                break;
            case px4ctrl::FsmCommand::HOVER:
                pending_events_.hover = true;
                break;
            case px4ctrl::FsmCommand::ABORT:
                pending_events_.abort = true;
                break;
            case px4ctrl::FsmCommand::RESET:
                pending_events_.reset = true;
                break;
            default:
                ROS_WARN_THROTTLE(1.0, "Ignoring unknown FSM command %u", message->command);
                break;
        }
    }

    void plannerEmergencyCallback(const std_msgs::Bool::ConstPtr& message) {
        if (!message->data) {
            return;
        }
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_events_.hover = true;
    }

    void rcCallback(const mavros_msgs::RCIn::ConstPtr& message) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        rc_channels_    = message->channels;
        rc_received_at_ = steadyNow();
    }

    bool enableCallback(
        std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        enabled_ = request.data;
        if (!enabled_) {
            pending_events_ = Events();
        }
        response.success = true;
        response.message = enabled_ ? "OFFBOARD FSM enabled" : "OFFBOARD FSM disabled";
        return true;
    }

    bool resetCallback(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& response) {
        std::lock_guard<std::mutex> lock(data_mutex_);
        pending_events_.reset = true;
        response.success      = true;
        response.message      = "failsafe reset event queued";
        return true;
    }

    int channelIndex(const RcSwitchConfig& config) const {
        return config.channel - config_.rc.index_base;
    }

    void appendRcEvents(const double now, Events& events) {
        if (!config_.rc.enabled ||
            !receiptFresh(rc_received_at_, config_.timeouts.rc_input, now)) {
            return;
        }
        const int channel_index = channelIndex(config_.rc.takeoff_land);
        if (channel_index < 0 || static_cast<std::size_t>(channel_index) >= rc_channels_.size()) {
            ROS_ERROR_THROTTLE(1.0, "Configured RC channel is not present in /mavros/rc/in");
            return;
        }
        if (!takeoff_land_switch_.update(
                rc_channels_[channel_index], config_.rc.takeoff_land, config_.rc.debounce_time,
                config_.rc.require_release_before_trigger, now)) {
            return;
        }

        FsmState state;
        {
            std::lock_guard<std::mutex> lock(core_mutex_);
            state = core_.state();
        }
        const bool extended_fresh =
            have_extended_state_ &&
            receiptFresh(extended_state_received_at_, config_.timeouts.extended_state, now);
        const Events rc_event = rcTakeoffLandEvent(
            state, extended_fresh, have_extended_state_ ? extended_state_.landed_state : 0);
        events.takeoff = events.takeoff || rc_event.takeoff;
        events.land    = events.land || rc_event.land;
        if (!rc_event.takeoff && !rc_event.land) {
            ROS_WARN(
                "RC takeoff/land edge ignored in state %s%s", stateName(state),
                state == FsmState::GROUND && !extended_fresh ? ": extended state is stale" : "");
        }
    }

    InputSnapshot snapshot() {
        std::lock_guard<std::mutex> lock(data_mutex_);
        InputSnapshot input;
        // Take the monotonic timestamp while holding the same lock that guards
        // callback receipt times. This keeps one coherent temporal snapshot.
        input.now     = steadyNow();
        input.enabled = enabled_;
        input.state_fresh =
            have_state_ &&
            receiptFresh(state_received_at_, config_.timeouts.mavros_state, input.now);
        input.extended_state_fresh =
            have_extended_state_ &&
            receiptFresh(
                extended_state_received_at_, config_.timeouts.extended_state, input.now);
        input.odometry_fresh =
            have_px4_odom_ &&
            receiptFresh(px4_odom_received_at_, config_.timeouts.px4_odometry, input.now);
        input.livo_odometry_fresh =
            have_livo_odom_ &&
            receiptFresh(livo_odom_received_at_, config_.timeouts.livo_odometry, input.now);
        input.planner_fresh =
            have_planner_command_ &&
            receiptFresh(planner_received_at_, config_.timeouts.planner_command, input.now);
        if (have_px4_odom_) {
            input.odometry_age = std::max(0.0, input.now - px4_odom_received_at_);
        }
        if (have_planner_command_) {
            input.planner_age = std::max(0.0, input.now - planner_received_at_);
        }

        if (have_state_) {
            input.connected = state_.connected;
            input.armed     = state_.armed;
            input.mode      = state_.mode;
        }
        if (have_extended_state_) {
            input.landed_state = extended_state_.landed_state;
        }
        if (have_px4_odom_) {
            input.position = px4_odom_.pose.pose.position;
            input.yaw      = yawFromQuaternion(px4_odom_.pose.pose.orientation);
            input.velocity_world =
                rotateBodyToWorld(px4_odom_.pose.pose.orientation, px4_odom_.twist.twist.linear);
        }
        if (have_livo_odom_) {
            input.livo_position = livo_odom_.pose.pose.position;
            input.livo_yaw      = yawFromQuaternion(livo_odom_.pose.pose.orientation);
        }
        if (have_planner_command_) {
            input.planner             = planner_command_;
            input.planner_sequence    = planner_sequence_;
            input.planner_received_at = planner_received_at_;
        }

        appendRcEvents(input.now, pending_events_);

        // Resolve simultaneous edge events deterministically. A landing request
        // always wins over every command that could keep or put the vehicle in
        // flight; abort/hover in turn win over takeoff.
        if (pending_events_.land) {
            pending_events_.takeoff = false;
            pending_events_.hover   = false;
            pending_events_.abort   = false;
        } else if (pending_events_.abort || pending_events_.hover) {
            pending_events_.takeoff = false;
        }
        input.events    = pending_events_;
        pending_events_ = Events();
        return input;
    }

    mavros_msgs::PositionTarget positionTarget(const Reference& reference) const {
        mavros_msgs::PositionTarget target;
        target.header.stamp          = ros::Time::now();
        target.coordinate_frame      = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
        target.position              = reference.position;
        target.velocity              = reference.velocity;
        target.acceleration_or_force = reference.acceleration;
        target.yaw                   = reference.yaw;
        target.yaw_rate              = reference.yaw_rate;
        if (reference.use_velocity_acceleration) {
            target.type_mask = 0;
        } else {
            target.type_mask =
                mavros_msgs::PositionTarget::IGNORE_VX | mavros_msgs::PositionTarget::IGNORE_VY |
                mavros_msgs::PositionTarget::IGNORE_VZ | mavros_msgs::PositionTarget::IGNORE_AFX |
                mavros_msgs::PositionTarget::IGNORE_AFY | mavros_msgs::PositionTarget::IGNORE_AFZ |
                mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
        }
        return target;
    }

    void publishReference(const Reference& reference) {
        nav_msgs::Odometry message;
        message.header.stamp          = ros::Time::now();
        message.header.frame_id       = "map";
        message.child_frame_id        = "offboard_reference";
        message.pose.pose.position    = reference.position;
        message.pose.pose.orientation = quaternionFromYaw(reference.yaw);
        message.twist.twist.linear    = reference.velocity;
        message.twist.twist.angular.z = reference.yaw_rate;
        reference_publisher_.publish(message);
    }

    void queueServiceRequests(const FsmOutput& output) {
        std::lock_guard<std::mutex> lock(service_request_mutex_);
        service_requests_.arm      = output.request_arm;
        service_requests_.disarm   = output.request_disarm;
        service_requests_.offboard = output.request_offboard;
    }

    void serviceRequestWorker() {
        while (running_.load() && ros::ok()) {
            ServiceRequests requests;
            {
                std::lock_guard<std::mutex> lock(service_request_mutex_);
                requests = service_requests_;
            }

            double now = steadyNow();
            if ((requests.arm || requests.disarm) &&
                now - last_arm_request_at_ >= config_.requests.retry_interval) {
                // A contradictory request should never be emitted by FsmCore. If
                // it happens, choose the safer action instead of arming.
                const bool request_arm = requests.arm && !requests.disarm;
                const double started_at = steadyNow();
                mavros_msgs::CommandBool request;
                request.request.value = request_arm;
                if (!arming_client_.call(request) || !request.response.success) {
                    ROS_ERROR_THROTTLE(
                        1.0, "PX4 %s request rejected", request_arm ? "arming" : "disarming");
                }
                const double elapsed = steadyNow() - started_at;
                if (elapsed > 0.1) {
                    ROS_WARN(
                        "PX4 %s service call took %.3f s; control setpoint publishing remained "
                        "independent",
                        request_arm ? "arming" : "disarming", elapsed);
                }
                last_arm_request_at_ = steadyNow();
            }

            now = steadyNow();
            if (requests.offboard &&
                now - last_mode_request_at_ >= config_.requests.retry_interval) {
                const double started_at = steadyNow();
                mavros_msgs::SetMode request;
                request.request.custom_mode = "OFFBOARD";
                if (!mode_client_.call(request) || !request.response.mode_sent) {
                    ROS_ERROR_THROTTLE(1.0, "PX4 OFFBOARD request rejected");
                }
                const double elapsed = steadyNow() - started_at;
                if (elapsed > 0.1) {
                    ROS_WARN(
                        "PX4 OFFBOARD service call took %.3f s; control setpoint publishing "
                        "remained independent",
                        elapsed);
                }
                last_mode_request_at_ = steadyNow();
            }

            ros::WallDuration(0.02).sleep();
        }
    }

    void controlCallback(const ros::WallTimerEvent& event) {
        double dt        = 1.0 / config_.node.control_rate;
        if (!event.last_real.isZero()) {
            dt = (event.current_real - event.last_real).toSec();
        }
        const InputSnapshot input = snapshot();

        if (input.events.takeoff || input.events.land) {
            quadrotor_msgs::TakeoffLand lifecycle;
            lifecycle.takeoff_land_cmd = input.events.land ? quadrotor_msgs::TakeoffLand::LAND
                                                           : quadrotor_msgs::TakeoffLand::TAKEOFF;
            planner_lifecycle_publisher_.publish(lifecycle);
        }

        FsmOutput output;
        FsmState previous_state;
        FsmState current_state;
        std::string reason;
        {
            std::lock_guard<std::mutex> lock(core_mutex_);
            previous_state = core_.state();
            output         = core_.step(input, dt);
            current_state  = core_.state();
            reason         = core_.transitionReason();
        }
        if (previous_state != current_state) {
            ROS_WARN(
                "OFFBOARD FSM: %s -> %s: %s (PX4 odom age %.3f s, planner age %.3f s)",
                stateName(previous_state), stateName(current_state), reason.c_str(),
                input.odometry_age, input.planner_age);
        }

        if (output.publish_traj_start_trigger) {
            geometry_msgs::PoseStamped trigger;
            trigger.header.stamp     = ros::Time::now();
            trigger.header.frame_id  = "map";
            trigger.pose.position    = input.livo_position;
            trigger.pose.orientation = quaternionFromYaw(input.livo_yaw);
            traj_start_trigger_publisher_.publish(trigger);
            ROS_INFO(
                "Published trajectory start trigger on %s after takeoff "
                "completed",
                config_.topics.traj_start_trigger.c_str());
        }

        if (output.publish_setpoint) {
            setpoint_publisher_.publish(positionTarget(output.reference));
            publishReference(output.reference);
        }
        queueServiceRequests(output);
    }

    void statusCallback(const ros::WallTimerEvent&) {
        px4ctrl::FsmStatus output;
        output.header.stamp = ros::Time::now();
        double odometry_age      = std::numeric_limits<double>::infinity();
        double planner_age       = std::numeric_limits<double>::infinity();
        double odometry_last_gap = 0.0;
        double odometry_max_gap  = 0.0;

        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            const double now = steadyNow();
            output.enabled      = enabled_;
            output.connected    = have_state_ && state_.connected;
            output.armed        = have_state_ && state_.armed;
            output.px4_mode     = have_state_ ? state_.mode : std::string();
            output.landed_state = have_extended_state_ ? extended_state_.landed_state : 0;
            output.odometry_fresh =
                have_px4_odom_ &&
                receiptFresh(px4_odom_received_at_, config_.timeouts.px4_odometry, now);
            output.planner_fresh =
                have_planner_command_ &&
                receiptFresh(planner_received_at_, config_.timeouts.planner_command, now);
            if (have_px4_odom_) {
                odometry_age = std::max(0.0, now - px4_odom_received_at_);
            }
            if (have_planner_command_) {
                planner_age = std::max(0.0, now - planner_received_at_);
            }
            odometry_last_gap = px4_odom_last_gap_;
            odometry_max_gap  = px4_odom_max_gap_;
        }
        {
            std::lock_guard<std::mutex> lock(core_mutex_);
            output.state                  = static_cast<uint8_t>(core_.state());
            output.state_name             = stateName(core_.state());
            output.reference_position     = core_.reference().position;
            output.reference_yaw          = core_.reference().yaw;
            output.last_transition_reason = core_.transitionReason();
        }
        status_publisher_.publish(output);

        diagnostic_msgs::DiagnosticArray diagnostics;
        diagnostics.header.stamp = output.header.stamp;
        diagnostic_msgs::DiagnosticStatus status;
        status.name        = "px4ctrl/offboard_fsm";
        status.hardware_id = "orin-lidar-01";
        if (!output.connected || !output.odometry_fresh ||
            output.state == px4ctrl::FsmStatus::FAILSAFE ||
            message_interval_required_failure_.load()) {
            status.level   = diagnostic_msgs::DiagnosticStatus::ERROR;
            status.message = message_interval_required_failure_.load()
                                 ? "required MAVLink message-rate request failed"
                                 : output.last_transition_reason;
        } else if (
            output.state == px4ctrl::FsmStatus::WAIT_FCU ||
            output.state == px4ctrl::FsmStatus::PRESTREAM ||
            output.state == px4ctrl::FsmStatus::ARMING ||
            output.state == px4ctrl::FsmStatus::BRAKE ||
            output.state == px4ctrl::FsmStatus::LANDING) {
            status.level   = diagnostic_msgs::DiagnosticStatus::WARN;
            status.message = output.state_name;
        } else {
            status.level   = diagnostic_msgs::DiagnosticStatus::OK;
            status.message = output.state_name;
        }
        auto add_value = [&status](const std::string& key, const std::string& value) {
            diagnostic_msgs::KeyValue item;
            item.key   = key;
            item.value = value;
            status.values.push_back(item);
        };
        add_value("state", output.state_name);
        add_value("px4_mode", output.px4_mode);
        add_value("armed", output.armed ? "true" : "false");
        add_value("odometry_fresh", output.odometry_fresh ? "true" : "false");
        add_value("odometry_age_s", std::to_string(odometry_age));
        add_value("odometry_last_gap_s", std::to_string(odometry_last_gap));
        add_value("odometry_max_gap_s", std::to_string(odometry_max_gap));
        add_value("planner_fresh", output.planner_fresh ? "true" : "false");
        add_value("planner_age_s", std::to_string(planner_age));
        add_value(
            "mavlink_rate_configuration_complete",
            message_interval_complete_.load() ? "true" : "false");
        add_value(
            "mavlink_required_rate_failure",
            message_interval_required_failure_.load() ? "true" : "false");
        diagnostics.status.push_back(status);
        diagnostics_publisher_.publish(diagnostics);
    }

    void messageIntervalWorker() {
        std::size_t index = 0;
        int attempts      = 0;
        while (running_.load() && ros::ok()) {
            if (!config_.message_intervals.enabled || !connected_.load()) {
                ros::WallDuration(0.2).sleep();
                continue;
            }
            if (interval_reset_requested_.exchange(false)) {
                index    = 0;
                attempts = 0;
                message_interval_complete_.store(false);
                message_interval_required_failure_.store(false);
            }
            while (index < config_.message_intervals.messages.size() &&
                   !config_.message_intervals.messages[index].enabled) {
                ++index;
            }
            if (index >= config_.message_intervals.messages.size()) {
                message_interval_complete_.store(true);
                ros::WallDuration(0.5).sleep();
                continue;
            }
            if (!message_interval_client_.exists()) {
                ROS_WARN_THROTTLE(
                    2.0, "Waiting for MAVROS message interval service %s",
                    config_.services.message_interval.c_str());
                ros::WallDuration(config_.message_intervals.retry_delay).sleep();
                continue;
            }

            const MessageIntervalConfig& item = config_.message_intervals.messages[index];
            mavros_msgs::MessageInterval request;
            request.request.message_id   = item.id;
            request.request.message_rate = item.rate;
            const bool called            = message_interval_client_.call(request);
            if (called && request.response.success) {
                ROS_INFO(
                    "Configured MAVLink message %s (id=%u) to %.1f Hz", item.name.c_str(), item.id,
                    item.rate);
                ++index;
                attempts = 0;
            } else {
                ++attempts;
                ROS_WARN(
                    "Failed to configure MAVLink message %s (id=%u) to %.1f Hz "
                    "(%d/%d)",
                    item.name.c_str(), item.id, item.rate, attempts,
                    config_.message_intervals.retry_count);
                if (attempts >= config_.message_intervals.retry_count) {
                    if (item.required) {
                        message_interval_required_failure_.store(true);
                        ROS_ERROR(
                            "Required MAVLink message-rate configuration failed: %s",
                            item.name.c_str());
                    }
                    ++index;
                    attempts = 0;
                }
            }
            ros::WallDuration(config_.message_intervals.retry_delay).sleep();
        }
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    const Config config_;
    FsmCore core_;

    ros::Publisher setpoint_publisher_;
    ros::Publisher status_publisher_;
    ros::Publisher reference_publisher_;
    ros::Publisher planner_lifecycle_publisher_;
    ros::Publisher traj_start_trigger_publisher_;
    ros::Publisher diagnostics_publisher_;
    ros::Subscriber state_subscriber_;
    ros::Subscriber extended_state_subscriber_;
    ros::Subscriber px4_odom_subscriber_;
    ros::Subscriber livo_odom_subscriber_;
    ros::Subscriber planner_subscriber_;
    ros::Subscriber planner_emergency_subscriber_;
    ros::Subscriber command_subscriber_;
    ros::Subscriber rc_subscriber_;
    ros::ServiceClient arming_client_;
    ros::ServiceClient mode_client_;
    ros::ServiceClient message_interval_client_;
    ros::ServiceServer enable_service_;
    ros::ServiceServer reset_service_;
    ros::WallTimer control_timer_;
    ros::WallTimer status_timer_;

    std::mutex data_mutex_;
    std::mutex core_mutex_;
    std::mutex service_request_mutex_;
    mavros_msgs::State state_;
    mavros_msgs::ExtendedState extended_state_;
    nav_msgs::Odometry px4_odom_;
    nav_msgs::Odometry livo_odom_;
    quadrotor_msgs::PositionCommand planner_command_;
    std::vector<uint16_t> rc_channels_;
    bool have_state_{false};
    bool have_extended_state_{false};
    bool have_px4_odom_{false};
    bool have_livo_odom_{false};
    bool have_planner_command_{false};
    bool enabled_{true};
    double state_received_at_{0.0};
    double extended_state_received_at_{0.0};
    double px4_odom_received_at_{0.0};
    double px4_odom_last_gap_{0.0};
    double px4_odom_max_gap_{0.0};
    double livo_odom_received_at_{0.0};
    double planner_received_at_{0.0};
    double rc_received_at_{0.0};
    uint64_t planner_sequence_{0};
    Events pending_events_;
    DebouncedSwitch takeoff_land_switch_;

    double last_arm_request_at_{0.0};
    double last_mode_request_at_{0.0};
    ServiceRequests service_requests_;
    std::atomic<bool> connected_{false};
    std::atomic<bool> ever_connected_{false};
    std::atomic<bool> running_{false};
    std::atomic<bool> interval_reset_requested_{false};
    std::atomic<bool> message_interval_complete_{false};
    std::atomic<bool> message_interval_required_failure_{false};
    std::thread service_request_thread_;
    std::thread message_interval_thread_;
};

}  // namespace px4ctrl

int main(int argc, char** argv) {
    ros::init(argc, argv, "px4_offboard_fsm");
    try {
        px4ctrl::OffboardFsmNode node;
        ros::AsyncSpinner spinner(2);
        spinner.start();
        ros::waitForShutdown();
    } catch (const std::exception& error) {
        ROS_FATAL("Failed to start PX4 OFFBOARD FSM: %s", error.what());
        return 1;
    }
    return 0;
}
