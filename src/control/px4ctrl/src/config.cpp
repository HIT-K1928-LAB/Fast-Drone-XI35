#include <XmlRpcValue.h>
#include <cmath>
#include <px4ctrl/offboard_fsm/config.h>
#include <sstream>
#include <stdexcept>

namespace px4ctrl {
namespace {

template <typename T>
void loadParam(const ros::NodeHandle& nh, const std::string& key, T& value) {
    nh.param(key, value, value);
}

double number(const XmlRpc::XmlRpcValue& value, const std::string& field) {
    if (!value.hasMember(field)) {
        throw std::runtime_error("message_intervals entry is missing '" + field + "'");
    }
    const XmlRpc::XmlRpcValue& item = value[field];
    if (item.getType() == XmlRpc::XmlRpcValue::TypeInt) {
        return static_cast<int>(item);
    }
    if (item.getType() == XmlRpc::XmlRpcValue::TypeDouble) {
        return static_cast<double>(item);
    }
    throw std::runtime_error("message_intervals field '" + field + "' must be numeric");
}

bool boolean(const XmlRpc::XmlRpcValue& value, const std::string& field, const bool fallback) {
    if (!value.hasMember(field)) {
        return fallback;
    }
    if (value[field].getType() != XmlRpc::XmlRpcValue::TypeBoolean) {
        throw std::runtime_error("message_intervals field '" + field + "' must be boolean");
    }
    return static_cast<bool>(value[field]);
}

std::string text(
    const XmlRpc::XmlRpcValue& value, const std::string& field, const std::string& fallback) {
    if (!value.hasMember(field)) {
        return fallback;
    }
    if (value[field].getType() != XmlRpc::XmlRpcValue::TypeString) {
        throw std::runtime_error("message_intervals field '" + field + "' must be a string");
    }
    return static_cast<std::string>(value[field]);
}

void positive(const double value, const std::string& name) {
    if (!std::isfinite(value) || value <= 0.0) {
        throw std::runtime_error(name + " must be finite and positive");
    }
}

}  // namespace

Config Config::load(const ros::NodeHandle& nh) {
    Config c;
    if (!nh.getParam("schema_version", c.schema_version)) {
        throw std::runtime_error(
            "missing schema_version; this launch may still use a legacy px4ctrl "
            "configuration");
    }
    loadParam(nh, "node/control_rate", c.node.control_rate);
    loadParam(nh, "node/status_rate", c.node.status_rate);
    loadParam(nh, "node/start_enabled", c.node.start_enabled);
    loadParam(nh, "node/prestream_when_ready", c.node.prestream_when_ready);
    loadParam(nh, "node/prestream_min_time", c.node.prestream_min_time);
    loadParam(nh, "node/request_offboard_automatically", c.node.request_offboard_automatically);

    loadParam(nh, "topics/mavros_state", c.topics.mavros_state);
    loadParam(nh, "topics/extended_state", c.topics.extended_state);
    loadParam(nh, "topics/px4_odometry", c.topics.px4_odometry);
    loadParam(nh, "topics/livo_odometry", c.topics.livo_odometry);
    loadParam(nh, "topics/rc_input", c.topics.rc_input);
    loadParam(nh, "topics/planner_command", c.topics.planner_command);
    loadParam(nh, "topics/planner_emergency_hover", c.topics.planner_emergency_hover);
    loadParam(nh, "topics/fsm_command", c.topics.fsm_command);
    loadParam(nh, "topics/planner_lifecycle", c.topics.planner_lifecycle);
    loadParam(nh, "topics/traj_start_trigger", c.topics.traj_start_trigger);
    loadParam(nh, "topics/setpoint", c.topics.setpoint);
    loadParam(nh, "topics/status", c.topics.status);
    loadParam(nh, "topics/reference", c.topics.reference);
    loadParam(nh, "topics/diagnostics", c.topics.diagnostics);

    loadParam(nh, "services/arming", c.services.arming);
    loadParam(nh, "services/set_mode", c.services.set_mode);
    loadParam(nh, "services/message_interval", c.services.message_interval);

    loadParam(nh, "timeouts/mavros_state", c.timeouts.mavros_state);
    loadParam(nh, "timeouts/extended_state", c.timeouts.extended_state);
    loadParam(nh, "timeouts/px4_odometry", c.timeouts.px4_odometry);
    loadParam(nh, "timeouts/livo_odometry", c.timeouts.livo_odometry);
    loadParam(nh, "timeouts/planner_command", c.timeouts.planner_command);
    loadParam(nh, "timeouts/rc_input", c.timeouts.rc_input);
    loadParam(nh, "timeouts/arm_request", c.timeouts.arm_request);
    loadParam(nh, "timeouts/disarm_request", c.timeouts.disarm_request);
    loadParam(nh, "timeouts/takeoff", c.timeouts.takeoff);
    loadParam(nh, "timeouts/landing", c.timeouts.landing);

    loadParam(nh, "rc/enabled", c.rc.enabled);
    loadParam(nh, "rc/index_base", c.rc.index_base);
    loadParam(nh, "rc/debounce_time", c.rc.debounce_time);
    loadParam(nh, "rc/require_release_before_trigger", c.rc.require_release_before_trigger);
    loadParam(nh, "rc/takeoff_land/channel", c.rc.takeoff_land.channel);
    loadParam(nh, "rc/takeoff_land/reversed", c.rc.takeoff_land.reversed);
    loadParam(nh, "rc/takeoff_land/trigger_pwm", c.rc.takeoff_land.trigger_pwm);
    loadParam(nh, "rc/takeoff_land/release_pwm", c.rc.takeoff_land.release_pwm);

    loadParam(nh, "takeoff/relative_height", c.takeoff.relative_height);
    loadParam(nh, "takeoff/max_velocity", c.takeoff.max_velocity);
    loadParam(nh, "takeoff/max_acceleration", c.takeoff.max_acceleration);
    loadParam(nh, "takeoff/max_jerk", c.takeoff.max_jerk);
    loadParam(nh, "takeoff/position_tolerance", c.takeoff.position_tolerance);
    loadParam(nh, "takeoff/velocity_tolerance", c.takeoff.velocity_tolerance);
    loadParam(nh, "takeoff/stable_time", c.takeoff.stable_time);

    loadParam(nh, "planner/activation_messages", c.planner.activation_messages);
    loadParam(nh, "planner/max_position_jump", c.planner.max_position_jump);
    loadParam(nh, "planner/max_yaw_jump_deg", c.planner.max_yaw_jump_deg);
    loadParam(nh, "planner/max_velocity", c.planner.max_velocity);
    loadParam(nh, "planner/max_acceleration", c.planner.max_acceleration);
    loadParam(nh, "planner/brake_acceleration", c.planner.brake_acceleration);
    loadParam(nh, "planner/brake_velocity_tolerance", c.planner.brake_velocity_tolerance);
    loadParam(nh, "planner/brake_stable_time", c.planner.brake_stable_time);

    loadParam(nh, "landing/strategy", c.landing.strategy);
    loadParam(nh, "landing/max_velocity", c.landing.max_velocity);
    loadParam(nh, "landing/max_acceleration", c.landing.max_acceleration);
    loadParam(nh, "landing/flare_height", c.landing.flare_height);
    loadParam(nh, "landing/flare_velocity", c.landing.flare_velocity);
    loadParam(nh, "landing/touchdown_confirm_time", c.landing.touchdown_confirm_time);
    loadParam(nh, "landing/auto_disarm", c.landing.auto_disarm);
    loadParam(nh, "landing/disarm_delay", c.landing.disarm_delay);
    loadParam(nh, "landing/max_descent_distance", c.landing.max_descent_distance);

    loadParam(
        nh, "frames/align_planner_on_offboard_entry", c.frames.align_planner_on_offboard_entry);
    loadParam(nh, "limits/max_horizontal_distance", c.limits.max_horizontal_distance);
    loadParam(nh, "limits/max_height_above_origin", c.limits.max_height_above_origin);
    loadParam(nh, "limits/min_height_below_origin", c.limits.min_height_below_origin);
    loadParam(
        nh, "failsafe/stop_setpoints_on_critical_failure",
        c.failsafe.stop_setpoints_on_critical_failure);
    loadParam(nh, "requests/retry_interval", c.requests.retry_interval);

    loadParam(nh, "message_intervals/enabled", c.message_intervals.enabled);
    loadParam(
        nh, "message_intervals/reapply_on_reconnect", c.message_intervals.reapply_on_reconnect);
    loadParam(nh, "message_intervals/retry_count", c.message_intervals.retry_count);
    loadParam(nh, "message_intervals/retry_delay", c.message_intervals.retry_delay);

    XmlRpc::XmlRpcValue entries;
    if (nh.getParam("message_intervals/messages", entries)) {
        if (entries.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            throw std::runtime_error("message_intervals/messages must be a list");
        }
        c.message_intervals.messages.clear();
        for (int i = 0; i < entries.size(); ++i) {
            if (entries[i].getType() != XmlRpc::XmlRpcValue::TypeStruct) {
                throw std::runtime_error("each message_intervals/messages entry must be a map");
            }
            MessageIntervalConfig message;
            const double message_id = number(entries[i], "id");
            if (message_id < 0.0 || message_id > 4294967295.0 ||
                std::floor(message_id) != message_id) {
                throw std::runtime_error("message_intervals id must be a non-negative integer");
            }
            message.id       = static_cast<uint32_t>(message_id);
            message.rate     = number(entries[i], "rate");
            message.name     = text(entries[i], "name", std::to_string(message.id));
            message.enabled  = boolean(entries[i], "enabled", true);
            message.required = boolean(entries[i], "required", false);
            c.message_intervals.messages.push_back(message);
        }
    }

    c.validate();
    return c;
}

void Config::validate() const {
    if (schema_version != 1) {
        throw std::runtime_error("unsupported px4ctrl schema_version");
    }
    positive(node.control_rate, "node/control_rate");
    if (node.control_rate < 20.0) {
        throw std::runtime_error("node/control_rate must be at least 20 Hz");
    }
    positive(node.status_rate, "node/status_rate");
    positive(node.prestream_min_time, "node/prestream_min_time");

    positive(timeouts.mavros_state, "timeouts/mavros_state");
    positive(timeouts.extended_state, "timeouts/extended_state");
    positive(timeouts.px4_odometry, "timeouts/px4_odometry");
    positive(timeouts.livo_odometry, "timeouts/livo_odometry");
    positive(timeouts.planner_command, "timeouts/planner_command");
    positive(timeouts.rc_input, "timeouts/rc_input");
    positive(timeouts.arm_request, "timeouts/arm_request");
    positive(timeouts.disarm_request, "timeouts/disarm_request");
    positive(timeouts.takeoff, "timeouts/takeoff");
    positive(timeouts.landing, "timeouts/landing");

    positive(takeoff.relative_height, "takeoff/relative_height");
    positive(takeoff.max_velocity, "takeoff/max_velocity");
    positive(takeoff.max_acceleration, "takeoff/max_acceleration");
    positive(takeoff.max_jerk, "takeoff/max_jerk");
    positive(takeoff.stable_time, "takeoff/stable_time");
    positive(planner.max_velocity, "planner/max_velocity");
    positive(planner.max_acceleration, "planner/max_acceleration");
    positive(planner.brake_acceleration, "planner/brake_acceleration");
    positive(landing.max_velocity, "landing/max_velocity");
    positive(landing.max_acceleration, "landing/max_acceleration");
    positive(landing.max_descent_distance, "landing/max_descent_distance");
    positive(limits.max_horizontal_distance, "limits/max_horizontal_distance");
    positive(limits.max_height_above_origin, "limits/max_height_above_origin");
    if (takeoff.relative_height > limits.max_height_above_origin) {
        throw std::runtime_error("takeoff/relative_height exceeds limits/max_height_above_origin");
    }
    if (!std::isfinite(limits.min_height_below_origin) || limits.min_height_below_origin > 0.0) {
        throw std::runtime_error("limits/min_height_below_origin must be finite and <= 0");
    }

    if (planner.activation_messages < 1) {
        throw std::runtime_error("planner/activation_messages must be >= 1");
    }
    if (landing.strategy != "offboard_descent") {
        throw std::runtime_error("only landing/strategy=offboard_descent is currently supported");
    }

    if (rc.enabled) {
        if (rc.index_base != 0 && rc.index_base != 1) {
            throw std::runtime_error("rc/index_base must be 0 or 1");
        }
        const RcSwitchConfig& item = rc.takeoff_land;
        if (item.channel - rc.index_base < 0) {
            throw std::runtime_error("RC channel resolves to a negative index");
        }
        if (item.trigger_pwm < 800 || item.trigger_pwm > 2200 || item.release_pwm < 800 ||
            item.release_pwm > 2200) {
            throw std::runtime_error("RC PWM thresholds must be in [800, 2200]");
        }
        if ((!item.reversed && item.trigger_pwm <= item.release_pwm) ||
            (item.reversed && item.trigger_pwm >= item.release_pwm)) {
            throw std::runtime_error("RC trigger/release thresholds do not match reversed setting");
        }
    }

    if (message_intervals.retry_count < 1) {
        throw std::runtime_error("message_intervals/retry_count must be >= 1");
    }
    positive(message_intervals.retry_delay, "message_intervals/retry_delay");
    for (const MessageIntervalConfig& message : message_intervals.messages) {
        if (message.enabled) {
            positive(message.rate, "message interval rate for " + message.name);
        }
    }
}

}  // namespace px4ctrl
