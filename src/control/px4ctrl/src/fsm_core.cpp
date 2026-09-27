#include <algorithm>
#include <cmath>
#include <mavros_msgs/ExtendedState.h>
#include <px4ctrl/offboard_fsm/fsm_core.h>

namespace px4ctrl {
namespace {

constexpr double kPi = 3.14159265358979323846;

double clamp(const double value, const double low, const double high) {
    return std::max(low, std::min(value, high));
}

double approach(const double value, const double target, const double max_delta) {
    if (value < target) {
        return std::min(value + max_delta, target);
    }
    return std::max(value - max_delta, target);
}

double wrapPi(const double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

double norm3(const geometry_msgs::Vector3& value) {
    return std::sqrt(value.x * value.x + value.y * value.y + value.z * value.z);
}

double distance3(const geometry_msgs::Point& a, const geometry_msgs::Point& b) {
    const double dx = a.x - b.x;
    const double dy = a.y - b.y;
    const double dz = a.z - b.z;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool finiteCommand(const quadrotor_msgs::PositionCommand& command) {
    const double values[] = {
        command.position.x,     command.position.y, command.position.z,     command.velocity.x,
        command.velocity.y,     command.velocity.z, command.acceleration.x, command.acceleration.y,
        command.acceleration.z, command.yaw,        command.yaw_dot};
    for (const double value : values) {
        if (!std::isfinite(value)) {
            return false;
        }
    }
    return true;
}

void rotateZ(
    const double x, const double y, const double yaw, double& rotated_x, double& rotated_y) {
    const double cosine = std::cos(yaw);
    const double sine   = std::sin(yaw);
    rotated_x           = cosine * x - sine * y;
    rotated_y           = sine * x + cosine * y;
}

}  // namespace

const char* stateName(const FsmState state) {
    switch (state) {
        case FsmState::WAIT_FCU:
            return "WAIT_FCU";
        case FsmState::PRESTREAM:
            return "PRESTREAM";
        case FsmState::GROUND:
            return "GROUND";
        case FsmState::ARMING:
            return "ARMING";
        case FsmState::TAKEOFF:
            return "TAKEOFF";
        case FsmState::HOVER:
            return "HOVER";
        case FsmState::EXTERNAL:
            return "EXTERNAL";
        case FsmState::BRAKE:
            return "BRAKE";
        case FsmState::LANDING:
            return "LANDING";
        case FsmState::DISARMING:
            return "DISARMING";
        case FsmState::FAILSAFE:
            return "FAILSAFE";
    }
    return "UNKNOWN";
}

Events rcTakeoffLandEvent(
    const FsmState state, const bool extended_state_fresh, const uint8_t landed_state) {
    Events events;
    switch (state) {
        case FsmState::GROUND:
            if (extended_state_fresh &&
                landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                events.takeoff = true;
            }
            break;
        case FsmState::ARMING:
        case FsmState::TAKEOFF:
        case FsmState::HOVER:
        case FsmState::EXTERNAL:
        case FsmState::BRAKE:
            events.land = true;
            break;
        case FsmState::WAIT_FCU:
        case FsmState::PRESTREAM:
        case FsmState::LANDING:
        case FsmState::DISARMING:
        case FsmState::FAILSAFE:
            break;
    }
    return events;
}

FsmCore::FsmCore(const Config& config) : config_(config) {
    requested_takeoff_height_ = config_.takeoff.relative_height;
}

bool FsmCore::activeState() const {
    return state_ == FsmState::GROUND || state_ == FsmState::ARMING ||
           state_ == FsmState::TAKEOFF || state_ == FsmState::HOVER ||
           state_ == FsmState::EXTERNAL || state_ == FsmState::BRAKE ||
           state_ == FsmState::LANDING || state_ == FsmState::DISARMING;
}

bool FsmCore::healthyForControl(const InputSnapshot& input) const {
    return input.state_fresh && input.connected && input.odometry_fresh;
}

void FsmCore::captureHold(const InputSnapshot& input) {
    hold_position_                       = input.position;
    hold_yaw_                            = input.yaw;
    reference_.position                  = hold_position_;
    reference_.velocity                  = geometry_msgs::Vector3();
    reference_.acceleration              = geometry_msgs::Vector3();
    reference_.yaw                       = hold_yaw_;
    reference_.yaw_rate                  = 0.0;
    reference_.use_velocity_acceleration = false;
}

void FsmCore::capturePlannerAlignment(const InputSnapshot& input) {
    if (!config_.frames.align_planner_on_offboard_entry) {
        yaw_offset_              = 0.0;
        translation_             = geometry_msgs::Vector3();
        planner_alignment_valid_ = true;
        return;
    }
    if (!input.livo_odometry_fresh) {
        planner_alignment_valid_ = false;
        return;
    }

    yaw_offset_      = wrapPi(input.yaw - input.livo_yaw);
    double rotated_x = 0.0;
    double rotated_y = 0.0;
    rotateZ(input.livo_position.x, input.livo_position.y, yaw_offset_, rotated_x, rotated_y);
    translation_.x           = input.position.x - rotated_x;
    translation_.y           = input.position.y - rotated_y;
    translation_.z           = input.position.z - input.livo_position.z;
    planner_alignment_valid_ = true;
}

void FsmCore::transition(
    const FsmState next, const std::string& reason, const InputSnapshot& input) {
    if (state_ == next) {
        return;
    }
    state_             = next;
    state_entered_at_  = input.now;
    transition_reason_ = reason;
    stable_since_      = 0.0;

    switch (next) {
        case FsmState::PRESTREAM:
        case FsmState::GROUND:
            captureHold(input);
            if (next == FsmState::GROUND) {
                safety_origin_      = input.position;
                have_safety_origin_ = true;
                capturePlannerAlignment(input);
            }
            break;
        case FsmState::ARMING:
            captureHold(input);
            break;
        case FsmState::TAKEOFF:
            takeoff_origin_                      = input.position;
            have_takeoff_origin_                 = true;
            hold_position_                       = input.position;
            hold_yaw_                            = input.yaw;
            reference_.position                  = input.position;
            reference_.velocity                  = geometry_msgs::Vector3();
            reference_.acceleration              = geometry_msgs::Vector3();
            reference_.yaw                       = hold_yaw_;
            reference_.yaw_rate                  = 0.0;
            reference_.use_velocity_acceleration = true;
            vertical_velocity_                   = 0.0;
            vertical_acceleration_               = 0.0;
            break;
        case FsmState::HOVER:
            captureHold(input);
            if (!have_safety_origin_) {
                safety_origin_      = input.position;
                have_safety_origin_ = true;
            }
            capturePlannerAlignment(input);
            planner_accept_after_     = input.now;
            last_planner_sequence_    = input.planner_sequence;
            planner_activation_count_ = 0;
            break;
        case FsmState::EXTERNAL:
            break;
        case FsmState::BRAKE:
            reference_.position                  = input.position;
            reference_.velocity                  = input.velocity_world;
            reference_.acceleration              = geometry_msgs::Vector3();
            reference_.yaw                       = input.yaw;
            reference_.yaw_rate                  = 0.0;
            reference_.use_velocity_acceleration = true;
            brake_velocity_                      = input.velocity_world;
            break;
        case FsmState::LANDING:
            hold_position_                       = input.position;
            hold_yaw_                            = input.yaw;
            reference_.position                  = input.position;
            reference_.velocity                  = geometry_msgs::Vector3();
            reference_.acceleration              = geometry_msgs::Vector3();
            reference_.yaw                       = hold_yaw_;
            reference_.yaw_rate                  = 0.0;
            reference_.use_velocity_acceleration = true;
            vertical_velocity_                   = 0.0;
            vertical_acceleration_               = 0.0;
            touchdown_since_                     = 0.0;
            landing_control_elapsed_             = 0.0;
            landing_floor_limit_                 = have_takeoff_origin_
                                                       ? takeoff_origin_.z - 0.25
                                                       : input.position.z - config_.landing.max_descent_distance;
            break;
        case FsmState::DISARMING:
            break;
        case FsmState::FAILSAFE:
        case FsmState::WAIT_FCU:
            // Never resume a pre-failure automatic takeoff after FCU feedback
            // recovers. A fresh takeoff command is required.
            pending_takeoff_ = false;
            break;
    }
}

bool FsmCore::streamReady(const InputSnapshot& input) const {
    return stream_started_at_ > 0.0 &&
           input.now - stream_started_at_ >= config_.node.prestream_min_time;
}

Reference FsmCore::transformedPlannerReference(const InputSnapshot& input) const {
    Reference output;
    double x = 0.0;
    double y = 0.0;
    rotateZ(input.planner.position.x, input.planner.position.y, yaw_offset_, x, y);
    output.position.x = x + translation_.x;
    output.position.y = y + translation_.y;
    output.position.z = input.planner.position.z + translation_.z;

    rotateZ(input.planner.velocity.x, input.planner.velocity.y, yaw_offset_, x, y);
    output.velocity.x = x;
    output.velocity.y = y;
    output.velocity.z = input.planner.velocity.z;

    rotateZ(input.planner.acceleration.x, input.planner.acceleration.y, yaw_offset_, x, y);
    output.acceleration.x            = x;
    output.acceleration.y            = y;
    output.acceleration.z            = input.planner.acceleration.z;
    output.yaw                       = wrapPi(input.planner.yaw + yaw_offset_);
    output.yaw_rate                  = input.planner.yaw_dot;
    output.use_velocity_acceleration = true;
    return output;
}

bool FsmCore::plannerCommandValid(const InputSnapshot& input, const bool activation_check) const {
    if (!input.planner_fresh || !planner_alignment_valid_ ||
        input.planner_received_at <= planner_accept_after_ ||
        input.planner.trajectory_flag != quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY ||
        !finiteCommand(input.planner) ||
        norm3(input.planner.velocity) > config_.planner.max_velocity ||
        norm3(input.planner.acceleration) > config_.planner.max_acceleration) {
        return false;
    }

    if (activation_check) {
        const Reference candidate = transformedPlannerReference(input);
        if (distance3(candidate.position, input.position) > config_.planner.max_position_jump ||
            std::abs(wrapPi(candidate.yaw - input.yaw)) >
                config_.planner.max_yaw_jump_deg * kPi / 180.0) {
            return false;
        }
    }
    if (have_safety_origin_) {
        const Reference candidate    = transformedPlannerReference(input);
        const double dx              = candidate.position.x - safety_origin_.x;
        const double dy              = candidate.position.y - safety_origin_.y;
        const double relative_height = candidate.position.z - safety_origin_.z;
        if (std::hypot(dx, dy) > config_.limits.max_horizontal_distance ||
            relative_height > config_.limits.max_height_above_origin ||
            relative_height < config_.limits.min_height_below_origin) {
            return false;
        }
    }
    return true;
}

void FsmCore::updateVerticalProfile(
    const double target_z, const double max_velocity, const double max_acceleration,
    const double max_jerk, const double dt) {
    const double error     = target_z - reference_.position.z;
    const double direction = error >= 0.0 ? 1.0 : -1.0;
    const double braking_velocity =
        std::sqrt(std::max(0.0, 2.0 * max_acceleration * std::abs(error)));
    const double desired_velocity     = direction * std::min(max_velocity, braking_velocity);
    const double desired_acceleration = clamp(
        (desired_velocity - vertical_velocity_) / std::max(dt, 1e-3), -max_acceleration,
        max_acceleration);
    vertical_acceleration_ = approach(vertical_acceleration_, desired_acceleration, max_jerk * dt);
    vertical_velocity_ += vertical_acceleration_ * dt;
    vertical_velocity_ = clamp(vertical_velocity_, -max_velocity, max_velocity);

    const double next_z = reference_.position.z + vertical_velocity_ * dt;
    if ((target_z - reference_.position.z) * (target_z - next_z) <= 0.0) {
        reference_.position.z  = target_z;
        vertical_velocity_     = 0.0;
        vertical_acceleration_ = 0.0;
    } else {
        reference_.position.z = next_z;
    }
    reference_.velocity.z     = vertical_velocity_;
    reference_.acceleration.z = vertical_acceleration_;
}

void FsmCore::updateLandingProfile(const InputSnapshot& input, const double dt) {
    double max_velocity = config_.landing.max_velocity;
    if (have_takeoff_origin_ &&
        input.position.z - takeoff_origin_.z <= config_.landing.flare_height) {
        max_velocity = std::min(max_velocity, config_.landing.flare_velocity);
    }
    updateVerticalProfile(
        landing_floor_limit_, max_velocity, config_.landing.max_acceleration,
        2.0 * config_.landing.max_acceleration, dt);

    // Keep expressing a downward velocity intent after the position reference
    // reaches its below-ground safety limit. PX4's multicopter land detector
    // requires an active descent setpoint (`in_descend`) before it can assert
    // ground_contact/landed. Sending zero velocity here leaves the vehicle on
    // the floor at minimum thrust without ever allowing PX4 to confirm landing.
    // Position remains clamped, so this does not integrate the reference farther
    // below the takeoff surface.
    constexpr double kFloorTolerance = 1e-6;
    if (reference_.position.z <= landing_floor_limit_ + kFloorTolerance) {
        reference_.position.z     = landing_floor_limit_;
        reference_.velocity.z     = -max_velocity;
        reference_.acceleration.z = 0.0;
    }
    reference_.position.x = hold_position_.x;
    reference_.position.y = hold_position_.y;
    reference_.yaw        = hold_yaw_;
}

void FsmCore::updateBrakeProfile(const double dt) {
    const double max_delta = config_.planner.brake_acceleration * dt;
    brake_velocity_.x      = approach(brake_velocity_.x, 0.0, max_delta);
    brake_velocity_.y      = approach(brake_velocity_.y, 0.0, max_delta);
    brake_velocity_.z      = approach(brake_velocity_.z, 0.0, max_delta);
    reference_.position.x += brake_velocity_.x * dt;
    reference_.position.y += brake_velocity_.y * dt;
    reference_.position.z += brake_velocity_.z * dt;
    reference_.velocity     = brake_velocity_;
    reference_.acceleration = geometry_msgs::Vector3();
}

void FsmCore::notePublishing(const InputSnapshot& input, const bool publishing) {
    const double expected_period = 1.0 / config_.node.control_rate;
    if (!publishing) {
        stream_started_at_ = 0.0;
        last_publish_at_   = 0.0;
        return;
    }
    if (last_publish_at_ <= 0.0 || input.now - last_publish_at_ > 3.0 * expected_period) {
        stream_started_at_ = input.now;
    }
    last_publish_at_ = input.now;
}

FsmOutput FsmCore::step(const InputSnapshot& input, const double raw_dt) {
    FsmOutput output;
    const double dt = clamp(raw_dt, 1e-3, 0.1);

    if (!input.enabled) {
        transition(FsmState::WAIT_FCU, "FSM disabled", input);
        notePublishing(input, false);
        return output;
    }

    const bool landing_sequence = state_ == FsmState::LANDING || state_ == FsmState::DISARMING;

    if ((!input.state_fresh || !input.connected) && !landing_sequence) {
        if (activeState()) {
            transition(FsmState::FAILSAFE, "MAVROS state lost", input);
        } else {
            transition(FsmState::WAIT_FCU, "waiting for MAVROS", input);
        }
        notePublishing(input, false);
        return output;
    }

    if (!input.odometry_fresh && !landing_sequence) {
        if (activeState()) {
            transition(FsmState::FAILSAFE, "PX4 local odometry stale", input);
        } else {
            transition(FsmState::WAIT_FCU, "waiting for PX4 local odometry", input);
        }
        notePublishing(input, false);
        return output;
    }

    if (activeState() && !landing_sequence && input.mode != "OFFBOARD") {
        transition(FsmState::PRESTREAM, "pilot or PX4 exited OFFBOARD", input);
    }
    const bool flight_state = state_ == FsmState::TAKEOFF || state_ == FsmState::HOVER ||
                              state_ == FsmState::EXTERNAL || state_ == FsmState::BRAKE ||
                              state_ == FsmState::LANDING;
    if (input.state_fresh && flight_state && !input.armed) {
        if (input.extended_state_fresh &&
            input.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
            transition(FsmState::GROUND, "vehicle disarmed on ground", input);
        } else {
            transition(FsmState::FAILSAFE, "unexpected in-flight disarm", input);
        }
    }
    if (state_ == FsmState::FAILSAFE && input.mode != "OFFBOARD") {
        transition(FsmState::PRESTREAM, "OFFBOARD exited after failsafe", input);
    }
    if (input.events.reset && state_ == FsmState::FAILSAFE && healthyForControl(input)) {
        transition(FsmState::PRESTREAM, "failsafe reset requested", input);
    }

    if (state_ == FsmState::WAIT_FCU && healthyForControl(input)) {
        transition(FsmState::PRESTREAM, "MAVROS and odometry ready", input);
    }

    switch (state_) {
        case FsmState::WAIT_FCU:
            break;

        case FsmState::PRESTREAM:
            captureHold(input);
            output.publish_setpoint = config_.node.prestream_when_ready;
            output.reference        = reference_;
            if (input.events.land || input.events.abort) {
                pending_takeoff_ = false;
            }
            if (input.events.takeoff && config_.node.request_offboard_automatically) {
                pending_takeoff_          = true;
                requested_takeoff_height_ = input.events.takeoff_height > 0.0
                                                ? input.events.takeoff_height
                                                : config_.takeoff.relative_height;
            }
            if (pending_takeoff_ && config_.node.request_offboard_automatically &&
                streamReady(input)) {
                // Keep requesting at the node's rate-limited service cadence until
                // PX4 actually reports OFFBOARD, or a land/abort event cancels it.
                output.request_offboard = true;
            }
            if (input.mode == "OFFBOARD" && input.extended_state_fresh) {
                if (input.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                    transition(FsmState::GROUND, "OFFBOARD entered on ground", input);
                } else if (
                    input.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR ||
                    input.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_TAKEOFF ||
                    input.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_LANDING) {
                    transition(FsmState::HOVER, "OFFBOARD entered in flight", input);
                }
            }
            break;

        case FsmState::GROUND:
            captureHold(input);
            output.publish_setpoint = true;
            output.reference        = reference_;
            if ((input.events.takeoff || pending_takeoff_) && streamReady(input) &&
                input.extended_state_fresh &&
                input.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                const double requested_height =
                    pending_takeoff_
                        ? requested_takeoff_height_
                        : (input.events.takeoff_height > 0.0 ? input.events.takeoff_height
                                                             : config_.takeoff.relative_height);
                if (requested_height > config_.limits.max_height_above_origin) {
                    pending_takeoff_   = false;
                    transition_reason_ = "takeoff request rejected by height limit";
                    break;
                }
                if (!pending_takeoff_) {
                    requested_takeoff_height_ = requested_height;
                }
                pending_takeoff_ = false;
                transition(
                    input.armed ? FsmState::TAKEOFF : FsmState::ARMING,
                    input.armed ? "takeoff requested while already armed" : "takeoff requested",
                    input);
            }
            break;

        case FsmState::ARMING:
            output.publish_setpoint = true;
            output.reference        = reference_;
            if (input.events.land || input.events.abort) {
                if (input.armed) {
                    transition(FsmState::DISARMING, "takeoff cancelled", input);
                } else {
                    transition(FsmState::GROUND, "takeoff cancelled", input);
                }
            } else if (input.armed) {
                transition(FsmState::TAKEOFF, "vehicle armed", input);
            } else if (input.now - state_entered_at_ > config_.timeouts.arm_request) {
                transition(FsmState::GROUND, "arming timed out", input);
            } else {
                output.request_arm = true;
            }
            break;

        case FsmState::TAKEOFF: {
            if (input.events.land || input.events.abort) {
                transition(FsmState::LANDING, "landing requested during takeoff", input);
                break;
            }
            if (input.now - state_entered_at_ > config_.timeouts.takeoff) {
                transition(FsmState::LANDING, "takeoff timed out", input);
                break;
            }
            const double target_z = takeoff_origin_.z + requested_takeoff_height_;
            reference_.position.x = takeoff_origin_.x;
            reference_.position.y = takeoff_origin_.y;
            reference_.yaw        = hold_yaw_;
            updateVerticalProfile(
                target_z, config_.takeoff.max_velocity, config_.takeoff.max_acceleration,
                config_.takeoff.max_jerk, dt);
            output.publish_setpoint = true;
            output.reference        = reference_;

            if (std::abs(target_z - input.position.z) <= config_.takeoff.position_tolerance &&
                std::abs(input.velocity_world.z) <= config_.takeoff.velocity_tolerance) {
                if (stable_since_ <= 0.0) {
                    stable_since_ = input.now;
                }
                if (input.now - stable_since_ >= config_.takeoff.stable_time) {
                    transition(FsmState::HOVER, "takeoff completed", input);
                    output.publish_traj_start_trigger = true;
                }
            } else {
                stable_since_ = 0.0;
            }
            break;
        }

        case FsmState::HOVER:
            reference_.position                  = hold_position_;
            reference_.velocity                  = geometry_msgs::Vector3();
            reference_.acceleration              = geometry_msgs::Vector3();
            reference_.yaw                       = hold_yaw_;
            reference_.yaw_rate                  = 0.0;
            reference_.use_velocity_acceleration = false;
            output.publish_setpoint              = true;
            output.reference                     = reference_;
            if (input.events.land) {
                transition(FsmState::LANDING, "landing requested from hover", input);
                break;
            }
            if (input.planner_sequence != last_planner_sequence_) {
                last_planner_sequence_ = input.planner_sequence;
                if (plannerCommandValid(input, true)) {
                    ++planner_activation_count_;
                } else {
                    planner_activation_count_ = 0;
                }
                if (planner_activation_count_ >= config_.planner.activation_messages) {
                    reference_ = transformedPlannerReference(input);
                    transition(
                        FsmState::EXTERNAL, "continuous valid planner commands received", input);
                }
            }
            break;

        case FsmState::EXTERNAL:
            if (input.events.land) {
                transition(FsmState::LANDING, "landing requested from external control", input);
                break;
            }
            if (input.events.hover || input.events.abort || !plannerCommandValid(input, false)) {
                transition(
                    FsmState::BRAKE,
                    input.events.hover || input.events.abort
                        ? "hover requested from external control"
                        : "planner command invalid or timed out",
                    input);
                break;
            }
            reference_              = transformedPlannerReference(input);
            output.publish_setpoint = true;
            output.reference        = reference_;
            break;

        case FsmState::BRAKE:
            if (input.events.land) {
                transition(FsmState::LANDING, "landing requested while braking", input);
                break;
            }
            updateBrakeProfile(dt);
            output.publish_setpoint = true;
            output.reference        = reference_;
            if (norm3(input.velocity_world) <= config_.planner.brake_velocity_tolerance) {
                if (stable_since_ <= 0.0) {
                    stable_since_ = input.now;
                }
                if (input.now - stable_since_ >= config_.planner.brake_stable_time) {
                    transition(FsmState::HOVER, "vehicle stopped after planner loss", input);
                }
            } else {
                stable_since_ = 0.0;
            }
            break;

        case FsmState::LANDING:
            // LANDING is intentionally sticky. A transient gap in PX4's returned
            // LOCAL_POSITION_NED stream must not erase the landing request, because
            // touchdown confirmation and disarming do not require position data.
            // If PX4 has left OFFBOARD, keep pre-streaming the frozen reference so
            // the pilot can re-enter OFFBOARD, but never take the mode back
            // automatically. The descent profile advances only while PX4 is
            // actually accepting it.
            if (input.state_fresh && input.connected && input.odometry_fresh) {
                if (input.mode == "OFFBOARD") {
                    updateLandingProfile(input, dt);
                    landing_control_elapsed_ += dt;
                }
                output.publish_setpoint = true;
                output.reference        = reference_;
            }
            if (input.extended_state_fresh &&
                input.landed_state == mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND) {
                if (touchdown_since_ <= 0.0) {
                    touchdown_since_ = input.now;
                }
                if (input.now - touchdown_since_ >= config_.landing.touchdown_confirm_time) {
                    transition(
                        config_.landing.auto_disarm ? FsmState::DISARMING : FsmState::GROUND,
                        "touchdown confirmed", input);
                }
            } else {
                touchdown_since_ = 0.0;
            }
            if (state_ == FsmState::LANDING &&
                landing_control_elapsed_ > config_.timeouts.landing) {
                transition(FsmState::FAILSAFE, "landing timed out", input);
                output.publish_setpoint = false;
            }
            break;

        case FsmState::DISARMING:
            // Once touchdown has been confirmed, disarming no longer depends on
            // odometry or OFFBOARD mode. Keep a valid setpoint stream only when
            // odometry is available, and keep requesting disarm until PX4 reports
            // that the vehicle is no longer armed.
            if (input.state_fresh && input.connected && input.odometry_fresh) {
                output.publish_setpoint = true;
                output.reference        = reference_;
            }
            if (input.state_fresh && input.connected && !input.armed) {
                have_takeoff_origin_ = false;
                transition(FsmState::GROUND, "vehicle disarmed", input);
            } else if (
                input.state_fresh && input.connected &&
                input.now - state_entered_at_ >= config_.landing.disarm_delay) {
                output.request_disarm = true;
            }
            break;

        case FsmState::FAILSAFE:
            output.publish_setpoint =
                !config_.failsafe.stop_setpoints_on_critical_failure && input.odometry_fresh;
            output.reference = reference_;
            break;
    }

    // State transitions happen inside the cases above. Preserve an unbroken
    // setpoint stream on the exact transition cycle (for example EXTERNAL to
    // BRAKE or HOVER to LANDING); FAILSAFE deliberately remains excluded.
    if (activeState() && state_ != FsmState::LANDING && state_ != FsmState::DISARMING &&
        !output.publish_setpoint) {
        output.publish_setpoint = true;
        output.reference        = reference_;
    }

    notePublishing(input, output.publish_setpoint);
    return output;
}

}  // namespace px4ctrl
