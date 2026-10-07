#include "yopo_planner/yopo_planner.h"
#include <algorithm>
#include <cmath>
#include <limits>
#include <ros/console.h>

namespace yopo_planner {

namespace {

constexpr double kPi = 3.14159265358979323846;

Eigen::Matrix3d rotationFromYawPitchRoll(double yaw, double pitch, double roll) {
    return (Eigen::AngleAxisd(yaw, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

}  // namespace

void Poly5Solver::reset(
    double pos0, double vel0, double acc0, double pos1, double vel1, double acc1, double tf) {
    const double t = tf;
    Eigen::Matrix<double, 6, 6> inv;
    inv << 1, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0.5, 0, 0, 0, -10 / std::pow(t, 3),
        -6 / std::pow(t, 2), -3 / (2 * t), 10 / std::pow(t, 3), -4 / std::pow(t, 2), 1 / (2 * t),
        15 / std::pow(t, 4), 8 / std::pow(t, 3), 3 / (2 * std::pow(t, 2)), -15 / std::pow(t, 4),
        7 / std::pow(t, 3), -1 / std::pow(t, 2), -6 / std::pow(t, 5), -3 / std::pow(t, 4),
        -1 / (2 * std::pow(t, 3)), 6 / std::pow(t, 5), -3 / std::pow(t, 4),
        1 / (2 * std::pow(t, 3));
    Eigen::Matrix<double, 6, 1> state;
    state << pos0, vel0, acc0, pos1, vel1, acc1;
    coeff = inv * state;
}

double Poly5Solver::position(double t) const {
    return coeff[0] + coeff[1] * t + coeff[2] * t * t + coeff[3] * std::pow(t, 3) +
           coeff[4] * std::pow(t, 4) + coeff[5] * std::pow(t, 5);
}

double Poly5Solver::velocity(double t) const {
    return coeff[1] + 2 * coeff[2] * t + 3 * coeff[3] * t * t + 4 * coeff[4] * std::pow(t, 3) +
           5 * coeff[5] * std::pow(t, 4);
}

double Poly5Solver::acceleration(double t) const {
    return 2 * coeff[2] + 6 * coeff[3] * t + 12 * coeff[4] * t * t + 20 * coeff[5] * std::pow(t, 3);
}

double Poly5Solver::jerk(double t) const {
    return 6 * coeff[3] + 24 * coeff[4] * t + 60 * coeff[5] * t * t;
}

YopoPlanner::YopoPlanner(YopoParams params) : params_(params) {
    params_.traj_num    = params_.horizon_num * params_.vertical_num * params_.radio_num;
    params_.goal_length = 2.0 * params_.radio_range;
    const double ratio  = params_.velocity / params_.vel_max_train;
    vel_max_            = ratio * params_.vel_max_train;
    acc_max_            = ratio * ratio * params_.acc_max_train;
    segment_time_       = (2.0 * params_.radio_range / params_.vel_max_train) / ratio;
    yaw_diff_           = 0.5 * params_.horizon_anchor_fov / 180.0 * kPi;
    pitch_diff_         = 0.5 * params_.vertical_anchor_fov / 180.0 * kPi;
    rotation_bc_        = rotationFromYawPitchRoll(0.0, params_.pitch_angle_deg / 180.0 * kPi, 0.0);
    camera_extrinsic_ready_ = !params_.require_camera_extrinsic;
    buildLattice();
    last_all_endstates_.resize(params_.traj_num);
    last_scores_.assign(params_.traj_num, 0.0f);
}

void YopoPlanner::setCameraToBodyExtrinsic(const Eigen::Matrix3d& rotation_bc) {
    std::lock_guard<std::mutex> lock(mutex_);
    rotation_bc_            = rotation_bc;
    camera_extrinsic_ready_ = true;
}

bool YopoPlanner::goalAlignmentActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return goal_alignment_state_ != GoalAlignmentState::INACTIVE;
}

bool YopoPlanner::terminalApproachActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return terminal_approach_active_;
}

double YopoPlanner::controlTime() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ctrl_time_;
}

bool YopoPlanner::dynamicTargetLost() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dynamic_target_lost_;
}

bool YopoPlanner::dynamicLossHoldActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dynamic_loss_hold_active_;
}

bool YopoPlanner::dynamicStandoffHoldActive() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return dynamic_standoff_hold_active_;
}

void YopoPlanner::setGoal(const Eigen::Vector3d& goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    // Goal publishers commonly send the same PoseStamped more than once.  Do
    // not restart braking or turning for a duplicate goal.
    if ((goal - goal_).norm() < 1e-4) return;

    goal_                      = goal;
    goal_velocity_.setZero();
    dynamic_target_active_     = false;
    dynamic_target_lost_       = false;
    dynamic_loss_hold_active_  = false;
    dynamic_standoff_hold_active_ = false;
    dynamic_standoff_release_distance_ = 0.0;
    dynamic_target_tracking_scale_ = 1.0;
    dynamic_target_yaw_rate_cmd_ = 0.0;
    arrive_                    = false;
    terminal_approach_active_  = false;
    terminal_stable_elapsed_   = 0.0;
    terminal_last_odom_stamp_  = ros::Time();
    arrival_hover_initialized_ = false;
    has_trajectory_            = false;
    has_last_control_msg_      = false;
    ctrl_time_                 = 0.0;
    desire_init_               = false;
    desire_acc_.setZero();
    goal_alignment_state_      = GoalAlignmentState::INACTIVE;
    goal_alignment_pending_    = params_.goal_yaw_alignment_enabled;
    goal_alignment_stable_elapsed_ = 0.0;
    if (odom_init_) {
        syncReferenceFromOdomLocked();
        startGoalAlignmentIfNeededLocked();
    }
}

bool YopoPlanner::updateDynamicTarget(
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    if (!position.allFinite() || !velocity.allFinite()) return false;

    std::lock_guard<std::mutex> lock(mutex_);
    dynamic_target_position_ = position;
    goal_velocity_           = velocity;
    dynamic_target_active_   = true;
    dynamic_target_lost_     = false;
    arrive_                  = false;
    arrival_hover_initialized_ = false;
    terminal_stable_elapsed_ = 0.0;
    terminal_last_odom_stamp_ = ros::Time();

    // A moving target is a rolling update inside the same flight task.  It
    // must not restart the trajectory clock, discard the current reference,
    // or re-enter the per-waypoint yaw alignment state machine.
    goal_alignment_pending_        = false;
    goal_alignment_state_          = GoalAlignmentState::INACTIVE;
    goal_alignment_stable_elapsed_ = 0.0;

    if (odom_init_) {
        const Eigen::Vector3d odom_position(
            odom_.pose.pose.position.x, odom_.pose.pose.position.y,
            odom_.pose.pose.position.z);
        const Eigen::Vector3d odom_velocity(
            odom_.twist.twist.linear.x, odom_.twist.twist.linear.y,
            odom_.twist.twist.linear.z);
        const Eigen::Vector3d target_delta = position - odom_position;
        const double target_distance = target_delta.norm();
        Eigen::Vector3d target_direction = Eigen::Vector3d::Zero();
        if (target_distance > 1e-6) {
            target_direction = target_delta / target_distance;
        }
        const double standoff = std::max(0.0, params_.dynamic_target_standoff_distance);
        goal_ = position - target_direction * std::min(standoff, target_distance);

        const double speed = odom_velocity.norm();
        const double stop_duration = std::max(
            0.8, std::min(
                     2.0, 2.0 * speed / std::max(params_.terminal_brake_accel, 0.1)));
        const double braking_allowance = 0.5 * stop_duration * speed;
        const double enter_distance = standoff + braking_allowance;
        const bool enter_standoff =
            standoff > 0.0 && !dynamic_standoff_hold_active_ &&
            target_distance <= enter_distance;
        const bool remain_in_standoff =
            dynamic_standoff_hold_active_ &&
            target_distance <= dynamic_standoff_release_distance_;

        if (enter_standoff || remain_in_standoff) {
            if (!dynamic_standoff_hold_active_) {
                // Latch the release boundary at entry.  Recomputing it from
                // the decreasing braking allowance would immediately release
                // the hold while the vehicle is still decelerating.
                dynamic_standoff_release_distance_ =
                    enter_distance + params_.dynamic_target_standoff_hysteresis;
                startDynamicStandoffHoldLocked(odom_position, odom_velocity);
                ROS_WARN(
                    "YOPO dynamic target entered standoff zone: range=%.2f m, "
                    "desired=%.2f m; braking to hold.",
                    target_distance, standoff);
            }
            const double camera_forward_yaw_body =
                std::atan2(rotation_bc_(1, 0), rotation_bc_(0, 0));
            if (target_delta.head<2>().norm() > 0.1) {
                terminal_yaw_ = wrapToPi(
                    std::atan2(target_delta.y(), target_delta.x()) -
                    camera_forward_yaw_body);
            }
        } else if (dynamic_standoff_hold_active_) {
            dynamic_standoff_hold_active_ = false;
            dynamic_standoff_release_distance_ = 0.0;
            terminal_approach_active_ = false;
            has_trajectory_ = false;
            has_last_control_msg_ = false;
            ctrl_time_ = 0.0;
            syncReferenceFromOdomLocked();
            ROS_WARN(
                "YOPO dynamic target left standoff zone: range=%.2f m; resuming planning.",
                target_distance);
        }
    } else {
        goal_ = position;
    }
    return true;
}

void YopoPlanner::setDynamicTargetTrackingScale(double scale) {
    std::lock_guard<std::mutex> lock(mutex_);
    dynamic_target_tracking_scale_ = std::max(
        params_.dynamic_target_min_speed_scale, std::min(1.0, scale));
}

bool YopoPlanner::handleDynamicTargetLost() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!dynamic_target_active_ || dynamic_target_lost_ || !odom_init_) return false;

    dynamic_target_lost_      = true;
    dynamic_loss_hold_active_ = true;
    dynamic_standoff_hold_active_ = false;
    dynamic_standoff_release_distance_ = 0.0;
    arrive_                   = false;
    goal_alignment_pending_   = false;
    goal_alignment_state_     = GoalAlignmentState::INACTIVE;

    const Eigen::Vector3d odom_pos(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y,
        odom_.pose.pose.position.z);
    const Eigen::Vector3d odom_vel(
        odom_.twist.twist.linear.x, odom_.twist.twist.linear.y,
        odom_.twist.twist.linear.z);
    const bool reference_is_close =
        desire_init_ && (desire_pos_ - odom_pos).norm() < 0.75;
    const Eigen::Vector3d start_pos = reference_is_close ? desire_pos_ : odom_pos;
    const Eigen::Vector3d start_vel = reference_is_close ? desire_vel_ : odom_vel;
    const Eigen::Vector3d start_acc = reference_is_close ? desire_acc_ : Eigen::Vector3d::Zero();

    const double speed = start_vel.norm();
    const double duration = std::max(
        0.8, std::min(2.0, 2.0 * speed / std::max(params_.terminal_brake_accel, 0.1)));
    const Eigen::Vector3d stop_pos = start_pos + 0.5 * duration * start_vel;
    poly_x_.reset(
        start_pos.x(), start_vel.x(), start_acc.x(), stop_pos.x(), 0.0, 0.0, duration);
    poly_y_.reset(
        start_pos.y(), start_vel.y(), start_acc.y(), stop_pos.y(), 0.0, 0.0, duration);
    poly_z_.reset(
        start_pos.z(), start_vel.z(), start_acc.z(), stop_pos.z(), 0.0, 0.0, duration);
    terminal_time_             = duration;
    const Eigen::Vector3d last_target_dir = dynamic_target_position_ - odom_pos;
    const double camera_forward_yaw_body =
        std::atan2(rotation_bc_(1, 0), rotation_bc_(0, 0));
    terminal_yaw_ = last_target_dir.head<2>().norm() > 0.1
                        ? wrapToPi(
                              std::atan2(last_target_dir.y(), last_target_dir.x()) -
                              camera_forward_yaw_body)
                        : last_yaw_;
    dynamic_loss_search_center_yaw_ = terminal_yaw_;
    dynamic_loss_search_direction_ = 1;
    terminal_approach_active_  = true;
    ctrl_time_                 = 0.0;
    has_trajectory_            = true;
    has_last_control_msg_      = false;

    ROS_WARN(
        "YOPO dynamic target lost: one-shot smooth stop from %.2f m/s over %.2f s; "
        "holding at (%.2f %.2f %.2f).",
        speed, duration, stop_pos.x(), stop_pos.y(), stop_pos.z());
    return true;
}

void YopoPlanner::updateOdometry(const nav_msgs::Odometry& odom) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_ = odom;

    if (!desire_init_) {
        syncReferenceFromOdomLocked();
    }
    odom_init_ = true;

    if (goal_alignment_pending_) {
        startGoalAlignmentIfNeededLocked();
    }

    const Eigen::Vector3d pos(
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    if (goal_alignment_state_ != GoalAlignmentState::INACTIVE || arrive_ ||
        dynamic_target_lost_) return;
    if (dynamic_target_active_) return;

    const Eigen::Vector3d velocity(
        odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    const double distance = (pos - goal_).norm();
    if (params_.terminal_approach_enabled) {
        if (!terminal_approach_active_) {
            // A zero-end-acceleration quintic needs more distance than a
            // bang-bang stop. Use 0.75 v^2/a, including lateral/vertical speed.
            const double braking_distance =
                0.75 * velocity.squaredNorm() / params_.terminal_brake_accel;
            if (distance <= std::max(params_.arrive_distance,
                                     braking_distance + params_.terminal_brake_margin)) {
                startTerminalApproachLocked(pos, velocity);
            }
        }
        double odom_dt = params_.ctrl_dt;
        if (!odom.header.stamp.isZero() && !terminal_last_odom_stamp_.isZero()) {
            odom_dt = (odom.header.stamp - terminal_last_odom_stamp_).toSec();
        }
        terminal_last_odom_stamp_ = odom.header.stamp;
        if (terminal_approach_active_ && ctrl_time_ >= terminal_time_ &&
            distance <= params_.terminal_position_tolerance &&
            velocity.norm() <= params_.terminal_speed_tolerance) {
            if (odom_dt > 0.0 && odom_dt <= 0.1) terminal_stable_elapsed_ += odom_dt;
        } else {
            terminal_stable_elapsed_ = 0.0;
        }
        if (terminal_stable_elapsed_ >= params_.terminal_stable_time) {
            arrive_ = true;
            terminal_approach_active_ = false;
            initializeArrivalHoldLocked(goal_, terminal_yaw_);
            ROS_INFO("YOPO reached goal at low speed; holding goal (%.3f %.3f %.3f).",
                     goal_.x(), goal_.y(), goal_.z());
        }
    } else if (distance < params_.arrive_distance) {
        // Preserve the existing real-flight behavior while the straight-line
        // terminal approach is evaluated in SITL.
        arrive_ = true;
        initializeArrivalHoldLocked(pos, currentYawLocked());
    }
}

bool YopoPlanner::syncReferenceFromCurrentOdom() {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!odom_init_) return false;

    syncReferenceFromOdomLocked();
    desire_init_          = true;
    has_trajectory_       = false;
    terminal_approach_active_ = false;
    terminal_stable_elapsed_ = 0.0;
    terminal_last_odom_stamp_ = ros::Time();
    has_last_control_msg_ = false;
    ctrl_time_            = 0.0;
    dynamic_target_lost_  = false;
    dynamic_loss_hold_active_ = false;
    dynamic_standoff_hold_active_ = false;
    dynamic_standoff_release_distance_ = 0.0;
    dynamic_target_tracking_scale_ = 1.0;
    dynamic_target_yaw_rate_cmd_ = 0.0;

    if (arrive_) {
        initializeArrivalHoldLocked(
            params_.terminal_approach_enabled ? goal_ : desire_pos_, last_yaw_);
    }
    return true;
}

std::array<float, 1 * 9 * 3 * 5> YopoPlanner::prepareObsInput() {
    std::lock_guard<std::mutex> lock(mutex_);
    std::array<float, 1 * 9 * 3 * 5> obs_input{};

    if (params_.require_camera_extrinsic && !camera_extrinsic_ready_) {
        return obs_input;
    }

    const Eigen::Quaterniond q(
        odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
        odom_.pose.pose.orientation.z);
    const Eigen::Matrix3d rotation_wb = q.normalized().toRotationMatrix();
    rotation_wc_                      = rotation_wb * rotation_bc_;
    const Eigen::Matrix3d rotation_cw = rotation_wc_.transpose();

    const Eigen::Vector3d start_pos_w = currentStartPos();
    const Eigen::Vector3d vel_w       = params_.plan_from_reference
                                            ? desire_vel_
                                            : Eigen::Vector3d(
                                            odom_.twist.twist.linear.x, odom_.twist.twist.linear.y,
                                            odom_.twist.twist.linear.z);
    const Eigen::Vector3d vel_c       = rotation_cw * vel_w;
    const Eigen::Vector3d acc_c       = rotation_cw * desire_acc_;
    const Eigen::Vector3d goal_c      = rotation_cw * (goal_ - start_pos_w);

    Eigen::Matrix<double, 3, 3> obs;
    obs.row(0)                = (vel_c / vel_max_).transpose();
    obs.row(1)                = (acc_c / acc_max_).transpose();
    Eigen::Vector3d goal_norm = goal_c / std::max(goal_c.norm(), params_.goal_length);
    obs.row(2)                = goal_norm.transpose();

    for (int grid_id = 0; grid_id < params_.traj_num; ++grid_id) {
        const int lattice_id                          = params_.traj_num - 1 - grid_id;
        const Eigen::Matrix<double, 3, 3> transformed = obs * lattice_rbp_[lattice_id];
        for (int c = 0; c < 9; ++c) {
            const int r                               = c / 3;
            const int col                             = c % 3;
            obs_input[c * params_.traj_num + grid_id] = static_cast<float>(transformed(r, col));
        }
    }
    return obs_input;
}

void YopoPlanner::updateTrajectory(
    const std::array<float, 1 * 9 * 3 * 5>& endstate_pred,
    const std::array<float, 1 * 3 * 5>& score_pred, bool return_all_preds) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (params_.require_camera_extrinsic && !camera_extrinsic_ready_) return;
    const bool replacing_dynamic_loss_hold =
        dynamic_loss_hold_active_ && !dynamic_target_lost_;
    if (goal_alignment_state_ != GoalAlignmentState::INACTIVE ||
        (terminal_approach_active_ && !replacing_dynamic_loss_hold) || arrive_ ||
        dynamic_target_lost_) return;

    const Eigen::Vector3d start_pos  = currentStartPos();
    const Eigen::Vector3d start_vel  = currentStartVel();
    const Eigen::Vector3d goal_dir_w = goal_ - start_pos;

    int action_id    = 0;
    float best_score = std::numeric_limits<float>::max();
    for (int i = 0; i < params_.traj_num; ++i) {
        last_scores_[i] = score_pred[i];
        if (score_pred[i] < best_score) {
            best_score = score_pred[i];
            action_id  = i;
        }
    }

    if (return_all_preds) {
        for (int i = 0; i < params_.traj_num; ++i) {
            last_all_endstates_[i] = predToEndstate(endstate_pred, i);
        }
    } else {
        last_all_endstates_[0] = predToEndstate(endstate_pred, action_id);
        last_scores_[0]        = best_score;
        action_id              = 0;
    }

    const EndState& best = return_all_preds ? last_all_endstates_[action_id]
                                            : last_all_endstates_[0];
    const Eigen::Matrix<double, 3, 3> endstate_c =
        (Eigen::Matrix<double, 3, 3>() << best.pos.x(), best.vel.x(), best.acc.x(), best.pos.y(),
         best.vel.y(), best.acc.y(), best.pos.z(), best.vel.z(), best.acc.z())
            .finished();
    const Eigen::Matrix<double, 3, 3> endstate_w = rotation_wc_ * endstate_c;
    const double tracking_scale = dynamic_target_active_
                                      ? dynamic_target_tracking_scale_
                                      : 1.0;
    const Eigen::Vector3d end_delta_w = endstate_w.col(0) * tracking_scale;
    const Eigen::Vector3d end_vel_w = endstate_w.col(1) * tracking_scale;
    const Eigen::Vector3d end_acc_w =
        endstate_w.col(2) * tracking_scale * tracking_scale;
    const Eigen::Vector3d end_pos_w              = start_pos + end_delta_w;
    ROS_WARN_THROTTLE(
        3.0,
        "YOPO best end pos: start=(%.3f %.3f %.3f), local_yopo_rel=(%.3f %.3f %.3f), "
        "world_delta=(%.3f %.3f %.3f), world_abs=(%.3f %.3f %.3f), goal_dir=(%.3f %.3f %.3f), "
        "score=%.3f",
        start_pos.x(), start_pos.y(), start_pos.z(), best.pos.x(), best.pos.y(), best.pos.z(),
        end_delta_w.x(), end_delta_w.y(), end_delta_w.z(), end_pos_w.x(), end_pos_w.y(),
        end_pos_w.z(), goal_dir_w.x(), goal_dir_w.y(), goal_dir_w.z(), best_score);
    poly_x_.reset(
        start_pos.x(), start_vel.x(), desire_acc_.x(), end_delta_w.x() + start_pos.x(),
        end_vel_w.x(), end_acc_w.x(), segment_time_);
    poly_y_.reset(
        start_pos.y(), start_vel.y(), desire_acc_.y(), end_delta_w.y() + start_pos.y(),
        end_vel_w.y(), end_acc_w.y(), segment_time_);
    poly_z_.reset(
        start_pos.z(), start_vel.z(), desire_acc_.z(), end_delta_w.z() + start_pos.z(),
        end_vel_w.z(), end_acc_w.z(), segment_time_);

    ctrl_time_      = 0.0;
    has_trajectory_ = true;
    if (replacing_dynamic_loss_hold) {
        terminal_approach_active_ = false;
        dynamic_loss_hold_active_ = false;
    }
}

bool YopoPlanner::fillControlCommand(quadrotor_msgs::PositionCommand* cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!cmd) return false;

    if (goal_alignment_state_ != GoalAlignmentState::INACTIVE) {
        return fillGoalAlignmentCommandLocked(cmd);
    }

    if (arrive_) {
        if (!arrival_hover_initialized_ && odom_init_) {
            const Eigen::Quaterniond q(
                odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x,
                odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z);
            const Eigen::Matrix3d rotation = q.toRotationMatrix();
            initializeArrivalHoldLocked(
                Eigen::Vector3d(
                    odom_.pose.pose.position.x, odom_.pose.pose.position.y,
                    odom_.pose.pose.position.z),
                std::atan2(rotation(1, 0), rotation(0, 0)));
        }

        const auto yaw = calculateTargetYaw(
            arrival_target_yaw_, arrival_hover_yaw_, params_.ctrl_dt,
            params_.arrival_yaw_rate);
        arrival_hover_yaw_ = yaw.first;

        cmd->header.stamp    = ros::Time::now();
        cmd->trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        cmd->position.x      = arrival_hover_pos_.x();
        cmd->position.y      = arrival_hover_pos_.y();
        cmd->position.z      = arrival_hover_pos_.z();
        cmd->velocity.x      = 0.0;
        cmd->velocity.y      = 0.0;
        cmd->velocity.z      = 0.0;
        cmd->acceleration.x  = 0.0;
        cmd->acceleration.y  = 0.0;
        cmd->acceleration.z  = 0.0;
        cmd->jerk.x          = 0.0;
        cmd->jerk.y          = 0.0;
        cmd->jerk.z          = 0.0;
        cmd->yaw             = yaw.first;
        cmd->yaw_dot         = yaw.second;

        desire_pos_ = arrival_hover_pos_;
        desire_vel_.setZero();
        desire_acc_.setZero();
        last_yaw_            = arrival_hover_yaw_;
        desire_init_         = true;
        last_control_msg_     = *cmd;
        has_last_control_msg_ = true;
        return true;
    }

    if (params_.require_camera_extrinsic && !camera_extrinsic_ready_) return false;
    if (!has_trajectory_ || (!terminal_approach_active_ && ctrl_time_ > segment_time_))
        return false;

    ctrl_time_ = terminal_approach_active_
                     ? std::min(ctrl_time_ + params_.ctrl_dt, terminal_time_)
                     : ctrl_time_ + params_.ctrl_dt;
    cmd->header.stamp    = ros::Time::now();
    cmd->trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd->position.x      = poly_x_.position(ctrl_time_);
    cmd->position.y      = poly_y_.position(ctrl_time_);
    cmd->position.z      = poly_z_.position(ctrl_time_);
    cmd->velocity.x      = poly_x_.velocity(ctrl_time_);
    cmd->velocity.y      = poly_y_.velocity(ctrl_time_);
    cmd->velocity.z      = poly_z_.velocity(ctrl_time_);
    cmd->acceleration.x  = poly_x_.acceleration(ctrl_time_);
    cmd->acceleration.y  = poly_y_.acceleration(ctrl_time_);
    cmd->acceleration.z  = poly_z_.acceleration(ctrl_time_);
    cmd->jerk.x          = poly_x_.jerk(ctrl_time_);
    cmd->jerk.y          = poly_y_.jerk(ctrl_time_);
    cmd->jerk.z          = poly_z_.jerk(ctrl_time_);

    ROS_WARN_THROTTLE(
        1.0,
        "YOPO position_cmd z: t=%.3f/%.3f, cmd_z=%.3f, cmd_vz=%.3f, cmd_az=%.3f, "
        "odom_z=%.3f, goal_z=%.3f",
        ctrl_time_, terminal_approach_active_ ? terminal_time_ : segment_time_,
        cmd->position.z, cmd->velocity.z, cmd->acceleration.z,
        odom_.pose.pose.position.z, goal_.z());

    desire_pos_    = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    desire_vel_    = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    desire_acc_    = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    std::pair<double, double> yaw;
    if (terminal_approach_active_) {
        double yaw_target = terminal_yaw_;
        double yaw_rate_limit = params_.arrival_yaw_rate;
        if (dynamic_loss_hold_active_) {
            yaw_rate_limit = params_.dynamic_target_yaw_rate;
            if (params_.dynamic_target_search_enabled &&
                ctrl_time_ >= terminal_time_ - 1e-6) {
                const double amplitude =
                    params_.dynamic_target_search_yaw_amplitude_deg / 180.0 * kPi;
                yaw_target = wrapToPi(
                    dynamic_loss_search_center_yaw_ +
                    dynamic_loss_search_direction_ * amplitude);
                if (std::abs(wrapToPi(yaw_target - last_yaw_)) < 2.0 / 180.0 * kPi) {
                    dynamic_loss_search_direction_ *= -1;
                    yaw_target = wrapToPi(
                        dynamic_loss_search_center_yaw_ +
                        dynamic_loss_search_direction_ * amplitude);
                }
                yaw_rate_limit = params_.dynamic_target_search_yaw_rate;
            }
            yaw = calculateTargetYawLimited(
                yaw_target, last_yaw_, dynamic_target_yaw_rate_cmd_, params_.ctrl_dt,
                yaw_rate_limit, params_.dynamic_target_yaw_accel,
                params_.dynamic_target_yaw_deadband_deg / 180.0 * kPi);
            dynamic_target_yaw_rate_cmd_ = yaw.second;
        } else {
            yaw = calculateTargetYaw(
                yaw_target, last_yaw_, params_.ctrl_dt, yaw_rate_limit);
            dynamic_target_yaw_rate_cmd_ = 0.0;
        }
    } else if (dynamic_target_active_ && !dynamic_target_lost_) {
        const Eigen::Vector3d actual_position(
            odom_.pose.pose.position.x, odom_.pose.pose.position.y,
            odom_.pose.pose.position.z);
        const Eigen::Vector3d target_direction =
            dynamic_target_position_ - actual_position;
        const double camera_forward_yaw_body =
            std::atan2(rotation_bc_(1, 0), rotation_bc_(0, 0));
        const double target_yaw = target_direction.head<2>().norm() > 0.1
                                      ? wrapToPi(
                                            std::atan2(
                                                target_direction.y(), target_direction.x()) -
                                            camera_forward_yaw_body)
                                      : last_yaw_;
        yaw = calculateTargetYawLimited(
            target_yaw, last_yaw_, dynamic_target_yaw_rate_cmd_, params_.ctrl_dt,
            params_.dynamic_target_yaw_rate, params_.dynamic_target_yaw_accel,
            params_.dynamic_target_yaw_deadband_deg / 180.0 * kPi);
        dynamic_target_yaw_rate_cmd_ = yaw.second;
    } else {
        yaw = calculateYaw(
            desire_vel_, goal_ - desire_pos_, last_yaw_, params_.ctrl_dt);
        dynamic_target_yaw_rate_cmd_ = 0.0;
    }
    last_yaw_      = yaw.first;
    cmd->yaw       = yaw.first;
    cmd->yaw_dot   = yaw.second;
    desire_init_   = true;
    last_control_msg_     = *cmd;
    has_last_control_msg_ = true;
    return true;
}

quadrotor_msgs::PositionCommand YopoPlanner::emptyControlCommand() const {
    quadrotor_msgs::PositionCommand cmd;
    cmd.header.stamp    = ros::Time::now();
    cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
    return cmd;
}

std::vector<Eigen::Vector3f> YopoPlanner::bestTrajectoryPoints(int samples) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Eigen::Vector3f> points;
    if (!has_trajectory_ || samples <= 0) return points;
    points.reserve(samples);
    for (int i = 0; i < samples; ++i) {
        const double duration = terminal_approach_active_ ? terminal_time_ : segment_time_;
        const double t = duration * static_cast<double>(i) / static_cast<double>(samples);
        points.emplace_back(poly_x_.position(t), poly_y_.position(t), poly_z_.position(t));
    }
    return points;
}

std::vector<Eigen::Vector3f> YopoPlanner::latticeTrajectoryPoints(int samples) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Eigen::Vector3f> points;
    if (!odom_init_ || samples <= 0) return points;
    points.reserve(params_.traj_num * samples);
    const Eigen::Vector3d start_pos = currentStartPos();
    const Eigen::Vector3d start_vel = currentStartVel();
    for (const auto& pos_c : lattice_pos_) {
        const Eigen::Vector3d pos_w = rotation_wc_ * pos_c;
        Poly5Solver px;
        Poly5Solver py;
        Poly5Solver pz;
        px.reset(
            start_pos.x(), start_vel.x(), desire_acc_.x(), start_pos.x() + pos_w.x(), 0.0, 0.0,
            segment_time_);
        py.reset(
            start_pos.y(), start_vel.y(), desire_acc_.y(), start_pos.y() + pos_w.y(), 0.0, 0.0,
            segment_time_);
        pz.reset(
            start_pos.z(), start_vel.z(), desire_acc_.z(), start_pos.z() + pos_w.z(), 0.0, 0.0,
            segment_time_);
        for (int i = 0; i < samples; ++i) {
            const double t = segment_time_ * static_cast<double>(i) / static_cast<double>(samples);
            points.emplace_back(px.position(t), py.position(t), pz.position(t));
        }
    }
    return points;
}

std::vector<Eigen::Vector4f> YopoPlanner::allTrajectoryPoints(int samples) const {
    std::lock_guard<std::mutex> lock(mutex_);
    std::vector<Eigen::Vector4f> points;
    if (!has_trajectory_ || samples <= 0) return points;
    points.reserve(params_.traj_num * samples);
    const Eigen::Vector3d start_pos = currentStartPos();
    const Eigen::Vector3d start_vel = currentStartVel();
    for (int i = 0; i < params_.traj_num; ++i) {
        const EndState& state = last_all_endstates_[i];
        const Eigen::Matrix<double, 3, 3> endstate_c =
            (Eigen::Matrix<double, 3, 3>() << state.pos.x(), state.vel.x(), state.acc.x(),
             state.pos.y(), state.vel.y(), state.acc.y(), state.pos.z(), state.vel.z(),
             state.acc.z())
                .finished();
        const Eigen::Matrix<double, 3, 3> endstate_w = rotation_wc_ * endstate_c;
        Poly5Solver px;
        Poly5Solver py;
        Poly5Solver pz;
        px.reset(
            start_pos.x(), start_vel.x(), desire_acc_.x(), start_pos.x() + endstate_w(0, 0),
            endstate_w(0, 1), endstate_w(0, 2), segment_time_);
        py.reset(
            start_pos.y(), start_vel.y(), desire_acc_.y(), start_pos.y() + endstate_w(1, 0),
            endstate_w(1, 1), endstate_w(1, 2), segment_time_);
        pz.reset(
            start_pos.z(), start_vel.z(), desire_acc_.z(), start_pos.z() + endstate_w(2, 0),
            endstate_w(2, 1), endstate_w(2, 2), segment_time_);
        for (int j = 0; j < samples; ++j) {
            const double t = segment_time_ * static_cast<double>(j) / static_cast<double>(samples);
            points.emplace_back(px.position(t), py.position(t), pz.position(t), last_scores_[i]);
        }
    }
    return points;
}

void YopoPlanner::buildLattice() {
    lattice_pos_.clear();
    lattice_angle_.clear();
    lattice_rbp_.clear();

    const double direction_diff =
        params_.horizon_num == 1 ? 0.0
                                 : (params_.horizon_camera_fov / 180.0 * kPi) / params_.horizon_num;
    const double altitude_diff =
        params_.vertical_num == 1
            ? 0.0
            : (params_.vertical_camera_fov / 180.0 * kPi) / params_.vertical_num;
    const double radio_diff = params_.radio_range / params_.radio_num;

    for (int h = 0; h < params_.radio_num; ++h) {
        for (int i = 0; i < params_.vertical_num; ++i) {
            for (int j = 0; j < params_.horizon_num; ++j) {
                const double search_radio = (h + 1) * radio_diff;
                const double alpha =
                    -direction_diff * (params_.horizon_num - 1) / 2.0 + j * direction_diff;
                const double beta =
                    -altitude_diff * (params_.vertical_num - 1) / 2.0 + i * altitude_diff;
                lattice_pos_.emplace_back(
                    std::cos(beta) * std::cos(alpha) * search_radio,
                    std::cos(beta) * std::sin(alpha) * search_radio, std::sin(beta) * search_radio);
                lattice_angle_.emplace_back(alpha, beta);
                lattice_rbp_.push_back(rotationFromYawPitchRoll(alpha, -beta, 0.0));
            }
        }
    }
}

void YopoPlanner::syncReferenceFromOdomLocked() {
    desire_pos_ = Eigen::Vector3d(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
    desire_vel_ = Eigen::Vector3d(
        odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, odom_.twist.twist.linear.z);
    desire_acc_.setZero();
    const Eigen::Quaterniond q(
        odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x, odom_.pose.pose.orientation.y,
        odom_.pose.pose.orientation.z);
    const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
    last_yaw_                      = std::atan2(rotation(1, 0), rotation(0, 0));
    dynamic_target_yaw_rate_cmd_   = 0.0;
}

void YopoPlanner::startTerminalApproachLocked(
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    // Use the last published reference when it is tracking measured odometry;
    // this avoids a setpoint jump at the transition to the terminal segment.
    const bool reference_tracks_odom =
        has_last_control_msg_ && (desire_pos_ - position).norm() < 0.5;
    const Eigen::Vector3d start_pos = reference_tracks_odom ? desire_pos_ : position;
    const Eigen::Vector3d start_vel = reference_tracks_odom ? desire_vel_ : velocity;
    const Eigen::Vector3d start_acc = reference_tracks_odom ? desire_acc_ : Eigen::Vector3d::Zero();
    const double distance = (goal_ - start_pos).norm();
    const double speed = start_vel.norm();
    const double base_time = std::max(0.8, 2.0 * distance / std::max(speed, 0.5));
    double chosen_time = base_time;
    double best_violation = std::numeric_limits<double>::infinity();
    Poly5Solver chosen_x, chosen_y, chosen_z;

    // Find a monotone, acceleration-limited quintic when one is feasible.
    // Simply increasing duration can make a fast initial velocity overshoot,
    // so check the entire segment rather than only its endpoints.
    for (int attempt = 0; attempt < 31; ++attempt) {
        const double duration = base_time * (0.65 + 0.035 * attempt);
        Poly5Solver px, py, pz;
        px.reset(start_pos.x(), start_vel.x(), start_acc.x(), goal_.x(), 0.0, 0.0, duration);
        py.reset(start_pos.y(), start_vel.y(), start_acc.y(), goal_.y(), 0.0, 0.0, duration);
        pz.reset(start_pos.z(), start_vel.z(), start_acc.z(), goal_.z(), 0.0, 0.0, duration);
        double max_accel = 0.0;
        double max_overshoot = 0.0;
        for (int i = 0; i <= 60; ++i) {
            const double t = duration * i / 60.0;
            const Eigen::Vector3d p(px.position(t), py.position(t), pz.position(t));
            const Eigen::Vector3d a(px.acceleration(t), py.acceleration(t), pz.acceleration(t));
            max_accel = std::max(max_accel, a.norm());
            if (distance > 1e-3) {
                const double progress = (p - start_pos).dot(goal_ - start_pos) / distance;
                max_overshoot = std::max(max_overshoot, std::max(-progress, progress - distance));
            }
        }
        const double violation =
            std::max(0.0, max_accel / params_.terminal_brake_accel - 1.0) +
            10.0 * max_overshoot;
        if (violation < best_violation) {
            best_violation = violation;
            chosen_time = duration;
            chosen_x = px;
            chosen_y = py;
            chosen_z = pz;
        }
        if (violation < 1e-6) break;
    }
    poly_x_ = chosen_x;
    poly_y_ = chosen_y;
    poly_z_ = chosen_z;
    terminal_time_ = chosen_time;
    terminal_yaw_ = last_yaw_;
    terminal_approach_active_ = true;
    terminal_stable_elapsed_ = 0.0;
    ctrl_time_ = 0.0;
    has_trajectory_ = true;
    ROS_WARN(
        "YOPO terminal approach: start=(%.2f %.2f %.2f), goal=(%.2f %.2f %.2f), "
        "speed=%.2f m/s, duration=%.2f s, feasibility_violation=%.3f.",
        start_pos.x(), start_pos.y(), start_pos.z(), goal_.x(), goal_.y(), goal_.z(),
        speed, terminal_time_, best_violation);
}

void YopoPlanner::startDynamicStandoffHoldLocked(
    const Eigen::Vector3d& position, const Eigen::Vector3d& velocity) {
    const bool reference_is_close =
        desire_init_ && (desire_pos_ - position).norm() < 0.75;
    const Eigen::Vector3d start_pos = reference_is_close ? desire_pos_ : position;
    const Eigen::Vector3d start_vel = reference_is_close ? desire_vel_ : velocity;
    const Eigen::Vector3d start_acc =
        reference_is_close ? desire_acc_ : Eigen::Vector3d::Zero();
    const double speed = start_vel.norm();
    const double duration = std::max(
        0.8, std::min(
                 2.0, 2.0 * speed / std::max(params_.terminal_brake_accel, 0.1)));
    const Eigen::Vector3d stop_pos = start_pos + 0.5 * duration * start_vel;

    poly_x_.reset(
        start_pos.x(), start_vel.x(), start_acc.x(), stop_pos.x(), 0.0, 0.0, duration);
    poly_y_.reset(
        start_pos.y(), start_vel.y(), start_acc.y(), stop_pos.y(), 0.0, 0.0, duration);
    poly_z_.reset(
        start_pos.z(), start_vel.z(), start_acc.z(), stop_pos.z(), 0.0, 0.0, duration);
    terminal_time_ = duration;
    terminal_approach_active_ = true;
    dynamic_standoff_hold_active_ = true;
    dynamic_loss_hold_active_ = false;
    ctrl_time_ = 0.0;
    has_trajectory_ = true;
    has_last_control_msg_ = false;
}

void YopoPlanner::initializeArrivalHoldLocked(const Eigen::Vector3d& position, double yaw) {
    arrival_hover_pos_ = position;
    arrival_hover_yaw_ = wrapToPi(yaw);

    const Eigen::Vector3d goal_delta = goal_ - arrival_hover_pos_;
    const double horizontal_distance = std::hypot(goal_delta.x(), goal_delta.y());
    arrival_target_yaw_ = horizontal_distance >= params_.arrival_yaw_min_distance
                              ? std::atan2(goal_delta.y(), goal_delta.x())
                              : arrival_hover_yaw_;
    arrival_hover_initialized_ = true;
}

double YopoPlanner::currentYawLocked() const {
    const Eigen::Quaterniond q(
        odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x,
        odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z);
    const Eigen::Matrix3d rotation = q.normalized().toRotationMatrix();
    return std::atan2(rotation(1, 0), rotation(0, 0));
}

void YopoPlanner::startGoalAlignmentIfNeededLocked() {
    goal_alignment_pending_ = false;
    goal_alignment_state_   = GoalAlignmentState::INACTIVE;
    if (!params_.goal_yaw_alignment_enabled || !odom_init_) return;

    const Eigen::Vector3d position(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y,
        odom_.pose.pose.position.z);
    const Eigen::Vector3d goal_delta = goal_ - position;
    const double horizontal_distance = std::hypot(goal_delta.x(), goal_delta.y());
    if (horizontal_distance < params_.goal_yaw_align_min_distance) return;

    const double current_yaw = currentYawLocked();
    const double target_yaw  = std::atan2(goal_delta.y(), goal_delta.x());
    const double yaw_error   = wrapToPi(target_yaw - current_yaw);
    const double enter_threshold = params_.goal_yaw_align_enter_deg / 180.0 * kPi;
    if (std::abs(yaw_error) <= enter_threshold) return;

    goal_alignment_state_          = GoalAlignmentState::BRAKING;
    goal_alignment_hold_pos_       = position;
    goal_alignment_command_yaw_    = current_yaw;
    goal_alignment_target_yaw_     = target_yaw;
    goal_alignment_stable_elapsed_ = 0.0;
    has_trajectory_                = false;
    has_last_control_msg_          = false;
    ctrl_time_                     = 0.0;
    desire_pos_                    = position;
    desire_vel_.setZero();
    desire_acc_.setZero();
    desire_init_ = true;

    ROS_WARN(
        "YOPO goal yaw alignment: target is %.1f deg outside camera heading; "
        "BRAKING at (%.3f %.3f %.3f) before turn-in-place.",
        yaw_error * 180.0 / kPi, position.x(), position.y(), position.z());
}

bool YopoPlanner::fillGoalAlignmentCommandLocked(quadrotor_msgs::PositionCommand* cmd) {
    if (!cmd || goal_alignment_state_ == GoalAlignmentState::INACTIVE) return false;

    const Eigen::Vector3d actual_position(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y,
        odom_.pose.pose.position.z);
    const double actual_yaw = currentYawLocked();

    if (goal_alignment_state_ == GoalAlignmentState::BRAKING) {
        const double horizontal_speed = std::hypot(
            odom_.twist.twist.linear.x, odom_.twist.twist.linear.y);
        if (horizontal_speed <= params_.goal_yaw_brake_speed_tolerance) {
            goal_alignment_stable_elapsed_ += params_.ctrl_dt;
        } else {
            goal_alignment_stable_elapsed_ = 0.0;
        }

        if (goal_alignment_stable_elapsed_ >= params_.goal_yaw_brake_stable_time) {
            const Eigen::Vector3d goal_delta = goal_ - actual_position;
            goal_alignment_state_          = GoalAlignmentState::TURNING;
            goal_alignment_hold_pos_       = actual_position;
            goal_alignment_command_yaw_    = actual_yaw;
            goal_alignment_target_yaw_     = std::atan2(goal_delta.y(), goal_delta.x());
            goal_alignment_stable_elapsed_ = 0.0;
            ROS_WARN(
                "YOPO goal yaw alignment: BRAKING -> TURNING, hold=(%.3f %.3f %.3f), "
                "target_yaw=%.1f deg.",
                goal_alignment_hold_pos_.x(), goal_alignment_hold_pos_.y(),
                goal_alignment_hold_pos_.z(), goal_alignment_target_yaw_ * 180.0 / kPi);
        }
    }

    double yaw_dot = 0.0;
    if (goal_alignment_state_ == GoalAlignmentState::TURNING) {
        const auto yaw = calculateTargetYaw(
            goal_alignment_target_yaw_, goal_alignment_command_yaw_, params_.ctrl_dt,
            params_.goal_yaw_align_rate);
        goal_alignment_command_yaw_ = yaw.first;
        yaw_dot                     = yaw.second;

        const double actual_yaw_error = wrapToPi(goal_alignment_target_yaw_ - actual_yaw);
        const double forward_sector = params_.goal_yaw_align_enter_deg / 180.0 * kPi;
        if (std::abs(actual_yaw_error) <= forward_sector) {
            goal_alignment_state_          = GoalAlignmentState::INACTIVE;
            goal_alignment_stable_elapsed_ = 0.0;
            goal_alignment_hold_pos_       = actual_position;
            goal_alignment_command_yaw_    = actual_yaw;
            syncReferenceFromOdomLocked();
            // Actual velocity is already below the braking threshold.  Start
            // the first post-turn trajectory from a true stationary reference.
            desire_vel_.setZero();
            desire_acc_.setZero();
            has_trajectory_       = false;
            has_last_control_msg_ = false;
            ctrl_time_            = 0.0;
            ROS_WARN(
                "YOPO goal yaw alignment: target entered +/-%.1f deg forward sector; "
                "TURNING -> PLANNING, actual yaw error %.1f deg.",
                params_.goal_yaw_align_enter_deg, actual_yaw_error * 180.0 / kPi);
        }
    }

    cmd->header.stamp    = ros::Time::now();
    cmd->trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
    cmd->position.x      = goal_alignment_hold_pos_.x();
    cmd->position.y      = goal_alignment_hold_pos_.y();
    cmd->position.z      = goal_alignment_hold_pos_.z();
    cmd->velocity.x      = 0.0;
    cmd->velocity.y      = 0.0;
    cmd->velocity.z      = 0.0;
    cmd->acceleration.x  = 0.0;
    cmd->acceleration.y  = 0.0;
    cmd->acceleration.z  = 0.0;
    cmd->jerk.x          = 0.0;
    cmd->jerk.y          = 0.0;
    cmd->jerk.z          = 0.0;
    cmd->yaw             = goal_alignment_command_yaw_;
    cmd->yaw_dot         = yaw_dot;

    desire_pos_ = goal_alignment_hold_pos_;
    desire_vel_.setZero();
    desire_acc_.setZero();
    last_yaw_            = goal_alignment_command_yaw_;
    desire_init_         = true;
    last_control_msg_     = *cmd;
    has_last_control_msg_ = true;
    return true;
}

YopoPlanner::EndState YopoPlanner::predToEndstate(
    const std::array<float, 1 * 9 * 3 * 5>& pred, int action_id) const {
    Eigen::Matrix<double, 9, 1> action_pred;
    for (int c = 0; c < 9; ++c) {
        action_pred[c] = pred[c * params_.traj_num + action_id];
    }
    const int lattice_id = params_.traj_num - 1 - action_id;
    return predToEndstateByLattice(action_pred, lattice_id);
}

YopoPlanner::EndState YopoPlanner::predToEndstateByLattice(
    const Eigen::Matrix<double, 9, 1>& pred, int lattice_id) const {
    EndState out;
    const double delta_yaw   = pred[0] * yaw_diff_;
    const double delta_pitch = pred[1] * pitch_diff_;
    const double radio       = (pred[2] + 1.0) * params_.radio_range;
    const double yaw         = lattice_angle_[lattice_id].x();
    const double pitch       = lattice_angle_[lattice_id].y();

    out.pos.x() = std::cos(pitch + delta_pitch) * std::cos(yaw + delta_yaw) * radio;
    out.pos.y() = std::cos(pitch + delta_pitch) * std::sin(yaw + delta_yaw) * radio;
    out.pos.z() = std::sin(pitch + delta_pitch) * radio;

    const Eigen::Vector3d vel_p = pred.segment<3>(3) * vel_max_;
    const Eigen::Vector3d acc_p = pred.segment<3>(6) * acc_max_;
    out.vel                     = lattice_rbp_[lattice_id] * vel_p;
    out.acc                     = lattice_rbp_[lattice_id] * acc_p;
    return out;
}

Eigen::Vector3d YopoPlanner::currentStartPos() const {
    if (params_.plan_from_reference) return desire_pos_;
    return Eigen::Vector3d(
        odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
}

Eigen::Vector3d YopoPlanner::currentStartVel() const {
    if (params_.plan_from_reference) return desire_vel_;
    return Eigen::Vector3d(
        odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, odom_.twist.twist.linear.z);
}

double YopoPlanner::wrapToPi(double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

std::pair<double, double> YopoPlanner::calculateYaw(
    const Eigen::Vector3d& vel_dir, const Eigen::Vector3d& goal_dir, double last_yaw, double dt) {
    const Eigen::Vector3d vel_norm  = vel_dir / (vel_dir.norm() + 1e-5);
    const double goal_dist          = goal_dir.norm();
    const Eigen::Vector3d goal_norm = goal_dir / (goal_dist + 1e-5);
    const double goal_yaw           = std::atan2(goal_norm.y(), goal_norm.x());
    const double delta_yaw          = wrapToPi(goal_yaw - last_yaw);
    const double weight             = 6.0 * std::abs(delta_yaw) / kPi;
    const Eigen::Vector3d dir_des   = vel_norm + weight * goal_norm;
    const double yaw_desired    = goal_dist > 0.5 ? std::atan2(dir_des.y(), dir_des.x()) : last_yaw;
    const double yaw_diff       = wrapToPi(yaw_desired - last_yaw);
    const double max_yaw_change = 0.5 * kPi * dt;
    const double yaw_change     = std::max(-max_yaw_change, std::min(max_yaw_change, yaw_diff));
    const double yaw            = wrapToPi(last_yaw + yaw_change);
    return {yaw, yaw_change / dt};
}

std::pair<double, double> YopoPlanner::calculateTargetYaw(
    double target_yaw, double last_yaw, double dt, double max_yaw_rate) {
    if (dt <= 0.0 || max_yaw_rate <= 0.0) {
        return {wrapToPi(last_yaw), 0.0};
    }

    const double yaw_error  = wrapToPi(target_yaw - last_yaw);
    const double max_change = max_yaw_rate * dt;
    const double yaw_change = std::max(-max_change, std::min(max_change, yaw_error));
    return {wrapToPi(last_yaw + yaw_change), yaw_change / dt};
}

std::pair<double, double> YopoPlanner::calculateTargetYawLimited(
    double target_yaw, double last_yaw, double last_yaw_rate, double dt,
    double max_yaw_rate, double max_yaw_accel, double deadband) {
    if (dt <= 0.0 || max_yaw_rate <= 0.0 || max_yaw_accel <= 0.0) {
        return {wrapToPi(last_yaw), 0.0};
    }

    const double error = wrapToPi(target_yaw - last_yaw);
    if (std::abs(error) <= std::max(0.0, deadband) &&
        std::abs(last_yaw_rate) <= max_yaw_accel * dt) {
        return {wrapToPi(last_yaw), 0.0};
    }

    // Limit the requested rate by both the configured maximum and the rate
    // from which the command can still stop at the target under max accel.
    const double stopping_rate = std::sqrt(2.0 * max_yaw_accel * std::abs(error));
    const double desired_rate = std::copysign(
        std::min(max_yaw_rate, stopping_rate), error);
    const double max_rate_change = max_yaw_accel * dt;
    double rate = last_yaw_rate + std::max(
        -max_rate_change, std::min(max_rate_change, desired_rate - last_yaw_rate));
    rate = std::max(-max_yaw_rate, std::min(max_yaw_rate, rate));

    double yaw_change = rate * dt;
    if ((error > 0.0 && yaw_change > error) ||
        (error < 0.0 && yaw_change < error)) {
        yaw_change = error;
        rate = yaw_change / dt;
    }
    return {wrapToPi(last_yaw + yaw_change), rate};
}

}  // namespace yopo_planner
