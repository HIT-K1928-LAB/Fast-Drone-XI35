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

void YopoPlanner::setGoal(const Eigen::Vector3d& goal) {
    std::lock_guard<std::mutex> lock(mutex_);
    goal_                      = goal;
    arrive_                    = false;
    arrival_hover_initialized_ = false;
    has_trajectory_            = false;
    has_last_control_msg_      = false;
    ctrl_time_                 = 0.0;
    desire_init_               = false;
    desire_acc_.setZero();
    if (odom_init_) {
        syncReferenceFromOdomLocked();
    }
}

void YopoPlanner::updateOdometry(const nav_msgs::Odometry& odom) {
    std::lock_guard<std::mutex> lock(mutex_);
    odom_ = odom;

    if (!desire_init_) {
        syncReferenceFromOdomLocked();
    }
    odom_init_ = true;

    const Eigen::Vector3d pos(
        odom.pose.pose.position.x, odom.pose.pose.position.y, odom.pose.pose.position.z);
    if ((pos - goal_).norm() < params_.arrive_distance && !arrive_) {
        const Eigen::Quaterniond q(
            odom.pose.pose.orientation.w, odom.pose.pose.orientation.x,
            odom.pose.pose.orientation.y, odom.pose.pose.orientation.z);
        const Eigen::Matrix3d rotation = q.toRotationMatrix();
        arrive_                        = true;
        arrival_hover_pos_             = pos;
        arrival_hover_yaw_             = std::atan2(rotation(1, 0), rotation(0, 0));
        arrival_hover_initialized_     = true;
    }
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

    const Eigen::Vector3d start_pos  = currentStartPos();
    const Eigen::Vector3d start_vel  = currentStartVel();
    const Eigen::Vector3d goal_dir_w = goal_ - start_pos;

    if (return_all_preds) {
        for (int i = 0; i < params_.traj_num; ++i) {
            last_all_endstates_[i] = predToEndstate(endstate_pred, i);
        }
    }

    int action_id       = 0;
    float best_score    = std::numeric_limits<float>::max();
    bool found_safe     = false;
    int rejected_height = 0;
    EndState best;
    Eigen::Matrix<double, 3, 3> best_endstate_w = Eigen::Matrix<double, 3, 3>::Zero();
    Eigen::Vector3d best_end_pos_w              = start_pos;

    for (int i = 0; i < params_.traj_num; ++i) {
        last_scores_[i] = score_pred[i];
        const EndState candidate =
            return_all_preds ? last_all_endstates_[i] : predToEndstate(endstate_pred, i);
        const Eigen::Matrix<double, 3, 3> candidate_c =
            (Eigen::Matrix<double, 3, 3>() << candidate.pos.x(), candidate.vel.x(),
             candidate.acc.x(), candidate.pos.y(), candidate.vel.y(), candidate.acc.y(),
             candidate.pos.z(), candidate.vel.z(), candidate.acc.z())
                .finished();
        const Eigen::Matrix<double, 3, 3> candidate_w = rotation_wc_ * candidate_c;
        const Eigen::Vector3d candidate_end_pos_w     = start_pos + candidate_w.col(0);
        if (candidate_end_pos_w.z() < params_.min_output_height) {
            ++rejected_height;
            continue;
        }
        if (score_pred[i] < best_score) {
            best_score       = score_pred[i];
            action_id        = i;
            best             = candidate;
            best_endstate_w  = candidate_w;
            best_end_pos_w   = candidate_end_pos_w;
            found_safe       = true;
        }
    }

    if (!found_safe) {
        best_score                    = score_pred[action_id];
        best.pos.setZero();
        best.vel.setZero();
        best.acc.setZero();
        best_endstate_w.setZero();
        best_end_pos_w = start_pos;
        best_end_pos_w.z() = std::max(start_pos.z(), params_.min_output_height);
        best_endstate_w(2, 0) = best_end_pos_w.z() - start_pos.z();
        ROS_WARN_THROTTLE(
            0.5,
            "YOPO height guard rejected all %d candidates below %.2fm; fallback end z=%.3f.",
            rejected_height, params_.min_output_height, best_end_pos_w.z());
    }

    if (!return_all_preds) {
        last_all_endstates_[0] = best;
        last_scores_[0]        = best_score;
        action_id              = 0;
    }

    const EndState& best_ref =
        found_safe ? (return_all_preds ? last_all_endstates_[action_id] : last_all_endstates_[0])
                   : best;
    const Eigen::Matrix<double, 3, 3> endstate_c =
        (Eigen::Matrix<double, 3, 3>() << best_ref.pos.x(), best_ref.vel.x(), best_ref.acc.x(),
         best_ref.pos.y(), best_ref.vel.y(), best_ref.acc.y(), best_ref.pos.z(), best_ref.vel.z(),
         best_ref.acc.z())
            .finished();
    const Eigen::Matrix<double, 3, 3> endstate_w =
        found_safe ? rotation_wc_ * endstate_c : best_endstate_w;
    const Eigen::Vector3d end_delta_w = endstate_w.col(0);
    const Eigen::Vector3d end_pos_w   = start_pos + end_delta_w;
    ROS_WARN_THROTTLE(
        0.5,
        "YOPO best end pos: start=(%.3f %.3f %.3f), local_yopo_rel=(%.3f %.3f %.3f), "
        "world_delta=(%.3f %.3f %.3f), world_abs=(%.3f %.3f %.3f), goal_dir=(%.3f %.3f %.3f), "
        "score=%.3f, height_rejected=%d",
        start_pos.x(), start_pos.y(), start_pos.z(), best_ref.pos.x(), best_ref.pos.y(), best_ref.pos.z(),
        end_delta_w.x(), end_delta_w.y(), end_delta_w.z(), end_pos_w.x(), end_pos_w.y(),
        end_pos_w.z(), goal_dir_w.x(), goal_dir_w.y(), goal_dir_w.z(), best_score,
        rejected_height);
    poly_x_.reset(
        start_pos.x(), start_vel.x(), desire_acc_.x(), endstate_w(0, 0) + start_pos.x(),
        endstate_w(0, 1), endstate_w(0, 2), segment_time_);
    poly_y_.reset(
        start_pos.y(), start_vel.y(), desire_acc_.y(), endstate_w(1, 0) + start_pos.y(),
        endstate_w(1, 1), endstate_w(1, 2), segment_time_);
    poly_z_.reset(
        start_pos.z(), start_vel.z(), desire_acc_.z(), endstate_w(2, 0) + start_pos.z(),
        endstate_w(2, 1), endstate_w(2, 2), segment_time_);
    ctrl_time_      = 0.0;
    has_trajectory_ = true;
}

bool YopoPlanner::fillControlCommand(quadrotor_msgs::PositionCommand* cmd) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (params_.require_camera_extrinsic && !camera_extrinsic_ready_) return false;
    if (!cmd || !has_trajectory_ || ctrl_time_ > segment_time_) return false;

    if (arrive_) {
        if (!arrival_hover_initialized_ && odom_init_) {
            const Eigen::Quaterniond q(
                odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x,
                odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z);
            const Eigen::Matrix3d rotation = q.toRotationMatrix();
            arrival_hover_pos_             = Eigen::Vector3d(
                odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z);
            arrival_hover_yaw_         = std::atan2(rotation(1, 0), rotation(0, 0));
            arrival_hover_initialized_ = true;
        }

        cmd->header.stamp    = ros::Time::now();
        cmd->trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_EMPTY;
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
        cmd->yaw             = arrival_hover_yaw_;
        cmd->yaw_dot         = 0.0;
        clampCommandHeightLocked(cmd);

        if (odom_init_) {
            syncReferenceFromOdomLocked();
        } else {
            desire_pos_ = arrival_hover_pos_;
            desire_vel_.setZero();
            desire_acc_.setZero();
            last_yaw_ = arrival_hover_yaw_;
        }
        desire_init_          = false;
        last_control_msg_     = *cmd;
        has_last_control_msg_ = true;
        return true;
    }

    ctrl_time_ += params_.ctrl_dt;
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
    clampCommandHeightLocked(cmd);

    desire_pos_    = Eigen::Vector3d(cmd->position.x, cmd->position.y, cmd->position.z);
    desire_vel_    = Eigen::Vector3d(cmd->velocity.x, cmd->velocity.y, cmd->velocity.z);
    desire_acc_    = Eigen::Vector3d(cmd->acceleration.x, cmd->acceleration.y, cmd->acceleration.z);
    const auto yaw = calculateYaw(desire_vel_, goal_ - desire_pos_, last_yaw_, params_.ctrl_dt);
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
        const double t = segment_time_ * static_cast<double>(i) / static_cast<double>(samples);
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

void YopoPlanner::clampCommandHeightLocked(quadrotor_msgs::PositionCommand* cmd) const {
    if (!cmd || cmd->position.z >= params_.min_output_height) return;

    cmd->position.z = params_.min_output_height;
    if (cmd->velocity.z < 0.0) cmd->velocity.z = 0.0;
    if (cmd->acceleration.z < 0.0) cmd->acceleration.z = 0.0;
    if (cmd->jerk.z < 0.0) cmd->jerk.z = 0.0;
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

double YopoPlanner::wrapToPi(double angle) { return std::fmod(angle + kPi, 2.0 * kPi) - kPi; }

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

}  // namespace yopo_planner
