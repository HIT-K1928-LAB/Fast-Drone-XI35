#include "controller.h"
#include "PX4CtrlFSM.h"
#include "input.h"
#include <algorithm>
#include <cmath>
#include <type_traits>

using namespace std;

namespace {

void setPoint(const Eigen::Vector3d &value, geometry_msgs::Point &msg) {
    msg.x = value.x();
    msg.y = value.y();
    msg.z = value.z();
}

void setVector3(const Eigen::Vector3d &value, geometry_msgs::Vector3 &msg) {
    msg.x = value.x();
    msg.y = value.y();
    msg.z = value.z();
}

Eigen::Vector3d quaternionToRpy(const Eigen::Quaterniond &q) {
    const Eigen::Quaterniond n = q.normalized();
    const double roll          = std::atan2(
        2.0 * (n.w() * n.x() + n.y() * n.z()), 1.0 - 2.0 * (n.x() * n.x() + n.y() * n.y()));
    const double sin_pitch = std::max(-1.0, std::min(1.0, 2.0 * (n.w() * n.y() - n.z() * n.x())));
    const double pitch     = std::asin(sin_pitch);
    const double yaw       = std::atan2(
        2.0 * (n.w() * n.z() + n.x() * n.y()), 1.0 - 2.0 * (n.y() * n.y() + n.z() * n.z()));
    return Eigen::Vector3d(roll, pitch, yaw);
}

double wrapAngle(const double angle) { return std::atan2(std::sin(angle), std::cos(angle)); }

double clampValue(const double value, const double lower, const double upper) {
    return std::max(lower, std::min(upper, value));
}

}  // namespace

double LinearControl::fromQuaternion2yaw(Eigen::Quaterniond q) {
    double yaw = atan2(
        2 * (q.x() * q.y() + q.w() * q.z()),
        q.w() * q.w() + q.x() * q.x() - q.y() * q.y() - q.z() * q.z());
    return yaw;
}

LinearControl::LinearControl(Parameter_t &param) : param_(param) { resetThrustMapping(); }

ros::Time LinearControl::odomSampleStamp(const Odom_Data_t &odom) const {
    if (!odom.msg.header.stamp.isZero()) return odom.msg.header.stamp;
    return odom.rcv_stamp;
}

void LinearControl::resetControlState(const Odom_Data_t &odom) {
    v_filt_ = odom.v;
    e_v_int_.setZero();
    last_ctrl_time_            = ros::Time::now();
    last_odom_stamp_           = odomSampleStamp(odom);
    last_output_saturated_     = false;
    first_control_after_reset_ = true;
    control_state_initialized_ = true;
}

void LinearControl::updateVelocityFilter(const Odom_Data_t &odom) {
    const ros::Time sample_stamp = odomSampleStamp(odom);

    // Headerless odometry cannot be de-duplicated reliably. Preserve legacy behavior
    // for that case while all normal ROS odometry messages use their source stamp.
    if (sample_stamp.isZero()) {
        v_filt_ = odom.v;
        return;
    }

    if (last_odom_stamp_.isZero()) {
        v_filt_          = odom.v;
        last_odom_stamp_ = sample_stamp;
        return;
    }

    if (sample_stamp == last_odom_stamp_) return;

    const double sample_dt = (sample_stamp - last_odom_stamp_).toSec();
    last_odom_stamp_       = sample_stamp;

    if (!param_.velocity_filter.enable || sample_dt <= 0.0 ||
        sample_dt > param_.velocity_filter.reset_after_gap) {
        v_filt_ = odom.v;
        return;
    }

    const double cutoff       = param_.velocity_filter.cutoff_frequency;
    const double filter_alpha = 1.0 - std::exp(-2.0 * M_PI * cutoff * sample_dt);
    v_filt_ += filter_alpha * (odom.v - v_filt_);
}

/*
  compute u.thrust and u.q, controller gains and other parameters are in param_
*/
quadrotor_msgs::Px4ctrlDebug LinearControl::calculateControl(
    const Desired_State_t &des, const Odom_Data_t &odom, const Imu_Data_t &imu,
    Controller_Output_t &u) {
    /* WRITE YOUR CODE HERE */
    // compute disired acceleration
    Eigen::Vector3d des_acc(0.0, 0.0, 0.0);
    Eigen::Vector3d Kp, Kd, Ki;
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Kd << param_.gain.Kd0, param_.gain.Kd1, param_.gain.Kd2;
    Ki << param_.gain.Ki0, param_.gain.Ki1, param_.gain.Ki2;

    if (!control_state_initialized_) resetControlState(odom);
    updateVelocityFilter(odom);

    const ros::Time now = ros::Time::now();
    double dt           = (now - last_ctrl_time_).toSec();
    last_ctrl_time_     = now;
    if (first_control_after_reset_ || !std::isfinite(dt) || dt <= 0.0 || dt > 0.1) {
        dt = 0.0;
    }
    first_control_after_reset_ = false;

    Eigen::Vector3d e_p = des.p - odom.p;
    Eigen::Vector3d e_v = des.v - v_filt_;

    // position integral (slow, bounded)
    if (!last_output_saturated_) e_v_int_ += e_v * dt;
    double i_limit = 1.0;
    e_v_int_       = e_v_int_.cwiseMax(-i_limit).cwiseMin(i_limit);
    e_v_int_ *= 0.995;
    e_v_int_.z() = 0;

    // final acceleration command
    const Eigen::Vector3d acc_p           = Kp.asDiagonal() * e_p;
    const Eigen::Vector3d acc_d           = Kd.asDiagonal() * e_v;
    const Eigen::Vector3d acc_i           = Ki.asDiagonal() * e_v_int_;
    const Eigen::Vector3d acc_feedback    = acc_p + acc_d + acc_i;
    const Eigen::Vector3d acc_feedforward = des.a;
    const Eigen::Vector3d gravity_compensation(0, 0, param_.gra);
    const Eigen::Vector3d des_acc_raw = acc_feedback + acc_feedforward + gravity_compensation;
    des_acc                           = des_acc_raw;

    bool acceleration_limit_exceeded = false;
    if (param_.control_limits.enable) {
        const double horizontal_limit = param_.control_limits.max_horizontal_acceleration;
        const double horizontal_norm  = des_acc.head<2>().norm();
        if (horizontal_limit > 0.0 && horizontal_norm > horizontal_limit) {
            des_acc.head<2>() *= horizontal_limit / horizontal_norm;
            acceleration_limit_exceeded = true;
        }

        const double vertical_acceleration   = des_acc.z() - param_.gra;
        double limited_vertical_acceleration = vertical_acceleration;
        if (param_.control_limits.max_vertical_acceleration_down >= 0.0) {
            limited_vertical_acceleration = std::max(
                -param_.control_limits.max_vertical_acceleration_down,
                limited_vertical_acceleration);
        }
        if (param_.control_limits.max_vertical_acceleration_up >= 0.0) {
            limited_vertical_acceleration = std::min(
                param_.control_limits.max_vertical_acceleration_up, limited_vertical_acceleration);
        }
        if (limited_vertical_acceleration != vertical_acceleration) {
            acceleration_limit_exceeded = true;
            des_acc.z()                 = param_.gra + limited_vertical_acceleration;
        }
    }

    double roll, pitch;
    double yaw_odom           = fromQuaternion2yaw(odom.q);
    double sin                = std::sin(yaw_odom);
    double cos                = std::cos(yaw_odom);
    roll                      = (des_acc(0) * sin - des_acc(1) * cos) / param_.gra;
    pitch                     = (des_acc(0) * cos + des_acc(1) * sin) / param_.gra;
    Eigen::Quaterniond q_odom = Eigen::AngleAxisd(des.yaw, Eigen::Vector3d::UnitZ()) *
                                Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(roll, Eigen::Vector3d::UnitX());
    Eigen::Quaterniond q_command = (imu.q * odom.q.inverse() * q_odom).normalized();

    const Eigen::Vector3d raw_command_rpy = quaternionToRpy(q_command);
    const Eigen::Vector3d raw_body_z      = q_command * Eigen::Vector3d::UnitZ();
    const double raw_cos_tilt             = clampValue(raw_body_z.z(), -1.0, 1.0);
    const double raw_tilt                 = std::acos(raw_cos_tilt);
    bool tilt_limit_exceeded              = false;

    if (param_.control_limits.enable && param_.max_angle >= 0.0 && raw_tilt > param_.max_angle) {
        const double roll_pitch_norm = raw_command_rpy.head<2>().norm();
        if (roll_pitch_norm > 1e-6) {
            const double scale         = param_.max_angle / roll_pitch_norm;
            const double limited_roll  = raw_command_rpy.x() * scale;
            const double limited_pitch = raw_command_rpy.y() * scale;
            q_command = Eigen::AngleAxisd(raw_command_rpy.z(), Eigen::Vector3d::UnitZ()) *
                        Eigen::AngleAxisd(limited_pitch, Eigen::Vector3d::UnitY()) *
                        Eigen::AngleAxisd(limited_roll, Eigen::Vector3d::UnitX());
        }
        tilt_limit_exceeded = true;
    }
    u.q = q_command.normalized();

    const Eigen::Vector3d command_rpy = quaternionToRpy(u.q);
    Eigen::Vector3d des_acc_thr(0.0, 0.0, des_acc(2));
    const double tilt_compensation =
        std::max(0.2, std::cos(command_rpy.y()) * std::cos(command_rpy.x()));
    des_acc_thr(2)             = des_acc_thr(2) / tilt_compensation;
    const double thrust_raw    = computeDesiredCollectiveThrustSignal(des_acc_thr);
    bool thrust_limit_exceeded = false;
    if (!std::isfinite(thrust_raw)) {
        u.thrust = param_.control_limits.enable ? param_.control_limits.min_thrust : 0.0;
        thrust_limit_exceeded = true;
    } else if (param_.control_limits.enable) {
        u.thrust = clampValue(
            thrust_raw, param_.control_limits.min_thrust, param_.control_limits.max_thrust);
        thrust_limit_exceeded = u.thrust != thrust_raw;
    } else {
        u.thrust = thrust_raw;
    }

    last_output_saturated_ = acceleration_limit_exceeded || tilt_limit_exceeded ||
                             thrust_limit_exceeded;

    // used for debug
    debug_msg_.des_v_x = des.v(0);
    debug_msg_.des_v_y = des.v(1);
    debug_msg_.des_v_z = des.v(2);

    debug_msg_.des_a_x = des_acc(0);
    debug_msg_.des_a_y = des_acc(1);
    debug_msg_.des_a_z = des_acc(2);

    debug_msg_.des_q_x = u.q.x();
    debug_msg_.des_q_y = u.q.y();
    debug_msg_.des_q_z = u.q.z();
    debug_msg_.des_q_w = u.q.w();

    debug_msg_.des_thr = u.thrust;

    if (param_.tuning_debug.enable) {
        setPoint(des.p, tune_debug_msg_.position_des);
        setPoint(odom.p, tune_debug_msg_.position_meas);
        setVector3(e_p, tune_debug_msg_.position_error);

        setVector3(des.v, tune_debug_msg_.velocity_des);
        setVector3(odom.v, tune_debug_msg_.velocity_meas);
        setVector3(v_filt_, tune_debug_msg_.velocity_filtered);
        setVector3(e_v, tune_debug_msg_.velocity_error);

        setVector3(acc_feedforward, tune_debug_msg_.acceleration_feedforward);
        setVector3(gravity_compensation, tune_debug_msg_.gravity_compensation);
        setVector3(acc_p, tune_debug_msg_.acceleration_proportional);
        setVector3(acc_d, tune_debug_msg_.acceleration_derivative);
        setVector3(acc_feedback, tune_debug_msg_.acceleration_feedback);
        setVector3(acc_i, tune_debug_msg_.acceleration_integral);
        setVector3(des_acc_raw, tune_debug_msg_.acceleration_command_raw);
        setVector3(des_acc, tune_debug_msg_.acceleration_command);
        setVector3(imu.a, tune_debug_msg_.specific_force_body_meas);
        setVector3(e_v_int_, tune_debug_msg_.velocity_error_integral);

        const Eigen::Vector3d attitude_des  = quaternionToRpy(u.q);
        const Eigen::Vector3d attitude_meas = quaternionToRpy(imu.q);
        Eigen::Vector3d attitude_error;
        attitude_error.x() = wrapAngle(attitude_des.x() - attitude_meas.x());
        attitude_error.y() = wrapAngle(attitude_des.y() - attitude_meas.y());
        attitude_error.z() = wrapAngle(attitude_des.z() - attitude_meas.z());
        setVector3(attitude_des, tune_debug_msg_.attitude_des_rpy);
        setVector3(attitude_meas, tune_debug_msg_.attitude_meas_rpy);
        setVector3(attitude_error, tune_debug_msg_.attitude_error_rpy);
        setVector3(imu.w, tune_debug_msg_.body_rate_meas);

        setVector3(Kp, tune_debug_msg_.kp);
        setVector3(Ki, tune_debug_msg_.ki);
        setVector3(Kd, tune_debug_msg_.kd);

        tune_debug_msg_.thrust_raw                  = thrust_raw;
        tune_debug_msg_.thrust_command              = u.thrust;
        tune_debug_msg_.thrust_to_accel             = thr2acc_;
        const Eigen::Vector3d body_z                = u.q * Eigen::Vector3d::UnitZ();
        const double cos_tilt                       = std::max(-1.0, std::min(1.0, body_z.z()));
        tune_debug_msg_.tilt_raw                    = raw_tilt;
        tune_debug_msg_.tilt_command                = std::acos(cos_tilt);
        tune_debug_msg_.acceleration_limit_exceeded = acceleration_limit_exceeded;
        tune_debug_msg_.tilt_limit_exceeded         = tilt_limit_exceeded;
        tune_debug_msg_.thrust_limit_exceeded       = thrust_limit_exceeded;
        tune_debug_msg_.control_dt                  = dt;
    }

    // Used for thrust-accel mapping estimation
    timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));

    while (timed_thrust_.size() > 100) {
        timed_thrust_.pop();
    }
    return debug_msg_;
}

/*
  compute throttle percentage
*/
double LinearControl::computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc) {
    double throttle_percentage(0.0);

    /* compute throttle, thr2acc has been estimated before */
    throttle_percentage = des_acc(2) / thr2acc_;

    return throttle_percentage;
}

bool LinearControl::estimateThrustModel(const Eigen::Vector3d &est_a, const Parameter_t &param) {
    if (!param.thr_map.online_estimation) return false;

    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1) {
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed               = (t_now - t_t.first).toSec();
        if (time_passed > 0.045)  // 45ms
        {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035)  // 35ms
        {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K     = gamma * P_ * thr;
        thr2acc_     = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
        P_           = (1 - K * thr) * P_ / rho2_;
        if (param_.thr_map.print_val == true) {
            ROS_INFO(
                "%6.3f,%6.3f,%6.3f,%6.3f,%6.3f,%6.3f\n", est_a(2), thr, thr2acc_, gamma, K, P_);
            fflush(stdout);
        }

        debug_msg_.thr_scale_compensate = thr2acc_;
        return true;
    }
    return false;
}

bool LinearControl::estimateThrustModel(
    const Eigen::Vector3d &est_a, const Parameter_t &param, const Battery_Data_t &bat_data) {
    if (!param.thr_map.online_estimation) return false;

    ros::Time t_now = ros::Time::now();

    while (!timed_thrust_.empty()) {
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        double time_passed               = (t_now - t_t.first).toSec();

        if (time_passed > 0.045) {
            timed_thrust_.pop();
            continue;
        }
        if (time_passed < 0.035) {
            return false;
        }

        double thr = t_t.second;
        timed_thrust_.pop();

        double measured_a = est_a(2);

        if (thr < 0.1) {
            return false;
        }

        double acc_variance = std::abs(measured_a - 9.8);
        if (acc_variance < 0.1) {
            return false;
        }

        double error = measured_a - thr * thr2acc_;

        // Update K (Gain)
        double gamma = 1.0 / (rho2_ + thr * P_ * thr);
        double K     = P_ * thr * gamma;

        // Update Estimate
        thr2acc_ = thr2acc_ + K * error;

        // Update Covariance
        P_ = (P_ - K * thr * P_) / rho2_;

        double min_thr2acc = 10.0;
        double max_thr2acc = 45.0;
        if (thr2acc_ < min_thr2acc) {
            thr2acc_ = min_thr2acc;
        }
        if (thr2acc_ > max_thr2acc) {
            thr2acc_ = max_thr2acc;
        }

        if (P_ > 100.0) {
            P_ = 100.0;
        }
        if (P_ < 0.01) {
            P_ = 0.01;
        }

        debug_msg_.thr_scale_compensate = thr2acc_;
        return true;
    }
    return false;
}

bool LinearControl::estimateThrustModelUsingVelFB(
    const Eigen::Vector3d &est_v, const Parameter_t &param) {
    if (!param.thr_map.online_estimation) return false;

    ros::Time t_now = ros::Time::now();
    while (timed_thrust_.size() >= 1) {
        // Choose data before 35~45ms ago
        std::pair<ros::Time, double> t_t = timed_thrust_.front();
        Eigen::Vector3d t_v              = timed_vel_.front();
        double time_passed               = (t_now - t_t.first).toSec();
        if (time_passed > 0.045)  // 45ms
        {
            // printf("continue, time_passed=%f\n", time_passed);
            timed_thrust_.pop();
            timed_vel_.pop();
            continue;
        }
        if (time_passed < 0.035)  // 35ms
        {
            // printf("skip, time_passed=%f\n", time_passed);
            return false;
        }

        /***********************************************************/
        /* Recursive least squares algorithm with vanishing memory */
        /***********************************************************/
        double thr = t_t.second;
        timed_thrust_.pop();

        /***********************************/
        /* Model: est_a(2) = thr1acc_ * thr */
        /***********************************/
        double gamma = 1 / (rho2_ + thr * P_ * thr);
        double K     = gamma * P_ * thr;
        double est_a = (est_v(2) - t_v(2)) / time_passed;
        thr2acc_     = thr2acc_ + K * (est_a - thr * thr2acc_);
        P_           = (1 - K * thr) * P_ / rho2_;
        // printf("%6.3f,%6.3f,%6.3f,%6.3f\n", thr2acc_, gamma, K, P_);
        // fflush(stdout);

        debug_msg_.thr_scale_compensate = thr2acc_;
        return true;
    }
    return false;
}

void LinearControl::resetThrustMapping(void) {
    thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
    P_       = 1e6;
}

void LinearControl::resetThrustMapping(Battery_Data_t &bat_data) {
    printf("Recieve bat volt: %f  V\n", bat_data.volt);

    double volt = bat_data.volt;
    double tmp  = volt2HoverPerOverM0(volt);
    thr2acc_    = param_.gra / (param_.mass * tmp);

    printf("hover percentage: %f\n", param_.mass * tmp);

    P_ = 1e6;
}

double LinearControl::volt2HoverPerOverM0(double volt) {
    double tmp;
    if (volt > 14.6713340547349) {
        tmp = -0.0006 * volt + 0.4067;
        return tmp;
    }
    tmp = 0.0027 * (volt * volt) - 0.1096 * volt + 1.4284;

    return tmp;
}
