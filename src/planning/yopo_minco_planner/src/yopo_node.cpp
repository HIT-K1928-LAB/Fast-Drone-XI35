#include "yopo_minco_planner/yopo_planner.h"
#include "yopo_minco_planner/recovery_gate.h"
#include "yopo_minco_planner/StartGoal.h"
#include <std_srvs/Trigger.h>
#include <std_msgs/Bool.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/ExtendedState.h>
#include <quadrotor_msgs/TakeoffLand.h>
#include <sstream>
#include <chrono>
#include <cmath>
#include <cstring>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <mavros_msgs/State.h>
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/photo.hpp>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/callback_queue.h>
#include <ros/master.h>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <stdexcept>

namespace yopo_minco_planner {
using Clock = std::chrono::steady_clock;
static double seconds(Clock::time_point a, Clock::time_point b) {
    return std::chrono::duration<double>(a - b).count();
}
class YopoMincoNode {
  public:
    YopoMincoNode() : private_("~") {
        private_.param("execute", execute_, false);
        private_.param("plan_from_reference", reference_, false);
        private_.param("depth_replan_rate", plan_hz_, 10.);
        private_.param("ctrl_dt", ctrl_dt_, 0.02);
        private_.param("sensor_timeout", sensor_timeout_, 0.3);
        private_.param("state_timeout", state_timeout_, 1.);
        private_.param("max_sensor_skew", skew_, 0.1);
        private_.param("plan_timeout", plan_timeout_, 0.25);
        private_.param("max_tracking_error", tracking_error_, 0.5);
        private_.param("goal_min_distance", goal_min_, 0.5);
        private_.param("goal_max_distance", goal_max_, 2.);
        private_.param("arrive_distance", arrive_distance_, 0.25);
        private_.param("arrive_speed", arrive_speed_, 0.15);
        private_.param("arrive_hold", arrive_hold_, 0.5);
        private_.param("hover_speed", hover_speed_, 0.15);
        private_.param("min_depth", min_depth_, 0.04);
        private_.param("max_depth", max_depth_, 20.);
        private_.param("max_invalid_depth_fraction", max_invalid_, 0.3);
        private_.param("depth_width", width_, 640);
        private_.param("depth_height", height_, 480);
        private_.param("visualize", visualize_, false);
#define PARAM(field) private_.param(#field, config_.field, config_.field)
        PARAM(velocity);
        PARAM(training_velocity);
        PARAM(training_acceleration);
        PARAM(tail_range);
        PARAM(horizontal_fov);
        PARAM(vertical_fov);
        PARAM(horizontal_anchor_fov);
        PARAM(vertical_anchor_fov);
        PARAM(duration_min);
        PARAM(radius_lambda);
        PARAM(safe_radius);
        PARAM(max_speed);
        PARAM(max_acceleration);
        PARAM(max_jerk);
        PARAM(min_height);
        PARAM(max_height);
        PARAM(height_band);
        PARAM(test_radius);
        PARAM(max_duration);
        PARAM(check_dt);
#undef PARAM
        const double positive[] = {plan_hz_,     ctrl_dt_,         sensor_timeout_, state_timeout_,
                                   skew_,        plan_timeout_,    tracking_error_, goal_min_,
                                   goal_max_,    arrive_distance_, arrive_speed_,   arrive_hold_,
                                   hover_speed_, min_depth_,       max_depth_,      max_invalid_};
        for (double v : positive)
            if (!std::isfinite(v) || v <= 0) throw std::runtime_error("Invalid node parameter");
        if (width_ <= 0 || height_ <= 0 || width_ > 4096 || height_ > 4096 ||
            min_depth_ >= max_depth_ || max_invalid_ >= 1 || arrive_distance_ >= goal_min_ ||
            goal_min_ >= goal_max_ || ctrl_dt_ > 0.1 || plan_hz_ > 100 ||
            plan_timeout_ <= 1 / plan_hz_)
            throw std::runtime_error("Inconsistent node parameter bounds");
        if (ros::Time::isSimTime())
            throw std::runtime_error(
                "Real-flight launch requires /use_sim_time=false; replay uses wall-time/restamped "
                "input");
        private_.param("recovery_enabled", recovery_enabled_, true);
        private_.param("recovery_auto_resume", auto_resume_, false);
        private_.param("recovery_stable_hold", recovery_.stable_hold, 1.0);
        private_.param("recovery_hover_speed", recovery_.speed_limit, 0.10);
        private_.param("recovery_position_band", recovery_.position_band, 0.10);
        private_.param("recovery_timeout", recovery_.timeout, 30.0);
        private_.param("recovery_good_span", recovery_.good_span, 0.4);
        private_.param("recovery_good_frames", recovery_.required_frames, 5);
        private_.param("recovery_max_attempts", recovery_.max_attempts, 3);
        private_.param("recovery_max_no_progress", recovery_.max_no_progress, 2);
        private_.param("recovery_min_progress", recovery_.min_progress, 0.05);
        double cmd_timeout=0.0;
        if (execute_ && recovery_enabled_ &&
            (!nh_.getParam("/px4ctrl/msg_timeout/cmd", cmd_timeout) ||
             !std::isfinite(cmd_timeout) || cmd_timeout<=0))
            throw std::runtime_error("Recovery requires PX4Ctrl cmd timeout parameter");
        recovery_.release_wait=std::max(0.5, cmd_timeout)+0.5;
        recovery_.max_frame_gap=plan_timeout_;
        for (double x : {recovery_.stable_hold, recovery_.speed_limit,
                         recovery_.position_band, recovery_.timeout,
                         recovery_.good_span, recovery_.min_progress})
            if (!std::isfinite(x) || x<=0) throw std::runtime_error("Invalid recovery parameter");
        if (recovery_.required_frames<2 || recovery_.max_attempts<1 ||
            recovery_.max_no_progress<1 || recovery_.speed_limit>hover_speed_ ||
            recovery_.timeout<=recovery_.release_wait+recovery_.stable_hold+recovery_.good_span)
            throw std::runtime_error("Inconsistent recovery bounds");
        std::string file, key;
        private_.param<std::string>("camera_extrinsic_config", file, "");
        private_.param<std::string>("camera_extrinsic_key", key, "body_T_cam0");
        cv::FileStorage fs(file, cv::FileStorage::READ);
        cv::Mat matrix;
        if (!fs.isOpened()) throw std::runtime_error("Cannot open camera_extrinsic_config");
        fs[key] >> matrix;
        if (matrix.rows != 4 || matrix.cols != 4)
            throw std::runtime_error("Camera extrinsic must be 4x4");
        matrix.convertTo(matrix, CV_64F);
        Eigen::Matrix3d optical;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) optical(i, j) = matrix.at<double>(i, j);
        if (!optical.allFinite() ||
            (optical.transpose() * optical - Eigen::Matrix3d::Identity()).norm() > 0.02 ||
            std::abs(optical.determinant() - 1) > 0.02)
            throw std::runtime_error("Invalid camera rotation");
        Eigen::Matrix3d optical_yopo;
        optical_yopo << 0, -1, 0, 0, 0, -1, 1, 0, 0;
        rotation_bc_ = optical * optical_yopo;
        ROS_WARN(
            "Camera rotation loaded; translation is not used. Verify cam0/depth/body alignment "
            "before execution.");
        planner_ = std::make_unique<YopoPlanner>(config_);
        private_.param<std::string>("engine_file", engine_file_, "");
        if (!engine_.load(engine_file_))
            throw std::runtime_error("Failed to load native FP16-binding MINCO engine");
        Depth dummy_depth{};
        dummy_depth.fill(1.f);
        Observation dummy_obs{};
        NetworkOutput dummy_output;
        if (!engine_.infer(dummy_depth, dummy_obs, &dummy_output))
            throw std::runtime_error("Engine warmup failed");
        std::string odom, depth, state, goal, trigger, stop;
        private_.param<std::string>("odom_topic", odom, "/kf_fusion/kf_imu_odom");
        private_.param<std::string>("depth_topic", depth, "/camera/depth/image_rect_raw");
        private_.param<std::string>("state_topic", state, "/mavros/state");
        private_.param<std::string>("goal_topic", goal, "/move_base_simple/goal");
        private_.param<std::string>("trigger_topic", trigger, "/traj_start_trigger");
        private_.param<std::string>("stop_topic", stop, "/yopo_minco/stop");
        output_topic_ = execute_ ? "/position_cmd" : "/yopo_minco/debug/position_cmd";
        // Debug mode must never be remapped to the flight controller topic.
        if (!execute_ && nh_.resolveName(output_topic_) != output_topic_)
            throw std::runtime_error("Debug command-topic remapping is disabled");
        if (execute_ && !singlePublisher())
            throw std::runtime_error(
                "Another publisher exists or ROS master unavailable on /position_cmd");
        commands_ = nh_.advertise<quadrotor_msgs::PositionCommand>(output_topic_, 1);
        status_   = nh_.advertise<std_msgs::String>("/yopo_minco/status", 1, true);
        recovery_status_ = nh_.advertise<std_msgs::String>("/yopo_minco/recovery_state", 1, true);
        accepted_goal_ = nh_.advertise<geometry_msgs::PoseStamped>("/yopo_minco/accepted_goal", 1, true);
        start_service_ = nh_.advertiseService("/yopo_minco/start_goal", &YopoMincoNode::startGoal, this);
        resume_service_ = nh_.advertiseService("/yopo_minco/resume", &YopoMincoNode::resume, this);
        fault_sub_ = nh_.subscribe("/kf_fusion/kf_fail", 1, &YopoMincoNode::faultCallback, this);
        vins_fault_sub_ = nh_.subscribe("/vins_fusion/vins_fail", 1, &YopoMincoNode::faultCallback, this);
        imu_watch_sub_ = nh_.subscribe("/imu_filter/data", 10, &YopoMincoNode::imuWatch, this);
        rc_sub_ = nh_.subscribe("/mavros/rc/in", 1, &YopoMincoNode::rcCallback, this);
        extended_sub_ = nh_.subscribe("/mavros/extended_state", 1, &YopoMincoNode::extendedCallback, this);
        takeoff_land_sub_ = nh_.subscribe("/px4ctrl/takeoff_land", 1, &YopoMincoNode::takeoffLandCallback, this);
        path_     = nh_.advertise<geometry_msgs::PoseArray>("/yopo_minco/best_trajectory", 1);
        odom_sub_ = nh_.subscribe(
            odom, 1, &YopoMincoNode::odomCallback, this, ros::TransportHints().tcpNoDelay());
        state_sub_   = nh_.subscribe(state, 1, &YopoMincoNode::stateCallback, this);
        goal_sub_    = nh_.subscribe(goal, 1, &YopoMincoNode::goalCallback, this);
        trigger_sub_ = nh_.subscribe(trigger, 1, &YopoMincoNode::triggerCallback, this);
        stop_sub_    = nh_.subscribe(stop, 1, &YopoMincoNode::stopCallback, this);
        // GPU/planning has its own single-threaded queue; never hold mutex across inference.
        depth_nh_.setCallbackQueue(&depth_queue_);
        depth_sub_ = depth_nh_.subscribe(
            depth, 1, &YopoMincoNode::depthCallback, this, ros::TransportHints().tcpNoDelay());
        timer_ =
            nh_.createWallTimer(ros::WallDuration(ctrl_dt_), &YopoMincoNode::controlCallback, this);
        publisher_timer_ =
            nh_.createWallTimer(ros::WallDuration(1.), &YopoMincoNode::publisherCheck, this);
        depth_spinner_ = std::make_unique<ros::AsyncSpinner>(1, &depth_queue_);
        publishStatus(
            "IDLE: use minco_goal console (world goal + start); execute=" +
            std::to_string(execute_)+"; auto_resume="+std::to_string(auto_resume_));
        depth_spinner_->start();
    }
    ~YopoMincoNode() {
        if (depth_spinner_) depth_spinner_->stop();
    }

  private:
    void publishStatus(const std::string& s) {
        std_msgs::String msg;
        msg.data = s;
        status_.publish(msg);
        ROS_WARN_STREAM(s);
        recovery_status_.publish(msg);
    }
    void halt(const std::string& why) {
        recovery_.stop(why);
        resume_requested_ = false;
        have_goal_ = false;  // A later unrelated trigger cannot restart the old task.
        active_    = false;
        have_plan_ = false;
        ++generation_;
        arrival_since_ = Clock::time_point{};
        publishStatus("STOP: " + why + "; commands released, no automatic resume");
    }
    bool singlePublisher() {
        XmlRpc::XmlRpcValue args, result, payload;
        args[0] = ros::this_node::getName();
        if (!ros::master::execute("getSystemState", args, result, payload, false)) return false;
        const auto& publishers = payload[0];
        for (int i = 0; i < publishers.size(); ++i)
            if (std::string(publishers[i][0]) ==
                nh_.resolveName(output_topic_.empty() ? "/position_cmd" : output_topic_))
                for (int j = 0; j < publishers[i][1].size(); ++j)
                    if (std::string(publishers[i][1][j]) != ros::this_node::getName()) return false;
        return true;
    }
    void publisherCheck(const ros::WallTimerEvent&) {
        if (!execute_) return;
        bool ok = singlePublisher();
        std::lock_guard<std::mutex> lock(mutex_);
        publisher_ok_ = ok;
        if (!ok && active_) halt("command publisher conflict/master unavailable");
    }
    bool inputsOK(Clock::time_point now, std::string* reason) const {
        if (!latched_fault_.empty()) { *reason=latched_fault_; return false; }
        if (recovery_enabled_ && (imu_received_==Clock::time_point{} ||
            seconds(now,imu_received_)>sensor_timeout_ || imu_stamp_.isZero() ||
            (ros::Time::now()-imu_stamp_).toSec() < -0.05 ||
            (ros::Time::now()-imu_stamp_).toSec() > sensor_timeout_)) {
            *reason="missing/stale monitored IMU"; return false;
        }
        if (execute_ && (!rc_allowed_ || seconds(now,rc_received_)>0.5 ||
            seconds(now,extended_received_)>2.0 ||
            extended_.landed_state!=mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)) {
            *reason="execution requires fresh neutral RC command mode and IN_AIR"; return false;
        }
        if (!have_odom_ || !have_depth_ || seconds(now, odom_received_) > sensor_timeout_ ||
            seconds(now, depth_received_) > sensor_timeout_) {
            *reason = "missing/stale odometry or depth";
            return false;
        }
        if (odom_.header.frame_id != "world" ||
            std::abs((odom_.header.stamp - depth_stamp_).toSec()) > skew_) {
            *reason = "world frame or depth/odometry time skew invalid";
            return false;
        }
        for (auto stamp : {odom_.header.stamp, depth_stamp_}) {
            double age = (ros::Time::now() - stamp).toSec();
            if (stamp.isZero() || age < -0.05 || age > sensor_timeout_) {
                *reason = "sensor timestamp invalid";
                return false;
            }
        }
        Eigen::Quaterniond q(
            odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x,
            odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z);
        if (!position().allFinite() || !velocity().allFinite() || !q.coeffs().allFinite() ||
            std::abs(q.norm() - 1) > 0.05) {
            *reason = "invalid odometry";
            return false;
        }
        if (execute_ && (!publisher_ok_ || seconds(now, state_received_) > state_timeout_ ||
                         !state_.connected || !state_.armed || state_.mode != "OFFBOARD")) {
            *reason =
                "execution requires unique publisher and fresh connected/armed/OFFBOARD state";
            return false;
        }
        return true;
    }
    Eigen::Vector3d position() const {
        return {odom_.pose.pose.position.x, odom_.pose.pose.position.y, odom_.pose.pose.position.z};
    }
    Eigen::Vector3d velocity() const {
        return {odom_.twist.twist.linear.x, odom_.twist.twist.linear.y, odom_.twist.twist.linear.z};
    }
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        odom_          = *msg;
        have_odom_     = true;
        odom_received_ = Clock::now();
    }
    void stateCallback(const mavros_msgs::State::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        state_          = *msg;
        state_received_ = Clock::now();
        if (execute_ && active_ && (!msg->connected || !msg->armed || msg->mode!="OFFBOARD"))
            halt("flight state lost; explicit new goal required");
    }
    void stopCallback(const std_msgs::Empty::ConstPtr&) {
        std::lock_guard<std::mutex> lock(mutex_);
        halt("user request");
    }
    double steadyNow() const {
        return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();
    }
    double measuredYaw() const {
        const auto& q=odom_.pose.pose.orientation;
        return std::atan2(2*(q.w*q.z+q.x*q.y),1-2*(q.y*q.y+q.z*q.z));
    }
    void faultCallback(const std_msgs::Bool::ConstPtr& msg) {
        if (!msg->data) return;
        std::lock_guard<std::mutex> lock(mutex_);
        latched_fault_="localization failure latched; restart after diagnosis";
        if (active_) halt(latched_fault_);
    }
    void imuWatch(const sensor_msgs::Imu::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        // KF output mixes measurement stamps; monitor upstream IMU instead.
        if (!imu_stamp_.isZero() && (imu_stamp_-msg->header.stamp).toSec()>0.001) {
            latched_fault_="IMU timestamp moved backwards; restart after diagnosis";
            if (active_) halt(latched_fault_);
        }
        if (msg->header.stamp>imu_stamp_) imu_stamp_=msg->header.stamp;
        imu_received_=Clock::now();
    }
    void rcCallback(const mavros_msgs::RCIn::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        rc_received_=Clock::now();
        rc_allowed_=msg->channels.size()>=8;
        if (rc_allowed_) {
            rc_allowed_=msg->channels[4]>1750 && msg->channels[5]>1750;
            for (int i=0;i<4;++i)
                rc_allowed_=rc_allowed_ && std::abs(int(msg->channels[i])-1500)<=125;
        }
        if (execute_ && active_ && !rc_allowed_) halt("RC takeover; explicit new goal required");
    }
    void extendedCallback(const mavros_msgs::ExtendedState::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        extended_=*msg; extended_received_=Clock::now();
        if (execute_ && active_ && msg->landed_state!=mavros_msgs::ExtendedState::LANDED_STATE_IN_AIR)
            halt("vehicle is not IN_AIR");
    }
    void takeoffLandCallback(const quadrotor_msgs::TakeoffLand::ConstPtr&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_) halt("takeoff/landing request; task cancelled");
    }
    bool startTask(std::string* detail) {  // Caller holds mutex_.
        if (active_) { *detail="task busy; stop it explicitly before a new goal"; return false; }
        auto now=Clock::now();
        if (!have_goal_) { *detail="no goal"; return false; }
        if (!inputsOK(now,detail)) return false;
        const auto p=position(); const double distance=(goal_-p).norm();
        std::ostringstream info;
        info << "position=[" << p.transpose() << "] goal=[" << goal_.transpose()
             << "] distance=" << distance << " allowed=[" << goal_min_ << "," << goal_max_
             << "] speed=" << velocity().norm();
        if (distance<goal_min_ || distance>goal_max_ || p.z()<config_.min_height ||
            p.z()>config_.max_height || goal_.z()<config_.min_height ||
            goal_.z()>config_.max_height || std::abs(goal_.z()-p.z())>config_.height_band ||
            velocity().norm()>hover_speed_) {
            *detail="trigger limits rejected: "+info.str(); return false;
        }
        origin_=p; active_=true; have_plan_=false; start_time_=now;
        arrival_since_=Clock::time_point{}; ++generation_;
        recovery_.start(distance); resume_requested_=false;
        last_yaw_=measuredYaw(); last_yaw_rate_=0; last_control_=now;
        geometry_msgs::PoseStamped accepted;
        accepted.header.stamp=ros::Time::now(); accepted.header.frame_id="world";
        accepted.pose.position.x=goal_.x(); accepted.pose.position.y=goal_.y();
        accepted.pose.position.z=goal_.z(); accepted.pose.orientation.w=1;
        accepted_goal_.publish(accepted);
        *detail="ACTIVE: "+info.str()+"; waiting for a fresh plan";
        publishStatus(*detail); return true;
    }
    bool startGoal(StartGoal::Request& req, StartGoal::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        Eigen::Vector3d g(req.goal.pose.position.x,req.goal.pose.position.y,req.goal.pose.position.z);
        const double age=(ros::Time::now()-req.goal.header.stamp).toSec();
        if (active_ || req.goal.header.frame_id!="world" || !g.allFinite() ||
            req.goal.header.stamp.isZero() || age < -0.05 || age>1.0) {
            res.success=false; res.message="busy or invalid/stale world goal; nothing started";
            return true;
        }
        // Selection and activation are atomic: no stale goal/trigger race.
        goal_=g; have_goal_=true; res.success=startTask(&res.message);
        if (!res.success) { have_goal_=false; ROS_WARN_STREAM(res.message); }
        return true;
    }
    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_) { ROS_WARN("Stop current task before setting another goal"); return; }
        Eigen::Vector3d g(msg->pose.position.x,msg->pose.position.y,msg->pose.position.z);
        if (msg->header.frame_id!="world" || !g.allFinite()) {
            ROS_WARN("Goal rejected: finite world-frame position required"); return;
        }
        goal_=g; have_goal_=true;
        publishStatus("GOAL_SET: waiting for explicit trajectory trigger");
    }
    void triggerCallback(const geometry_msgs::PoseStamped::ConstPtr&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (active_) return; // Triggers cannot bypass recovery readiness.
        std::string reason;
        if (!startTask(&reason)) ROS_WARN_STREAM("Trigger rejected: " << reason);
    }
    bool resume(std_srvs::Trigger::Request&, std_srvs::Trigger::Response& res) {
        std::lock_guard<std::mutex> lock(mutex_);
        std::string reason;
        if (!active_ || !recovery_.freshReady(steadyNow()) || !inputsOK(Clock::now(),&reason)) {
            res.success=false; res.message="not fresh READY or inputs invalid: "+reason; return true;
        }
        resume_requested_=true; resume_request_time_=Clock::now();
        res.success=true; res.message="confirmation received; a NEW valid plan is still required";
        return true;
    }
    bool recoveryUpdate() {  // Caller holds mutex_.
        if (!recovery_.recovering()) return active_;
        const auto p=position();
        if (p.z()<config_.min_height || p.z()>config_.max_height ||
            std::abs(p.z()-origin_.z())>config_.height_band ||
            (p-origin_).head<2>().norm()>config_.test_radius) {
            halt("recovery left original task bounds"); return false;
        }
        const auto old=recovery_.phase; const auto revision=recovery_.revision;
        recovery_.update(steadyNow(),{{p.x(),p.y(),p.z()}},velocity().norm());
        if (recovery_.phase==RecoveryGate::Phase::STOPPED) { halt(recovery_.failure); return false; }
        if (revision!=recovery_.revision) { ++generation_; resume_requested_=false; }
        if (old!=recovery_.phase)
            publishStatus(recovery_.phase==RecoveryGate::Phase::WAIT_HOVER ?
                "WAIT_HOVER: hover stability lost; commands remain released" :
                "HOVER_REPLAN: stable by timeout/speed/position; evaluating without commands");
        return true;
    }
    void corridorFailure(const std::string& reason) {
        if (!recovery_enabled_) { halt(reason); return; }
        if (!recovery_.begin(steadyNow(),(goal_-position()).norm())) {
            halt(recovery_.failure); return;
        }
        have_plan_=false; resume_requested_=false; ++generation_;
        publishStatus("WAIT_HOVER: "+reason+"; commands released; attempt="+
                      std::to_string(recovery_.attempts));
    }
    bool prepareDepth(const sensor_msgs::Image& msg, Depth* output) const {
        if (msg.width != unsigned(width_) || msg.height != unsigned(height_) || msg.is_bigendian)
            return false;
        int bytes = msg.encoding == "16UC1" ? 2 : msg.encoding == "32FC1" ? 4 : 0;
        if (!bytes || msg.step < msg.width * bytes ||
            msg.data.size() < size_t(msg.step) * msg.height)
            return false;
        cv::Mat depth(height_, width_, CV_32F);
        size_t invalid = 0;
        for (int y = 0; y < height_; ++y)
            for (int x = 0; x < width_; ++x) {
                float d;
                const uint8_t* src = msg.data.data() + size_t(y) * msg.step + x * bytes;
                if (bytes == 2) {
                    uint16_t mm;
                    std::memcpy(&mm, src, 2);
                    d = mm / 1000.f;
                } else
                    std::memcpy(&d, src, 4);
                if (!std::isfinite(d) || d < min_depth_) {
                    ++invalid;
                    d = 0;
                }
                depth.at<float>(y, x) = d;
            }
        if (double(invalid) / (size_t(width_) * height_) > max_invalid_) return false;
        cv::Mat resized, mask, normalized, u8, filled;
        cv::resize(depth, resized, cv::Size(160, 96), 0, 0, cv::INTER_NEAREST);
        mask = resized < min_depth_;
        cv::min(resized, max_depth_, normalized);
        normalized /= max_depth_;
        // Match Python np.uint8(depth*255) truncation, not OpenCV rounding.
        u8.create(96, 160, CV_8U);
        for (int y = 0; y < 96; ++y)
            for (int x = 0; x < 160; ++x)
                u8.at<uint8_t>(y, x) = static_cast<uint8_t>(normalized.at<float>(y, x) * 255.f);
        cv::inpaint(u8, mask, filled, 1, cv::INPAINT_NS);
        for (int y = 0; y < 96; ++y)
            for (int x = 0; x < 160; ++x) (*output)[y * 160 + x] = filled.at<uint8_t>(y, x) / 255.f;
        return true;
    }
    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        auto started = Clock::now();
        PlanningState snapshot;
        uint64_t generation;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!depth_stamp_.isZero() && (depth_stamp_-msg->header.stamp).toSec()>0.001) {
                latched_fault_="depth timestamp moved backwards; restart after diagnosis";
                if (active_) halt(latched_fault_);
                return;
            }
            if (!depth_stamp_.isZero() && msg->header.stamp<=depth_stamp_) return;
            have_depth_     = true;
            depth_stamp_    = msg->header.stamp;
            depth_received_ = started;
            if (!active_ || seconds(started, last_plan_attempt_) < 1 / plan_hz_) return;
            last_plan_attempt_ = started;
            std::string reason;
            if (!inputsOK(started, &reason)) {
                halt(reason);
                return;
            }
            if (!recoveryUpdate()) return;
            if (recovery_.phase==RecoveryGate::Phase::WAIT_HOVER) return;
            generation           = generation_;
            snapshot.head.row(0) = position().transpose();
            snapshot.head.row(1) = velocity().transpose();
            // The acceleration reference is retained as in Python; zero before first plan.
            if (have_plan_ && !recovery_.recovering()) {
                double t = seconds(started, plan_epoch_);
                if (t >= plan_.trajectory.duration() || t > plan_timeout_) {
                    halt("old plan expired before replanning");
                    return;
                }
                snapshot.head.row(2) = plan_.trajectory.evaluate(t, 2).transpose();
                if (reference_) {
                    snapshot.head.row(0) = plan_.trajectory.evaluate(t, 0).transpose();
                    snapshot.head.row(1) = plan_.trajectory.evaluate(t, 1).transpose();
                }
            }
            Eigen::Quaterniond q(
                odom_.pose.pose.orientation.w, odom_.pose.pose.orientation.x,
                odom_.pose.pose.orientation.y, odom_.pose.pose.orientation.z);
            snapshot.rotation_wc = q.normalized().toRotationMatrix() * rotation_bc_;
            snapshot.goal        = goal_;
            snapshot.origin      = origin_;
        }
        Depth depth;
        NetworkOutput out;
        Plan candidate;
        std::string error;
        bool ok = prepareDepth(*msg, &depth);
        if (!ok) error = "invalid depth dimensions/encoding/values";
        if (ok) {
            ok = engine_.infer(depth, planner_->observation(snapshot), &out);
            if (!ok) error = "TensorRT inference failed";
        }
        if (ok) ok = planner_->plan(snapshot, out, &candidate, &error);
        auto finished = Clock::now();
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (!active_ || generation != generation_)
                return;  // Ignore work completed after a stop/new task.
            if (seconds(finished, started) > plan_timeout_) {
                halt("planning latency exceeded budget");
                return;
            }
            const bool corridor_rejected=error.rfind("no candidate passes predicted corridor threshold",0)==0;
            if (!ok && !corridor_rejected) { halt(error); return; }
            std::string input_error;
            if (!inputsOK(finished, &input_error)) { halt(input_error); return; }
            if (!recoveryUpdate() || generation!=generation_) return;
            if (recovery_.recovering()) {
                const auto old=recovery_.phase;
                const auto revision=recovery_.revision;
                recovery_.candidate(steadyNow(),msg->header.stamp.toSec(),ok);
                if (revision!=recovery_.revision) resume_requested_=false;
                if (!ok) {
                    ROS_WARN_STREAM_THROTTLE(1.0, "HOVER_REPLAN: " << error);
                    resume_requested_=false;
                    if (old==RecoveryGate::Phase::READY)
                        publishStatus("HOVER_REPLAN: candidate failed; confirmation revoked; "+error);
                    return;
                }
                publishPath(candidate);
                ROS_INFO_THROTTLE(1.0, "Recovery passing frames=%d, predicted mu=%.4f",
                                  recovery_.good_frames, candidate.corridor_mu);
                if (recovery_.phase==RecoveryGate::Phase::READY && old!=RecoveryGate::Phase::READY)
                    publishStatus("READY: consecutive fresh plans passed; auto_resume="+
                                  std::to_string(auto_resume_));
                const bool confirmed=resume_requested_ && seconds(finished,resume_request_time_)<=1.0;
                if (!recovery_.freshReady(steadyNow()) || (!auto_resume_ && !confirmed)) return;
                const auto planned_now=candidate.trajectory.evaluate(seconds(finished,started),0);
                if ((planned_now-position()).norm()>std::min(0.15,tracking_error_)) {
                    recovery_.candidate(steadyNow(),msg->header.stamp.toSec(),false);
                    resume_requested_=false; return;
                }
                recovery_.resumed(); resume_requested_=false;
                last_yaw_=measuredYaw(); last_yaw_rate_=0; last_control_=finished;
                publishStatus("RESUME: fresh plan from measured hover state; original goal/bounds retained");
            } else if (!ok) { corridorFailure(error); return; }
            if (!inputsOK(finished, &error)) {
                halt(error);
                return;
            }
            plan_       = candidate;
            plan_epoch_ = started;
            have_plan_  = true;
        }
        ROS_INFO_THROTTLE(
            1., "MINCO plan %.2f ms, action=%d, duration=%.3f s, score=%.4f, predicted mu=%.4f",
            seconds(finished, started) * 1000, candidate.action, candidate.trajectory.duration(),
            candidate.score, candidate.corridor_mu);
        publishPath(candidate);
    }
    void publishPath(const Plan& candidate) {
        if (visualize_ && path_.getNumSubscribers()) {
            geometry_msgs::PoseArray path;
            path.header.frame_id = "world";
            path.header.stamp    = ros::Time::now();
            for (int i = 0; i <= 100; ++i) {
                auto p = candidate.trajectory.evaluate(candidate.trajectory.duration() * i / 100.);
                geometry_msgs::Pose pose;
                pose.position.x    = p.x();
                pose.position.y    = p.y();
                pose.position.z    = p.z();
                pose.orientation.w = 1;
                path.poses.push_back(pose);
            }
            path_.publish(path);
        }
    }
    void controlCallback(const ros::WallTimerEvent&) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!active_) return;
        auto now = Clock::now();
        std::string reason;
        if (!inputsOK(now, &reason)) {
            halt(reason);
            return;
        }
        if ((position() - goal_).norm() < arrive_distance_ && velocity().norm() < arrive_speed_) {
            if (arrival_since_ == Clock::time_point{}) arrival_since_ = now;
            if (seconds(now, arrival_since_) >= arrive_hold_) {
                halt("arrived and settled");
                return;
            }
        } else
            arrival_since_ = Clock::time_point{};
        if (recovery_.recovering()) { recoveryUpdate(); return; }
        if (!have_plan_) {
            if (seconds(now, start_time_) > plan_timeout_) halt("first plan timeout");
            return;
        }
        double t = seconds(now, plan_epoch_);
        if (t > plan_timeout_ || t >= plan_.trajectory.duration()) {
            halt("plan stale/expired");
            return;
        }
        double dt = seconds(now, last_control_);
        if (dt <= 0 || dt > sensor_timeout_) {
            halt("control scheduling gap");
            return;
        }
        auto p = plan_.trajectory.evaluate(t, 0), v = plan_.trajectory.evaluate(t, 1),
             a = plan_.trajectory.evaluate(t, 2), j = plan_.trajectory.evaluate(t, 3);
        if ((p - position()).norm() > tracking_error_ || !planner_->check(p, v, a, j, origin_)) {
            halt("reference tracking or motion limit");
            return;
        }
        auto yaw      = YopoPlanner::yaw(v, goal_ - p, last_yaw_, last_yaw_rate_, dt);
        last_yaw_     = yaw.first;
        last_yaw_rate_ = yaw.second;
        last_control_ = now;
        quadrotor_msgs::PositionCommand cmd;
        cmd.header.stamp    = ros::Time::now();
        cmd.header.frame_id = "world";
        cmd.trajectory_flag = quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        cmd.position.x      = p.x();
        cmd.position.y      = p.y();
        cmd.position.z      = p.z();
        cmd.velocity.x      = v.x();
        cmd.velocity.y      = v.y();
        cmd.velocity.z      = v.z();
        cmd.acceleration.x  = a.x();
        cmd.acceleration.y  = a.y();
        cmd.acceleration.z  = a.z();
        cmd.jerk.x          = j.x();
        cmd.jerk.y          = j.y();
        cmd.jerk.z          = j.z();
        cmd.yaw             = yaw.first;
        cmd.yaw_dot         = yaw.second;
        commands_.publish(cmd);
    }
    ros::NodeHandle nh_, private_, depth_nh_;
    ros::CallbackQueue depth_queue_;
    std::unique_ptr<ros::AsyncSpinner> depth_spinner_;
    ros::Publisher commands_, status_, path_, recovery_status_, accepted_goal_;
    ros::ServiceServer start_service_, resume_service_;
    ros::Subscriber fault_sub_, vins_fault_sub_, imu_watch_sub_, rc_sub_, extended_sub_, takeoff_land_sub_;
    RecoveryGate recovery_;
    bool recovery_enabled_=true, auto_resume_=false, resume_requested_=false, rc_allowed_=false;
    std::string latched_fault_;
    ros::Time imu_stamp_;
    Clock::time_point imu_received_{}, rc_received_{}, extended_received_{}, resume_request_time_{};
    mavros_msgs::ExtendedState extended_;
    ros::Subscriber odom_sub_, depth_sub_, state_sub_, goal_sub_, trigger_sub_, stop_sub_;
    ros::WallTimer timer_, publisher_timer_;
    std::mutex mutex_;
    PlannerConfig config_;
    std::unique_ptr<YopoPlanner> planner_;
    YopoEngine engine_;
    Eigen::Matrix3d rotation_bc_;
    Plan plan_;
    nav_msgs::Odometry odom_;
    mavros_msgs::State state_;
    Eigen::Vector3d goal_ = Eigen::Vector3d::Zero(), origin_ = Eigen::Vector3d::Zero();
    bool execute_ = false, reference_ = false, visualize_ = false, active_ = false,
         have_plan_ = false, have_goal_ = false, have_odom_ = false, have_depth_ = false,
         publisher_ok_   = true;
    uint64_t generation_ = 0;
    ros::Time depth_stamp_;
    Clock::time_point odom_received_{}, depth_received_{}, state_received_{}, plan_epoch_{},
        start_time_{}, arrival_since_{}, last_plan_attempt_{}, last_control_{};
    std::string engine_file_, output_topic_;
    int width_ = 640, height_ = 480;
    double plan_hz_, ctrl_dt_, sensor_timeout_, state_timeout_, skew_, plan_timeout_,
        tracking_error_, goal_min_, goal_max_, arrive_distance_, arrive_speed_, arrive_hold_,
        hover_speed_;
    double min_depth_, max_depth_, max_invalid_, last_yaw_ = 0, last_yaw_rate_ = 0;
};
}  // namespace yopo_minco_planner
int main(int argc, char** argv) {
    ros::init(argc, argv, "yopo_minco_planner");
    try {
        yopo_minco_planner::YopoMincoNode node;
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM(e.what());
        return 1;
    }
    return 0;
}
