#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <cmath>
#include <cstring>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#ifdef YOPO_HAVE_OPENCV_PHOTO
#include <opencv2/photo.hpp>
#endif
#include "yopo_planner/YopoLog.h"
#include "yopo_planner/dynamic_target_safety.h"
#include "yopo_planner/yopo_engine.h"
#include "yopo_planner/yopo_planner.h"
#include <px4ctrl/FsmStatus.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>
#include <std_msgs/String.h>
#include <vector>

namespace yopo_planner {

class YopoPlannerNode {
  public:
    YopoPlannerNode() : nh_(), pnh_("~") {
        YopoParams params;
        // param_name param_variable default_value
        pnh_.param("pitch_angle_deg", params.pitch_angle_deg, params.pitch_angle_deg);
        pnh_.param("plan_from_reference", params.plan_from_reference, params.plan_from_reference);
        pnh_.param(
            "require_camera_extrinsic", params.require_camera_extrinsic,
            params.require_camera_extrinsic);
        pnh_.param("velocity", params.velocity, params.velocity);
        pnh_.param("radio_range", params.radio_range, params.radio_range);
        pnh_.param("ctrl_dt", params.ctrl_dt, params.ctrl_dt);
        pnh_.param("arrive_distance", params.arrive_distance, params.arrive_distance);
        pnh_.param("terminal_approach_enabled", params.terminal_approach_enabled,
                   params.terminal_approach_enabled);
        pnh_.param("terminal_brake_accel", params.terminal_brake_accel,
                   params.terminal_brake_accel);
        pnh_.param("terminal_brake_margin", params.terminal_brake_margin,
                   params.terminal_brake_margin);
        pnh_.param("terminal_position_tolerance", params.terminal_position_tolerance,
                   params.terminal_position_tolerance);
        pnh_.param("terminal_speed_tolerance", params.terminal_speed_tolerance,
                   params.terminal_speed_tolerance);
        pnh_.param("terminal_stable_time", params.terminal_stable_time,
                   params.terminal_stable_time);
        pnh_.param("arrival_yaw_rate", params.arrival_yaw_rate, params.arrival_yaw_rate);
        pnh_.param(
            "arrival_yaw_min_distance", params.arrival_yaw_min_distance,
            params.arrival_yaw_min_distance);
        pnh_.param(
            "dynamic_target_yaw_rate", params.dynamic_target_yaw_rate,
            params.dynamic_target_yaw_rate);
        pnh_.param(
            "dynamic_target_yaw_accel", params.dynamic_target_yaw_accel,
            params.dynamic_target_yaw_accel);
        pnh_.param(
            "dynamic_target_yaw_deadband_deg", params.dynamic_target_yaw_deadband_deg,
            params.dynamic_target_yaw_deadband_deg);
        pnh_.param(
            "dynamic_target_standoff_distance", params.dynamic_target_standoff_distance,
            params.dynamic_target_standoff_distance);
        pnh_.param(
            "dynamic_target_standoff_hysteresis", params.dynamic_target_standoff_hysteresis,
            params.dynamic_target_standoff_hysteresis);
        pnh_.param(
            "dynamic_target_min_speed_scale", params.dynamic_target_min_speed_scale,
            params.dynamic_target_min_speed_scale);
        pnh_.param(
            "dynamic_target_coast_slowdown_time", params.dynamic_target_coast_slowdown_time,
            params.dynamic_target_coast_slowdown_time);
        pnh_.param(
            "dynamic_target_search_enabled", params.dynamic_target_search_enabled,
            params.dynamic_target_search_enabled);
        pnh_.param(
            "dynamic_target_search_yaw_amplitude_deg",
            params.dynamic_target_search_yaw_amplitude_deg,
            params.dynamic_target_search_yaw_amplitude_deg);
        pnh_.param(
            "dynamic_target_search_yaw_rate", params.dynamic_target_search_yaw_rate,
            params.dynamic_target_search_yaw_rate);
        pnh_.param(
            "goal_yaw_alignment_enabled", params.goal_yaw_alignment_enabled,
            params.goal_yaw_alignment_enabled);
        pnh_.param(
            "goal_yaw_align_enter_deg", params.goal_yaw_align_enter_deg,
            params.goal_yaw_align_enter_deg);
        pnh_.param("goal_yaw_align_rate", params.goal_yaw_align_rate, params.goal_yaw_align_rate);
        pnh_.param(
            "goal_yaw_brake_speed_tolerance", params.goal_yaw_brake_speed_tolerance,
            params.goal_yaw_brake_speed_tolerance);
        pnh_.param(
            "goal_yaw_brake_stable_time", params.goal_yaw_brake_stable_time,
            params.goal_yaw_brake_stable_time);
        pnh_.param(
            "goal_yaw_align_min_distance", params.goal_yaw_align_min_distance,
            params.goal_yaw_align_min_distance);
        pnh_.param(
            "sensor_min_depth", params.sensor_min_depth, params.sensor_min_depth);
        pnh_.param(
            "sensor_reliable_max_depth", params.sensor_reliable_max_depth,
            params.sensor_reliable_max_depth);
        pnh_.param("model_depth_max", params.model_depth_max, params.model_depth_max);
        if (!(params.sensor_min_depth > 0.0) ||
            !(params.sensor_reliable_max_depth > params.sensor_min_depth) ||
            !(params.model_depth_max >= params.sensor_reliable_max_depth)) {
            ROS_FATAL(
                "Invalid depth ranges: require 0 < sensor_min_depth < "
                "sensor_reliable_max_depth <= model_depth_max, got %.3f, %.3f, %.3f m.",
                params.sensor_min_depth, params.sensor_reliable_max_depth,
                params.model_depth_max);
            ros::shutdown();
            return;
        }
        if (params.goal_yaw_alignment_enabled &&
            (!(params.goal_yaw_align_enter_deg > 0.0) ||
             !(params.goal_yaw_align_enter_deg < 180.0) ||
             !(params.goal_yaw_align_rate > 0.0) ||
             !(params.goal_yaw_brake_speed_tolerance >= 0.0) ||
             !(params.goal_yaw_brake_stable_time >= 0.0) ||
             !(params.goal_yaw_align_min_distance >= 0.0))) {
            ROS_FATAL(
                "Invalid goal yaw alignment parameters: require 0 < enter < 180 deg, "
                "positive yaw rate, and non-negative speed/time/distance thresholds.");
            ros::shutdown();
            return;
        }
        if (!(params.arrive_distance > 0.0) ||
            !(params.terminal_brake_accel > 0.0) ||
            !(params.terminal_brake_margin >= 0.0) ||
            !(params.terminal_position_tolerance > 0.0) ||
            !(params.terminal_speed_tolerance >= 0.0) ||
            !(params.terminal_stable_time >= 0.0)) {
            ROS_FATAL("Invalid YOPO terminal approach parameters.");
            ros::shutdown();
            return;
        }
        if (!(params.dynamic_target_yaw_rate > 0.0) ||
            !(params.dynamic_target_yaw_accel > 0.0) ||
            !(params.dynamic_target_yaw_deadband_deg >= 0.0) ||
            !(params.dynamic_target_standoff_distance >= 0.0) ||
            !(params.dynamic_target_standoff_hysteresis >= 0.0) ||
            !(params.dynamic_target_min_speed_scale > 0.0) ||
            !(params.dynamic_target_min_speed_scale <= 1.0) ||
            !(params.dynamic_target_coast_slowdown_time > 0.0) ||
            !(params.dynamic_target_search_yaw_amplitude_deg >= 0.0) ||
            !(params.dynamic_target_search_yaw_amplitude_deg < 90.0) ||
            !(params.dynamic_target_search_yaw_rate > 0.0)) {
            ROS_FATAL("Invalid YOPO dynamic-target tracking parameters.");
            ros::shutdown();
            return;
        }
        pnh_.param("verbose", verbose_, false);
        pnh_.param("visualize", visualize_, true);
        pnh_.param("rotate_depth_180", rotate_depth_180_, false);
        pnh_.param("wait_for_traj_start_trigger", wait_for_traj_start_trigger_, false);
        pnh_.param("require_explicit_goal", require_explicit_goal_, false);
        pnh_.param("pause_with_fsm", pause_with_fsm_, false);
        pnh_.param("fsm_status_timeout", fsm_status_timeout_, 0.5);
        if (pause_with_fsm_ && fsm_status_timeout_ <= 0.0) {
            ROS_FATAL("fsm_status_timeout must be positive when pause_with_fsm is enabled.");
            ros::shutdown();
            return;
        }
        pnh_.param("depth_replan_rate", depth_replan_rate_, depth_replan_rate_);
        if (depth_replan_rate_ < 0.0) depth_replan_rate_ = 0.0;
        pnh_.param("target_mask_enabled", target_mask_enabled_, true);
        pnh_.param("target_mask_timeout", target_mask_timeout_, 0.25);
        pnh_.param("target_mask_inpaint_radius", target_mask_inpaint_radius_, 3.0);
        if (target_mask_enabled_ &&
            (!(target_mask_timeout_ > 0.0) || !(target_mask_inpaint_radius_ > 0.0))) {
            ROS_FATAL("target mask timeout and inpaint radius must be positive.");
            ros::shutdown();
            return;
        }

        double goal_x = 50.0;
        double goal_y = 0.0;
        double goal_z = 2.0;
        pnh_.param("goal_x", goal_x, goal_x);
        pnh_.param("goal_y", goal_y, goal_y);
        pnh_.param("goal_z", goal_z, goal_z);

        planner_.reset(new YopoPlanner(params));
        if (!require_explicit_goal_) {
            planner_->setGoal(Eigen::Vector3d(goal_x, goal_y, goal_z));
            goal_received_ = true;
        }
        ROS_INFO(
            "YOPO depth preprocessing: D455 valid range [%.2f, %.2f] m, "
            "model normalization depth %.2f m, rotate_depth_180=%s.",
            params.sensor_min_depth, params.sensor_reliable_max_depth,
            params.model_depth_max, rotate_depth_180_ ? "true" : "false");

        YopoEngine::Config engine_config;
        pnh_.param<std::string>("onnx_file", engine_config.onnx_file, "");
        pnh_.param<std::string>("engine_file", engine_config.engine_file, "");
        pnh_.param("fp16", engine_config.fp16, true);
        if (engine_config.onnx_file.empty() || engine_config.engine_file.empty()) {
            ROS_FATAL("onnx_file and engine_file params are required");
            ros::shutdown();
            return;
        }

        engine_.reset(new YopoEngine(engine_config));
        if (!engine_->build()) {
            ROS_FATAL("Failed to initialize YOPO TensorRT engine");
            ros::shutdown();
            return;
        }

        std::string odom_topic       = "/kf_fusion/kf_imu_odom";
        std::string depth_topic      = "/depth_image";
        std::string ctrl_topic       = "/so3_control/pos_cmd";
        std::string goal_topic       = "/planning/goal";
        std::string traj_start_topic = "/traj_start_trigger";
        std::string extrinsic_topic  = "/vins_fusion/extrinsic";
        std::string fsm_status_topic = "/offboard_fsm/status";
        std::string dynamic_target_topic = "/yolo_trt/tracked_target";
        std::string dynamic_target_status_topic = "/yolo_trt/target_status";
        std::string target_mask_topic = "/yolo_trt/tracked_target_mask";
        std::string camera_extrinsic_config;
        std::string camera_extrinsic_key = "body_T_cam0";
        bool use_config_camera_extrinsic = false;
        pnh_.param<std::string>("odom_topic", odom_topic, odom_topic);
        pnh_.param<std::string>("depth_topic", depth_topic, depth_topic);
        pnh_.param<std::string>("ctrl_topic", ctrl_topic, ctrl_topic);
        pnh_.param<std::string>("goal_topic", goal_topic, goal_topic);
        pnh_.param<std::string>("traj_start_topic", traj_start_topic, traj_start_topic);
        pnh_.param<std::string>("extrinsic_topic", extrinsic_topic, extrinsic_topic);
        pnh_.param<std::string>("fsm_status_topic", fsm_status_topic, fsm_status_topic);
        pnh_.param<std::string>(
            "dynamic_target_topic", dynamic_target_topic, dynamic_target_topic);
        pnh_.param<std::string>(
            "dynamic_target_status_topic", dynamic_target_status_topic,
            dynamic_target_status_topic);
        pnh_.param<std::string>("target_mask_topic", target_mask_topic, target_mask_topic);
        pnh_.param("dynamic_target_timeout", dynamic_target_timeout_, 0.8);
        pnh_.param(
            "dynamic_target_max_speed", dynamic_target_safety_config_.max_speed, 4.0);
        pnh_.param(
            "dynamic_target_max_range", dynamic_target_safety_config_.max_range, 18.0);
        pnh_.param(
            "dynamic_target_max_position_variance",
            dynamic_target_safety_config_.max_position_variance, 0.50);
        if (dynamic_target_timeout_ <= 0.0) {
            ROS_FATAL("dynamic_target_timeout must be positive.");
            ros::shutdown();
            return;
        }
        pnh_.param<std::string>(
            "camera_extrinsic_config", camera_extrinsic_config, camera_extrinsic_config);
        pnh_.param<std::string>("camera_extrinsic_key", camera_extrinsic_key, camera_extrinsic_key);
        pnh_.param(
            "use_config_camera_extrinsic", use_config_camera_extrinsic,
            use_config_camera_extrinsic);
        Eigen::Matrix3d rotation_body_optical;
        Eigen::Vector3d translation_body_optical = Eigen::Vector3d::Zero();
        const bool use_parameter_camera_extrinsic =
            pnh_.hasParam("camera_extrinsic/rotation_body_depth");
        if (use_parameter_camera_extrinsic) {
            if (!loadCameraExtrinsicFromParams(&rotation_body_optical, &translation_body_optical)) {
                ROS_FATAL("Failed to load YOPO depth-camera extrinsic from ROS parameters.");
                ros::shutdown();
                return;
            }
            setOpticalToBodyExtrinsic(rotation_body_optical, translation_body_optical);
            extrinsic_ready_logged_ = true;
            ROS_INFO(
                "YOPO loaded Depth-optical-to-body extrinsic from private parameters; "
                "translation=[%.6f %.6f %.6f] m.",
                translation_body_optical.x(), translation_body_optical.y(),
                translation_body_optical.z());
        } else if (use_config_camera_extrinsic) {
            if (!loadCameraExtrinsicFromConfig(
                    camera_extrinsic_config, camera_extrinsic_key, &rotation_body_optical)) {
                ROS_FATAL("Failed to load YOPO camera extrinsic from config.");
                ros::shutdown();
                return;
            }
            setOpticalToBodyExtrinsic(rotation_body_optical, Eigen::Vector3d::Zero());
            extrinsic_ready_logged_ = true;
            ROS_INFO(
                "YOPO loaded camera optical-to-body extrinsic from %s:%s",
                camera_extrinsic_config.c_str(), camera_extrinsic_key.c_str());
        }

        control_enabled_ = !wait_for_traj_start_trigger_;

        ctrl_pub_      = nh_.advertise<quadrotor_msgs::PositionCommand>(ctrl_topic, 1);
        best_traj_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/yopo_net/best_traj_visual", 1);
        all_trajs_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/yopo_net/trajs_visual", 1);
        lattice_traj_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/yopo_net/lattice_trajs_visual", 1);
        log_pub_ = nh_.advertise<yopo_planner::YopoLog>("/yopo_log/log", 10);
        preprocessed_depth_pub_ =
            nh_.advertise<sensor_msgs::Image>("/yopo_net/preprocessed_depth", 1);
        preprocessed_depth_float_pub_ =
            nh_.advertise<sensor_msgs::Image>("/yopo_net/preprocessed_depth_float", 1);

        odom_sub_ = nh_.subscribe(
            odom_topic, 1, &YopoPlannerNode::odomCallback, this,
            ros::TransportHints().tcpNoDelay());
        depth_sub_ = nh_.subscribe(
            depth_topic, 1, &YopoPlannerNode::depthCallback, this,
            ros::TransportHints().tcpNoDelay());
        goal_sub_ = nh_.subscribe(goal_topic, 1, &YopoPlannerNode::goalCallback, this);
        dynamic_target_sub_ = nh_.subscribe(
            dynamic_target_topic, 1, &YopoPlannerNode::dynamicTargetCallback, this,
            ros::TransportHints().tcpNoDelay());
        dynamic_target_status_sub_ = nh_.subscribe(
            dynamic_target_status_topic, 5,
            &YopoPlannerNode::dynamicTargetStatusCallback, this);
        if (target_mask_enabled_) {
            target_mask_sub_ = nh_.subscribe(
                target_mask_topic, 2, &YopoPlannerNode::targetMaskCallback, this,
                ros::TransportHints().tcpNoDelay());
        }
        traj_start_sub_ =
            nh_.subscribe(traj_start_topic, 1, &YopoPlannerNode::trajStartCallback, this);
        if (pause_with_fsm_) {
            fsm_status_sub_ = nh_.subscribe(
                fsm_status_topic, 5, &YopoPlannerNode::fsmStatusCallback, this);
        }
        if (!use_parameter_camera_extrinsic && !use_config_camera_extrinsic) {
            extrinsic_sub_ = nh_.subscribe(
                extrinsic_topic, 1, &YopoPlannerNode::extrinsicCallback, this,
                ros::TransportHints().tcpNoDelay());
        }
        ctrl_timer_ = nh_.createTimer(
            ros::Duration(planner_->params().ctrl_dt), &YopoPlannerNode::controlTimer, this);

        warmUp();
        if (wait_for_traj_start_trigger_) {
            ROS_INFO(
                "YOPO waiting for %s before publishing control commands.",
                traj_start_topic.c_str());
        }
        if (require_explicit_goal_) {
            ROS_INFO("YOPO waits for an explicit goal before inference or control output.");
        }
        if (pause_with_fsm_) {
            ROS_INFO("YOPO pauses and resynchronizes its reference with the OFFBOARD FSM lifecycle.");
        }
        if (depth_replan_rate_ > 0.0) {
            ROS_INFO("YOPO depth replanning limited to %.2f Hz.", depth_replan_rate_);
        }
        ROS_INFO(
            "YOPO dynamic tracking: target=%s status=%s timeout=%.2f s, "
            "target-facing yaw rate=%.2f rad/s, coast speed floor=%.2f.",
            dynamic_target_topic.c_str(), dynamic_target_status_topic.c_str(),
            dynamic_target_timeout_, params.dynamic_target_yaw_rate,
            params.dynamic_target_min_speed_scale);
        if (target_mask_enabled_) {
            ROS_INFO(
                "YOPO target-depth masking enabled: topic=%s timeout=%.2f s radius=%.1f px.",
                target_mask_topic.c_str(), target_mask_timeout_, target_mask_inpaint_radius_);
        }
        if (params.goal_yaw_alignment_enabled) {
            ROS_INFO(
                "YOPO goal yaw alignment enabled: forward sector +/-%.1f deg, "
                "turn rate %.2f rad/s.",
                params.goal_yaw_align_enter_deg, params.goal_yaw_align_rate);
        }
        if (planner_->requiresCameraExtrinsic() && !planner_->cameraExtrinsicReady()) {
            ROS_INFO(
                "YOPO waiting for camera-to-body extrinsic from %s before publishing control "
                "commands.",
                extrinsic_topic.c_str());
        }
        ROS_INFO("YOPO planner node ready.");
    }

  private:
    using Clock = std::chrono::steady_clock;

    static double elapsedMs(const Clock::time_point& a, const Clock::time_point& b) {
        return std::chrono::duration<double, std::milli>(b - a).count();
    }

    void warmUp() {
        std::array<float, 1 * 1 * 96 * 160> depth{};
        std::array<float, 1 * 9 * 3 * 5> obs{};
        std::array<float, 1 * 9 * 3 * 5> endstate{};
        std::array<float, 1 * 3 * 5> score{};
        (void)engine_->infer(depth, obs, &endstate, &score);
    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        latest_vehicle_position_ = Eigen::Vector3d(
            msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        has_vehicle_position_ = latest_vehicle_position_.allFinite();
        planner_->updateOdometry(*msg);
        ROS_INFO_THROTTLE(
            3.0, "YOPO odom position: x=%.3f y=%.3f z=%.3f", msg->pose.pose.position.x,
            msg->pose.pose.position.y, msg->pose.pose.position.z);
        if (planner_->arrived()) {
            ROS_WARN_THROTTLE(2.0, "YOPO planner arrived near goal.");
        }
    }

    void extrinsicCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        const Eigen::Quaterniond q(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        const Eigen::Matrix3d rotation_body_optical = q.normalized().toRotationMatrix();
        const Eigen::Vector3d translation_body_optical(
            msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
        setOpticalToBodyExtrinsic(rotation_body_optical, translation_body_optical);
        if (!extrinsic_ready_logged_) {
            ROS_INFO(
                "YOPO received camera optical-to-body extrinsic and converted YOPO frame to body.");
            extrinsic_ready_logged_ = true;
        }
    }

    static Eigen::Matrix3d rotationOpticalYopo() {
        return (Eigen::Matrix3d() << 0.0, -1.0, 0.0, 0.0, 0.0, -1.0, 1.0, 0.0, 0.0).finished();
    }

    void setOpticalToBodyExtrinsic(
        const Eigen::Matrix3d& rotation_body_optical,
        const Eigen::Vector3d& translation_body_optical) {
        Eigen::Matrix3d rotation_body_input_optical = rotation_body_optical;
        if (rotate_depth_180_) {
            // Rotating the pixel image by 180 degrees creates a virtual upright
            // optical frame: x_virtual=-x_physical, y_virtual=-y_physical,
            // z_virtual=z_physical. Compose the same rotation into the calibrated
            // physical-camera extrinsic so image pixels, state input and predicted
            // trajectories continue to describe the same physical directions.
            Eigen::Matrix3d rotation_physical_input = Eigen::Matrix3d::Identity();
            rotation_physical_input(0, 0) = -1.0;
            rotation_physical_input(1, 1) = -1.0;
            rotation_body_input_optical = rotation_body_optical * rotation_physical_input;
        }
        planner_->setCameraToBodyExtrinsic(
            rotation_body_input_optical * rotationOpticalYopo());
        translation_body_optical_ = translation_body_optical;
        if (!depth_orientation_logged_) {
            ROS_INFO_STREAM(
                "YOPO effective input-optical-to-body rotation (rotate_depth_180="
                << (rotate_depth_180_ ? "true" : "false") << "):\n"
                << rotation_body_input_optical);
            depth_orientation_logged_ = true;
        }
    }

    bool loadCameraExtrinsicFromParams(
        Eigen::Matrix3d* rotation_body_optical, Eigen::Vector3d* translation_body_optical) const {
        if (!rotation_body_optical || !translation_body_optical) return false;

        std::vector<double> rotation;
        std::vector<double> translation;
        if (!pnh_.getParam("camera_extrinsic/rotation_body_depth", rotation) ||
            rotation.size() != 9) {
            ROS_ERROR("camera_extrinsic/rotation_body_depth must contain 9 numbers.");
            return false;
        }
        if (!pnh_.getParam("camera_extrinsic/translation_body_depth", translation) ||
            translation.size() != 3) {
            ROS_ERROR("camera_extrinsic/translation_body_depth must contain 3 numbers.");
            return false;
        }

        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                (*rotation_body_optical)(r, c) = rotation[3 * r + c];
            }
            (*translation_body_optical)(r) = translation[r];
        }
        const double orthogonality_error =
            (rotation_body_optical->transpose() * (*rotation_body_optical) -
             Eigen::Matrix3d::Identity())
                .norm();
        const double determinant = rotation_body_optical->determinant();
        if (!rotation_body_optical->allFinite() || !translation_body_optical->allFinite() ||
            orthogonality_error > 1e-3 || std::abs(determinant - 1.0) > 1e-3) {
            ROS_ERROR(
                "Invalid camera extrinsic rotation: det=%.9f, orthogonality error=%.9g.",
                determinant, orthogonality_error);
            return false;
        }
        return true;
    }

    bool loadCameraExtrinsicFromConfig(
        const std::string& config_file, const std::string& key,
        Eigen::Matrix3d* rotation_body_optical) const {
        if (!rotation_body_optical) return false;
        if (config_file.empty()) {
            ROS_ERROR("camera_extrinsic_config is empty.");
            return false;
        }

        cv::FileStorage fs(config_file, cv::FileStorage::READ);
        if (!fs.isOpened()) {
            ROS_ERROR("Cannot open camera extrinsic config: %s", config_file.c_str());
            return false;
        }

        cv::Mat transform;
        fs[key] >> transform;
        if (transform.empty()) {
            ROS_ERROR(
                "Cannot find camera extrinsic key '%s' in %s", key.c_str(), config_file.c_str());
            return false;
        }
        if (transform.rows < 3 || transform.cols < 3) {
            ROS_ERROR(
                "Camera extrinsic '%s' must be at least 3x3, got %dx%d.", key.c_str(),
                transform.rows, transform.cols);
            return false;
        }

        transform.convertTo(transform, CV_64F);
        for (int r = 0; r < 3; ++r) {
            for (int c = 0; c < 3; ++c) {
                (*rotation_body_optical)(r, c) = transform.at<double>(r, c);
            }
        }
        return true;
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        planner_->setGoal(
            Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
        goal_received_ = true;
        ROS_INFO(
            "YOPO new goal: %.2f %.2f %.2f", msg->pose.position.x, msg->pose.position.y,
            msg->pose.position.z);
    }

    void dynamicTargetCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        const Eigen::Vector3d position(
            msg->pose.pose.position.x, msg->pose.pose.position.y,
            msg->pose.pose.position.z);
        const Eigen::Vector3d velocity(
            msg->twist.twist.linear.x, msg->twist.twist.linear.y,
            msg->twist.twist.linear.z);
        DynamicTargetSample safety_sample;
        safety_sample.position = {{position.x(), position.y(), position.z()}};
        safety_sample.velocity = {{velocity.x(), velocity.y(), velocity.z()}};
        safety_sample.position_variance = {{
            msg->pose.covariance[0], msg->pose.covariance[7],
            msg->pose.covariance[14]}};
        safety_sample.vehicle_position = {{
            latest_vehicle_position_.x(), latest_vehicle_position_.y(),
            latest_vehicle_position_.z()}};
        safety_sample.tracker_status = tracker_status_;
        const DynamicTargetRejectReason rejection = has_vehicle_position_
            ? validateDynamicTarget(safety_sample, dynamic_target_safety_config_)
            : DynamicTargetRejectReason::kNonFinite;
        if (rejection != DynamicTargetRejectReason::kNone) {
            ROS_ERROR_THROTTLE(
                0.5, "YOPO rejected unsafe dynamic target: %s.",
                dynamicTargetRejectReasonName(rejection));
            triggerDynamicTargetLoss("dynamic target failed safety validation");
            return;
        }
        if (!msg->header.stamp.isZero() && !last_dynamic_target_stamp_.isZero() &&
            msg->header.stamp <= last_dynamic_target_stamp_) {
            ROS_WARN_THROTTLE(1.0, "YOPO ignored out-of-order dynamic target.");
            return;
        }
        planner_->setDynamicTargetTrackingScale(dynamicTargetTrackingScale());
        if (!planner_->updateDynamicTarget(position, velocity)) return;

        last_dynamic_target_stamp_ = msg->header.stamp;
        last_dynamic_target_wall_ = ros::WallTime::now();
        dynamic_target_seen_ = true;
        dynamic_target_loss_latched_ = false;
        goal_received_ = true;
        ROS_INFO_THROTTLE(
            1.0, "YOPO rolling target: p=(%.2f %.2f %.2f), v=(%.2f %.2f %.2f).",
            position.x(), position.y(), position.z(), velocity.x(), velocity.y(), velocity.z());
    }

    void dynamicTargetStatusCallback(const std_msgs::String::ConstPtr& msg) {
        const ros::WallTime now = ros::WallTime::now();
        if (msg->data == "CONFIRMED") {
            tracker_status_ = msg->data;
            dynamic_target_coast_start_wall_ = ros::WallTime();
            planner_->setDynamicTargetTrackingScale(1.0);
            return;
        }
        if (msg->data == "COASTING") {
            if (tracker_status_ != "COASTING" ||
                dynamic_target_coast_start_wall_.isZero()) {
                dynamic_target_coast_start_wall_ = now;
                ROS_WARN("YOPO tracker COASTING: continue predictive replanning with slowdown.");
            }
            tracker_status_ = msg->data;
            planner_->setDynamicTargetTrackingScale(dynamicTargetTrackingScale());
            return;
        }
        tracker_status_ = msg->data;
        if (msg->data == "LOST") {
            triggerDynamicTargetLoss("tracker exceeded COASTING window and reported LOST");
        } else {
            triggerDynamicTargetLoss("tracker identity is not confirmed");
        }
    }

    void targetMaskCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (msg->encoding != sensor_msgs::image_encodings::MONO8 ||
            msg->height == 0 || msg->width == 0 ||
            msg->step < msg->width || msg->data.size() < msg->step * msg->height) {
            ROS_WARN_THROTTLE(1.0, "YOPO rejected invalid target mask image.");
            return;
        }
        cv::Mat view(
            static_cast<int>(msg->height), static_cast<int>(msg->width), CV_8UC1,
            const_cast<uint8_t*>(msg->data.data()), static_cast<size_t>(msg->step));
        std::lock_guard<std::mutex> lock(target_mask_mutex_);
        latest_target_mask_ = view.clone();
        latest_target_mask_stamp_ = msg->header.stamp;
    }

    double dynamicTargetTrackingScale() const {
        if (tracker_status_ != "COASTING" ||
            dynamic_target_coast_start_wall_.isZero()) return 1.0;
        const double elapsed =
            (ros::WallTime::now() - dynamic_target_coast_start_wall_).toSec();
        const double progress = std::max(
            0.0, std::min(1.0,
                          elapsed / planner_->params().dynamic_target_coast_slowdown_time));
        const double floor = planner_->params().dynamic_target_min_speed_scale;
        return 1.0 - progress * (1.0 - floor);
    }

    void triggerDynamicTargetLoss(const char* reason) {
        if (!dynamic_target_seen_ || dynamic_target_loss_latched_) return;
        if (planner_->handleDynamicTargetLost()) {
            ROS_WARN("YOPO dynamic target hold: %s.", reason);
        }
        dynamic_target_loss_latched_ = true;
    }

    void checkDynamicTargetTimeout() {
        if (!dynamic_target_seen_ || dynamic_target_loss_latched_ ||
            last_dynamic_target_wall_.isZero()) return;
        if ((ros::WallTime::now() - last_dynamic_target_wall_).toSec() >
            dynamic_target_timeout_) {
            triggerDynamicTargetLoss("tracked target timed out");
        }
    }

    void trajStartCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        (void)msg;
        const bool synced = planner_->syncReferenceFromCurrentOdom();
        if (!control_enabled_) {
            ROS_INFO("YOPO received traj start trigger, start publishing control commands.");
        }
        if (synced) {
            ROS_INFO("YOPO synced reference from current odom at traj start trigger.");
        } else {
            ROS_WARN("YOPO received traj start trigger before odom was initialized.");
        }
        control_enabled_ = true;
    }

    void fsmStatusCallback(const px4ctrl::FsmStatus::ConstPtr& msg) {
        last_fsm_status_wall_ = ros::WallTime::now();
        const bool usable_state = msg->state == px4ctrl::FsmStatus::HOVER ||
                                  msg->state == px4ctrl::FsmStatus::EXTERNAL;
        const bool ready = msg->enabled && msg->connected && msg->armed &&
                           msg->px4_mode == "OFFBOARD" && msg->odometry_fresh &&
                           usable_state;
        if (ready && !fsm_execution_allowed_) {
            if (control_enabled_ && planner_->odomInitialized()) {
                planner_->syncReferenceFromCurrentOdom();
                ROS_WARN("YOPO resumed after FSM interruption; reference resynced to current odom.");
            }
        } else if (!ready && fsm_execution_allowed_) {
            ROS_WARN("YOPO paused because OFFBOARD FSM is %s (odom_fresh=%s).",
                     msg->state_name.c_str(), msg->odometry_fresh ? "true" : "false");
        }
        fsm_execution_allowed_ = ready;
    }

    bool fsmExecutionReady() {
        if (!pause_with_fsm_) return true;
        const bool fresh = !last_fsm_status_wall_.isZero() &&
            (ros::WallTime::now() - last_fsm_status_wall_).toSec() <= fsm_status_timeout_;
        if (!fresh && fsm_execution_allowed_) {
            ROS_WARN("YOPO paused because OFFBOARD FSM status timed out.");
            fsm_execution_allowed_ = false;
        }
        return fresh && fsm_execution_allowed_;
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!planner_->odomInitialized() || !goal_received_ || !control_enabled_ ||
            !fsmExecutionReady() || planner_->arrived() ||
            (planner_->terminalApproachActive() &&
             !planner_->dynamicLossHoldActive())) return;
        if (planner_->dynamicTargetLost()) return;
        if (planner_->goalAlignmentActive()) {
            ROS_INFO_THROTTLE(
                1.0, "YOPO pauses depth replanning while braking/turning toward new goal.");
            return;
        }
        if (planner_->requiresCameraExtrinsic() && !planner_->cameraExtrinsicReady()) {
            ROS_WARN_THROTTLE(1.0, "YOPO waiting for camera-to-body extrinsic.");
            return;
        }
        if (!shouldProcessDepth()) return;

        const auto t0 = Clock::now();
        std::array<float, 1 * 1 * 96 * 160> depth_input{};
        if (!prepareDepth(*msg, &depth_input)) return;
        const auto t1 = Clock::now();

        const auto obs_input = planner_->prepareObsInput();
        const auto t2        = Clock::now();

        std::array<float, 1 * 9 * 3 * 5> endstate_pred{};
        std::array<float, 1 * 3 * 5> score_pred{};
        if (!engine_->infer(depth_input, obs_input, &endstate_pred, &score_pred)) {
            ROS_WARN("YOPO TensorRT inference failed");
            return;
        }
        const auto t3 = Clock::now();

        float best_score   = std::numeric_limits<float>::max();
        int best_action_id = 0;
        for (int i = 0; i < static_cast<int>(score_pred.size()); ++i) {
            if (score_pred[i] < best_score) {
                best_score     = score_pred[i];
                best_action_id = i;
            }
        }

        planner_->updateTrajectory(endstate_pred, score_pred, visualize_);
        const auto t4 = Clock::now();

        publishVisualization();
        const auto t5 = Clock::now();

        printTiming(t0, t1, t2, t3, t4, t5, best_score, best_action_id);
    }

    bool shouldProcessDepth() {
        if (depth_replan_rate_ <= 0.0) return true;

        const auto now = Clock::now();
        if (!has_last_depth_plan_time_) {
            last_depth_plan_time_     = now;
            has_last_depth_plan_time_ = true;
            return true;
        }

        const double elapsed_sec =
            std::chrono::duration<double>(now - last_depth_plan_time_).count();
        if (elapsed_sec < 1.0 / depth_replan_rate_) return false;

        last_depth_plan_time_ = now;
        return true;
    }

    bool prepareDepth(
        const sensor_msgs::Image& msg, std::array<float, 1 * 1 * 96 * 160>* depth_input) {
        if (!depth_input) return false;

        cv::Mat depth_meters(msg.height, msg.width, CV_32FC1);
        if (msg.encoding == "32FC1") {
            for (uint32_t r = 0; r < msg.height; ++r) {
                const auto* src = reinterpret_cast<const float*>(msg.data.data() + r * msg.step);
                float* dst      = depth_meters.ptr<float>(r);
                std::copy(src, src + msg.width, dst);
            }
        } else if (msg.encoding == "16UC1") {
            for (uint32_t r = 0; r < msg.height; ++r) {
                const auto* src = reinterpret_cast<const uint16_t*>(msg.data.data() + r * msg.step);
                float* dst      = depth_meters.ptr<float>(r);
                for (uint32_t c = 0; c < msg.width; ++c)
                    dst[c] = static_cast<float>(src[c]) / 1000.0f;
            }
        } else {
            ROS_WARN_THROTTLE(1.0, "Unsupported depth encoding: %s", msg.encoding.c_str());
            return false;
        }

        cv::Mat oriented_depth;
        if (rotate_depth_180_) {
            cv::rotate(depth_meters, oriented_depth, cv::ROTATE_180);
        } else {
            oriented_depth = depth_meters;
        }

        cv::Mat resized;
        const auto& params = planner_->params();
        if (oriented_depth.rows != params.image_height ||
            oriented_depth.cols != params.image_width) {
            cv::resize(
                oriented_depth, resized, cv::Size(params.image_width, params.image_height), 0.0, 0.0,
                cv::INTER_NEAREST);
        } else {
            resized = oriented_depth;
        }

        cv::Mat normalized(params.image_height, params.image_width, CV_32FC1);
        cv::Mat invalid_mask(params.image_height, params.image_width, CV_8UC1);
        const float invalid_fill = static_cast<float>(
            params.sensor_reliable_max_depth / params.model_depth_max);
        size_t valid_count = 0;
        for (int r = 0; r < params.image_height; ++r) {
            const float* src = resized.ptr<float>(r);
            float* dst       = normalized.ptr<float>(r);
            uint8_t* mask    = invalid_mask.ptr<uint8_t>(r);
            for (int c = 0; c < params.image_width; ++c) {
                const float v = src[c];
                const bool valid =
                    std::isfinite(v) && v >= params.sensor_min_depth &&
                    v <= params.sensor_reliable_max_depth;
                if (valid) {
                    dst[c] = v / static_cast<float>(params.model_depth_max);
                    mask[c] = 0;
                    ++valid_count;
                } else {
                    // Use the farthest reliable sensor value as the fallback
                    // if an invalid area cannot be filled from valid neighbours.
                    // This avoids representing missing depth as a zero-distance
                    // obstacle while keeping the network input in its trained scale.
                    dst[c] = invalid_fill;
                    mask[c] = 255;
                }
            }
        }

        if (valid_count == 0) {
            ROS_WARN_THROTTLE(
                1.0,
                "YOPO skipped depth frame: no samples in D455 reliable range "
                "[%.2f, %.2f] m.",
                params.sensor_min_depth, params.sensor_reliable_max_depth);
            return false;
        }

        cv::Mat inpainted;
#ifdef YOPO_HAVE_OPENCV_PHOTO
        cv::Mat normalized_u8;
        normalized.convertTo(normalized_u8, CV_8UC1, 255.0);
        cv::Mat inpainted_u8;
        cv::inpaint(normalized_u8, invalid_mask, inpainted_u8, 1.0, cv::INPAINT_NS);
        inpainted_u8.convertTo(inpainted, CV_32FC1, 1.0 / 255.0);
#else
        inpainted = normalized.clone();
        fillInvalidDepth(invalid_mask, &inpainted);
#endif

        if (target_mask_enabled_) {
            cv::Mat target_mask;
            ros::Time target_mask_stamp;
            {
                std::lock_guard<std::mutex> lock(target_mask_mutex_);
                target_mask = latest_target_mask_.clone();
                target_mask_stamp = latest_target_mask_stamp_;
            }
            const double stamp_delta =
                (!msg.header.stamp.isZero() && !target_mask_stamp.isZero())
                    ? std::abs((msg.header.stamp - target_mask_stamp).toSec())
                    : std::numeric_limits<double>::infinity();
            if (!target_mask.empty() && stamp_delta <= target_mask_timeout_) {
                cv::Mat resized_mask;
                cv::resize(
                    target_mask, resized_mask,
                    cv::Size(params.image_width, params.image_height), 0.0, 0.0,
                    cv::INTER_NEAREST);
                if (cv::countNonZero(resized_mask) > 0) {
#ifdef YOPO_HAVE_OPENCV_PHOTO
                    cv::Mat source_u8;
                    cv::Mat repaired_u8;
                    inpainted.convertTo(source_u8, CV_8UC1, 255.0);
                    cv::inpaint(
                        source_u8, resized_mask, repaired_u8,
                        target_mask_inpaint_radius_, cv::INPAINT_NS);
                    repaired_u8.convertTo(inpainted, CV_32FC1, 1.0 / 255.0);
#else
                    fillInvalidDepth(resized_mask, &inpainted);
#endif
                    ROS_INFO_THROTTLE(
                        1.0, "YOPO removed tracked target from network depth input (%d pixels).",
                        cv::countNonZero(resized_mask));
                }
            }
        }

        publishPreprocessedDepth(msg.header, inpainted);

        for (int r = 0; r < params.image_height; ++r) {
            const float* src = inpainted.ptr<float>(r);
            for (int c = 0; c < params.image_width; ++c) {
                (*depth_input)[r * params.image_width + c] = src[c];
            }
        }
        return true;
    }

    static void fillInvalidDepth(const cv::Mat& invalid_mask, cv::Mat* depth) {
        if (!depth) return;
        cv::Mat current   = depth->clone();
        cv::Mat remaining = invalid_mask.clone();
        for (int iter = 0; iter < 3; ++iter) {
            cv::Mat next           = current.clone();
            cv::Mat next_remaining = remaining.clone();
            for (int r = 0; r < current.rows; ++r) {
                for (int c = 0; c < current.cols; ++c) {
                    if (remaining.at<uint8_t>(r, c) == 0) continue;
                    float sum = 0.0f;
                    int count = 0;
                    for (int dr = -1; dr <= 1; ++dr) {
                        for (int dc = -1; dc <= 1; ++dc) {
                            if (dr == 0 && dc == 0) continue;
                            const int rr = r + dr;
                            const int cc = c + dc;
                            if (rr < 0 || rr >= current.rows || cc < 0 || cc >= current.cols)
                                continue;
                            if (remaining.at<uint8_t>(rr, cc) != 0) continue;
                            sum += current.at<float>(rr, cc);
                            ++count;
                        }
                    }
                    if (count > 0) {
                        next.at<float>(r, c)             = sum / static_cast<float>(count);
                        next_remaining.at<uint8_t>(r, c) = 0;
                    }
                }
            }
            current   = next;
            remaining = next_remaining;
        }
        *depth = current;
    }

    void publishPreprocessedDepth(const std_msgs::Header& header, const cv::Mat& normalized_depth) {
        if (preprocessed_depth_pub_.getNumSubscribers() > 0) {
            cv::Mat display_u8;
            normalized_depth.convertTo(display_u8, CV_8UC1, 255.0);

            sensor_msgs::Image msg;
            msg.header       = header;
            msg.height       = static_cast<uint32_t>(display_u8.rows);
            msg.width        = static_cast<uint32_t>(display_u8.cols);
            msg.encoding     = "mono8";
            msg.is_bigendian = false;
            msg.step         = static_cast<sensor_msgs::Image::_step_type>(display_u8.cols);
            msg.data.assign(display_u8.datastart, display_u8.dataend);
            preprocessed_depth_pub_.publish(msg);
        }

        if (preprocessed_depth_float_pub_.getNumSubscribers() > 0) {
            cv::Mat continuous_depth = normalized_depth.isContinuous() ? normalized_depth
                                                                       : normalized_depth.clone();

            sensor_msgs::Image msg;
            msg.header       = header;
            msg.height       = static_cast<uint32_t>(continuous_depth.rows);
            msg.width        = static_cast<uint32_t>(continuous_depth.cols);
            msg.encoding     = "32FC1";
            msg.is_bigendian = false;
            msg.step =
                static_cast<sensor_msgs::Image::_step_type>(continuous_depth.cols * sizeof(float));
            msg.data.resize(static_cast<size_t>(msg.step) * msg.height);
            std::memcpy(msg.data.data(), continuous_depth.ptr<float>(), msg.data.size());
            preprocessed_depth_float_pub_.publish(msg);
        }
    }

    void controlTimer(const ros::TimerEvent&) {
        checkDynamicTargetTimeout();
        if (!control_enabled_ || !goal_received_ || !fsmExecutionReady()) return;
        if (planner_->requiresCameraExtrinsic() && !planner_->cameraExtrinsicReady()) {
            ROS_WARN_THROTTLE(
                1.0, "YOPO holds control command until camera-to-body extrinsic is received.");
            return;
        }

        quadrotor_msgs::PositionCommand cmd;
        if (planner_->fillControlCommand(&cmd)) {
            ctrl_pub_.publish(cmd);
        }
    }

    static sensor_msgs::PointCloud2 makeXYZCloud(const std::vector<Eigen::Vector3f>& points) {
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp    = ros::Time::now();
        cloud.header.frame_id = "world";
        cloud.height          = 1;
        cloud.width           = static_cast<uint32_t>(points.size());
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2FieldsByString(1, "xyz");
        modifier.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        for (const auto& p : points) {
            *x = p.x();
            *y = p.y();
            *z = p.z();
            ++x;
            ++y;
            ++z;
        }
        return cloud;
    }

    static sensor_msgs::PointCloud2 makeXYZICloud(const std::vector<Eigen::Vector4f>& points) {
        sensor_msgs::PointCloud2 cloud;
        cloud.header.stamp    = ros::Time::now();
        cloud.header.frame_id = "world";
        sensor_msgs::PointCloud2Modifier modifier(cloud);
        modifier.setPointCloud2Fields(
            4, "x", 1, sensor_msgs::PointField::FLOAT32, "y", 1, sensor_msgs::PointField::FLOAT32,
            "z", 1, sensor_msgs::PointField::FLOAT32, "intensity", 1,
            sensor_msgs::PointField::FLOAT32);
        modifier.resize(points.size());
        sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
        sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
        sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
        sensor_msgs::PointCloud2Iterator<float> intensity(cloud, "intensity");
        for (const auto& p : points) {
            *x         = p.x();
            *y         = p.y();
            *z         = p.z();
            *intensity = p.w();
            ++x;
            ++y;
            ++z;
            ++intensity;
        }
        return cloud;
    }

    void publishVisualization() {
        constexpr int kSamples = 20;
        if (best_traj_pub_.getNumSubscribers() > 0) {
            best_traj_pub_.publish(makeXYZCloud(planner_->bestTrajectoryPoints(kSamples)));
        }
        if (visualize_ && lattice_traj_pub_.getNumSubscribers() > 0) {
            lattice_traj_pub_.publish(makeXYZCloud(planner_->latticeTrajectoryPoints(kSamples)));
        }
        if (visualize_ && all_trajs_pub_.getNumSubscribers() > 0) {
            all_trajs_pub_.publish(makeXYZICloud(planner_->allTrajectoryPoints(kSamples)));
        }
    }

    void printTiming(
        const Clock::time_point& t0, const Clock::time_point& t1, const Clock::time_point& t2,
        const Clock::time_point& t3, const Clock::time_point& t4, const Clock::time_point& t5,
        float best_score, int best_action_id) {
        ++count_;
        const double depth_ms     = elapsedMs(t0, t1);
        const double prepare_ms   = elapsedMs(t1, t2);
        const double infer_ms     = elapsedMs(t2, t3);
        const double post_ms      = elapsedMs(t3, t4);
        const double visualize_ms = elapsedMs(t4, t5);
        time_interpolation_ += depth_ms;
        time_prepare_ += prepare_ms;
        time_forward_ += infer_ms;
        time_process_ += post_ms;
        time_visualize_ += visualize_ms;
        const double total = elapsedMs(t0, t5);

        publishLog(
            depth_ms, prepare_ms, infer_ms, post_ms, visualize_ms, total, best_score,
            best_action_id);

        if (total > 1000.0 / depth_fps_) {
            ROS_WARN("YOPO processing %.2f ms exceeds %.2f ms", total, 1000.0 / depth_fps_);
        }
        if (verbose_ || total > 1000.0 / depth_fps_) {
            ROS_INFO(
                "YOPO avg ms: depth %.2f prepare %.2f infer %.2f post %.2f vis %.2f",
                time_interpolation_ / count_, time_prepare_ / count_, time_forward_ / count_,
                time_process_ / count_, time_visualize_ / count_);
        }
    }

    void publishLog(
        double depth_ms, double prepare_ms, double infer_ms, double post_ms, double visualize_ms,
        double total_ms, float best_score, int best_action_id) {
        if (log_pub_.getNumSubscribers() == 0) return;

        yopo_planner::YopoLog msg;
        msg.header.stamp     = ros::Time::now();
        msg.header.frame_id  = "world";
        msg.depth_ms         = static_cast<float>(depth_ms);
        msg.prepare_ms       = static_cast<float>(prepare_ms);
        msg.infer_ms         = static_cast<float>(infer_ms);
        msg.post_ms          = static_cast<float>(post_ms);
        msg.visualize_ms     = static_cast<float>(visualize_ms);
        msg.total_ms         = static_cast<float>(total_ms);
        msg.avg_depth_ms     = static_cast<float>(time_interpolation_ / count_);
        msg.avg_prepare_ms   = static_cast<float>(time_prepare_ / count_);
        msg.avg_infer_ms     = static_cast<float>(time_forward_ / count_);
        msg.avg_post_ms      = static_cast<float>(time_process_ / count_);
        msg.avg_visualize_ms = static_cast<float>(time_visualize_ / count_);
        msg.best_score       = best_score;
        msg.best_action_id   = best_action_id;
        log_pub_.publish(msg);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    std::unique_ptr<YopoPlanner> planner_;
    std::unique_ptr<YopoEngine> engine_;

    ros::Publisher ctrl_pub_;
    ros::Publisher best_traj_pub_;
    ros::Publisher all_trajs_pub_;
    ros::Publisher lattice_traj_pub_;
    ros::Publisher log_pub_;
    ros::Publisher preprocessed_depth_pub_;
    ros::Publisher preprocessed_depth_float_pub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber dynamic_target_sub_;
    ros::Subscriber dynamic_target_status_sub_;
    ros::Subscriber target_mask_sub_;
    ros::Subscriber traj_start_sub_;
    ros::Subscriber fsm_status_sub_;
    ros::Subscriber extrinsic_sub_;
    ros::Timer ctrl_timer_;

    bool verbose_                             = false;
    bool visualize_                           = true;
    bool rotate_depth_180_                    = false;
    bool wait_for_traj_start_trigger_         = false;
    bool control_enabled_                     = true;
    bool require_explicit_goal_               = false;
    bool goal_received_                       = false;
    bool dynamic_target_seen_                 = false;
    bool dynamic_target_loss_latched_         = false;
    std::string tracker_status_                = "LOST";
    ros::WallTime dynamic_target_coast_start_wall_;
    double dynamic_target_timeout_            = 0.8;
    DynamicTargetSafetyConfig dynamic_target_safety_config_;
    Eigen::Vector3d latest_vehicle_position_ = Eigen::Vector3d::Zero();
    bool has_vehicle_position_ = false;
    bool target_mask_enabled_                  = true;
    double target_mask_timeout_                = 0.25;
    double target_mask_inpaint_radius_         = 3.0;
    std::mutex target_mask_mutex_;
    cv::Mat latest_target_mask_;
    ros::Time latest_target_mask_stamp_;
    ros::Time last_dynamic_target_stamp_;
    ros::WallTime last_dynamic_target_wall_;
    bool pause_with_fsm_                      = false;
    bool fsm_execution_allowed_              = false;
    double fsm_status_timeout_                = 0.5;
    ros::WallTime last_fsm_status_wall_;
    bool extrinsic_ready_logged_              = false;
    bool depth_orientation_logged_            = false;
    Eigen::Vector3d translation_body_optical_ = Eigen::Vector3d::Zero();
    bool has_last_depth_plan_time_            = false;
    Clock::time_point last_depth_plan_time_;
    double depth_replan_rate_  = 0.0;
    double depth_fps_          = 30.0;
    int count_                 = 0;
    double time_forward_       = 0.0;
    double time_process_       = 0.0;
    double time_prepare_       = 0.0;
    double time_interpolation_ = 0.0;
    double time_visualize_     = 0.0;
};

}  // namespace yopo_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "yopo_planner_node");
    yopo_planner::YopoPlannerNode node;
    ros::spin();
    return 0;
}
