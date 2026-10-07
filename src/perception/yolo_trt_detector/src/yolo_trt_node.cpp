#include <NvInfer.h>
#include <NvInferVersion.h>
#include <boost/bind.hpp>
#include <algorithm>
#include <array>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <cuda_runtime_api.h>
#include <cv_bridge/cv_bridge.h>
#include <deque>
#include <fstream>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <image_transport/image_transport.h>
#include <limits>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <memory>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <numeric>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <stdexcept>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/String.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <yolo_trt_detector/frustum_lidar_association.h>
#include <yolo_trt_detector/image_orientation.h>
#include <yolo_trt_detector/lidar_target_tracker.h>
#include <yolo_trt_detector/publication_limiter.h>
#include <yolo_trt_detector/standoff_goal.h>
#include <yolo_trt_detector/track_reacquisition.h>
#include <yolo_trt_detector/visual_lidar_match.h>

namespace {
// nvinfer1是NVIDIA TensorRT的核心命名空间，包含了TensorRT的所有主要类和函数。
// ILogger是TensorRT提供的一个接口类，用于处理日志消息。定义“合同模板”
// TrtLogger是一个自定义的日志处理类，继承（public）自nvinfer1::ILogger。“合同”的具体实现
class TrtLogger : public nvinfer1::ILogger {
  public:  // public\protcted\private。代表下面的成员都可以被用
    // Severity是一个类型，表示日志信息的严重程度。severity是参数变量。
    // override检查是否重写，写错就报错；noexcept表示这个函数不会抛出异常
    // log（）接收日志消息msg及其严重度severity，如果严重就报警[TensorRT]：msg
    void log(Severity severity, const char* msg) noexcept override {
        if (severity <= Severity::kWARNING) {
            ROS_WARN_STREAM("[TensorRT] " << msg);
        }
    }
};
// 检测结果与预处理参数结构体
struct Detection {
    cv::Rect box;
    float score  = 0.0f;
    int class_id = 0;
};

struct LetterboxInfo {
    float scale = 1.0f;
    float pad_x = 0.0f;
    float pad_y = 0.0f;
};

struct DepthEstimate {
    float depth       = 0.0f;
    float mad         = 0.0f;
    int valid_samples = 0;
    bool valid        = false;
};

struct TargetMeasurement {
    Detection detection;
    tf2::Vector3 position;
    float depth = 0.0f;
    float depth_mad = 0.0f;
    int depth_samples = 0;
};

void checkCuda(cudaError_t status, const std::string& where) {
    if (status != cudaSuccess) {
        throw std::runtime_error(where + ": " + cudaGetErrorString(status));
    }
}

std::vector<char> readFile(const std::string& path) {
    std::ifstream file(path, std::ios::binary);
    if (!file) {
        throw std::runtime_error("failed to open engine file: " + path);
    }
    file.seekg(0, std::ios::end);
    const std::streamsize size = file.tellg();
    file.seekg(0, std::ios::beg);
    std::vector<char> data(size);
    if (!file.read(data.data(), size)) {
        throw std::runtime_error("failed to read engine file: " + path);
    }
    return data;
}

float iou(const cv::Rect& a, const cv::Rect& b) {
    const int inter_area = (a & b).area();
    const int union_area = a.area() + b.area() - inter_area;
    return union_area > 0 ? static_cast<float>(inter_area) / static_cast<float>(union_area) : 0.0f;
}

std::vector<Detection> nms(std::vector<Detection> detections, float threshold) {
    std::sort(detections.begin(), detections.end(), [](const Detection& lhs, const Detection& rhs) {
        return lhs.score > rhs.score;
    });

    std::vector<Detection> kept;
    std::vector<bool> removed(detections.size(), false);
    for (size_t i = 0; i < detections.size(); ++i) {
        if (removed[i]) {
            continue;
        }
        kept.push_back(detections[i]);
        for (size_t j = i + 1; j < detections.size(); ++j) {
            if (!removed[j] && detections[i].class_id == detections[j].class_id &&
                iou(detections[i].box, detections[j].box) > threshold) {
                removed[j] = true;
            }
        }
    }
    return kept;
}

}  // namespace

class YoloTrtNode {
  public:
    YoloTrtNode(ros::NodeHandle& nh, ros::NodeHandle& pnh) : nh_(nh), pnh_(pnh), it_(nh_) {
        pnh_.param<std::string>("engine_file", engine_file_, "models/best_floatdepth.engine");
        pnh_.param<std::string>("image_topic", image_topic_, "/camera/color/image_raw");
        pnh_.param<std::string>("depth_topic", depth_topic_, "/camera/aligned_depth_to_color/image_raw");
        pnh_.param<std::string>(
            "camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/kf_fusion/kf_imu_odom");
        pnh_.param<std::string>("extrinsic_topic", extrinsic_topic_, "/vins_fusion/extrinsic");
        pnh_.param<std::string>("target_topic", target_topic_, "yolo_trt/target_point");
        pnh_.param<std::string>(
            "target_mask_topic", target_mask_topic_, "/yolo_trt/tracked_target_mask");
        pnh_.param<bool>("publish_planner_goal", publish_planner_goal_, false);
        pnh_.param<std::string>("planner_goal_topic", planner_goal_topic_, "/planning/goal");
        pnh_.param<double>(
            "planner_goal_publish_rate", planner_goal_publish_rate_, 10.0);
        pnh_.param<double>(
            "planner_goal_standoff_distance", planner_goal_standoff_distance_, 0.0);
        pnh_.param<std::string>("frame_id", frame_id_, "");
        pnh_.param<std::string>("global_frame_id", global_frame_id_, "world");
        pnh_.param<float>("conf_threshold", conf_threshold_, 0.25f);
        pnh_.param<float>("nms_threshold", nms_threshold_, 0.70f);
        pnh_.param<float>("depth_scale", depth_scale_, 0.001f);
        pnh_.param<double>("max_pair_dt", max_pair_dt_, 0.05);
        pnh_.param<bool>("publish_target_point", publish_target_point_, true);
        pnh_.param<int>("depth_roi_radius", depth_roi_radius_, 3);
        pnh_.param<bool>("rotate_input_180", rotate_input_180_, false);
        pnh_.param<bool>("use_vins_extrinsic", use_vins_extrinsic_, true);
        pnh_.param<double>("camera_tx", camera_tx_, 0.0);
        pnh_.param<double>("camera_ty", camera_ty_, 0.0);
        pnh_.param<double>("camera_tz", camera_tz_, 0.0);
        pnh_.param<double>("camera_roll", camera_roll_, 0.0);
        pnh_.param<double>("camera_pitch", camera_pitch_, 0.0);
        pnh_.param<double>("camera_yaw", camera_yaw_, 0.0);
        pnh_.param<bool>(
            "use_camera_rotation_matrix", use_camera_rotation_matrix_, false);
        pnh_.param<bool>("output_world_frame", output_world_frame_, false);
        pnh_.param<double>("camera_r00", camera_rotation_values_[0], 0.0);
        pnh_.param<double>("camera_r01", camera_rotation_values_[1], 0.0);
        pnh_.param<double>("camera_r02", camera_rotation_values_[2], 1.0);
        pnh_.param<double>("camera_r10", camera_rotation_values_[3], -1.0);
        pnh_.param<double>("camera_r11", camera_rotation_values_[4], 0.0);
        pnh_.param<double>("camera_r12", camera_rotation_values_[5], 0.0);
        pnh_.param<double>("camera_r20", camera_rotation_values_[6], 0.0);
        pnh_.param<double>("camera_r21", camera_rotation_values_[7], -1.0);
        pnh_.param<double>("camera_r22", camera_rotation_values_[8], 0.0);
        pnh_.param<int>("detect_hz", detect_hz_, 30);

        // Target measurement quality and temporal tracking. These parameters only affect
        // post-processing; changing them never requires rebuilding the TensorRT engine.
        pnh_.param<bool>("enable_target_tracking", enable_target_tracking_, true);
        pnh_.param<std::string>(
            "raw_target_topic", raw_target_topic_, "yolo_trt/target_point_raw");
        pnh_.param<std::string>(
            "tracked_odom_topic", tracked_odom_topic_, "yolo_trt/tracked_target");
        pnh_.param<std::string>(
            "tracker_status_topic", tracker_status_topic_, "yolo_trt/target_status");
        pnh_.param<double>(
            "tracked_target_publish_rate", tracked_target_publish_rate_, 10.0);
        pnh_.param<double>("depth_roi_scale", depth_roi_scale_, 0.35);
        pnh_.param<int>("depth_min_samples", depth_min_samples_, 12);
        pnh_.param<double>("depth_cluster_abs", depth_cluster_abs_, 0.25);
        pnh_.param<double>("depth_cluster_rel", depth_cluster_rel_, 0.08);
        pnh_.param<double>("depth_mad_max", depth_mad_max_, 0.25);
        pnh_.param<double>("target_min_depth", target_min_depth_, 0.25);
        pnh_.param<double>("target_max_depth", target_max_depth_, 15.0);
        pnh_.param<double>("target_mask_expand_scale", target_mask_expand_scale_, 1.20);
        pnh_.param<double>("odom_buffer_duration", odom_buffer_duration_, 2.0);
        pnh_.param<double>("odom_max_time_offset", odom_max_time_offset_, 0.10);
        pnh_.param<double>("track_init_confidence", track_init_confidence_, 0.55);
        pnh_.param<double>("track_update_confidence", track_update_confidence_, 0.25);
        pnh_.param<int>("track_confirm_hits", track_confirm_hits_, 3);
        pnh_.param<int>("track_confirm_window", track_confirm_window_, 5);
        pnh_.param<double>("track_gate_mahalanobis", track_gate_mahalanobis_, 11.34);
        pnh_.param<double>("track_init_gate_distance", track_init_gate_distance_, 0.8);
        pnh_.param<double>("track_coast_publish_time", track_coast_publish_time_, 0.30);
        pnh_.param<double>("track_lost_time", track_lost_time_, 0.50);
        pnh_.param<double>(
            "track_identity_memory_time", track_identity_memory_time_, 2.0);
        pnh_.param<double>(
            "track_reacquire_distance", track_reacquire_distance_, 1.5);
        pnh_.param<double>(
            "track_max_measurement_jump", track_max_measurement_jump_, 1.0);
        pnh_.param<double>("track_process_accel_std", track_process_accel_std_, 1.5);
        pnh_.param<double>("track_measurement_std", track_measurement_std_, 0.08);
        pnh_.param<double>("track_initial_position_std", track_initial_position_std_, 0.15);
        pnh_.param<double>("track_initial_velocity_std", track_initial_velocity_std_, 1.0);

        // LiDAR is the primary 3D sensor.  YOLO only identifies a projected
        // cluster; subsequent updates come from the motion-compensated cloud.
        pnh_.param<bool>("enable_lidar_target_tracking", enable_lidar_target_tracking_, false);
        pnh_.param<std::string>(
            "lidar_cloud_topic", lidar_cloud_topic_,
            "/cloud_registered");
        pnh_.param<double>("lidar_min_range", lidar_tracker_config_.min_range, 0.45);
        pnh_.param<double>("lidar_max_range", lidar_tracker_config_.max_range, 18.0);
        pnh_.param<double>(
            "lidar_self_exclusion_radius",
            lidar_tracker_config_.self_exclusion_radius, 0.65);
        pnh_.param<double>(
            "lidar_background_voxel_size",
            lidar_tracker_config_.background_voxel_size, 0.18);
        pnh_.param<int>(
            "lidar_background_confirm_frames",
            lidar_tracker_config_.background_confirm_frames, 8);
        pnh_.param<double>(
            "lidar_background_forget_seconds",
            lidar_tracker_config_.background_forget_seconds, 3.0);
        pnh_.param<double>(
            "lidar_cluster_voxel_size",
            lidar_tracker_config_.cluster_voxel_size, 0.22);
        pnh_.param<int>(
            "lidar_cluster_min_points",
            lidar_tracker_config_.cluster_min_points, 2);
        pnh_.param<int>(
            "lidar_cluster_max_points",
            lidar_tracker_config_.cluster_max_points, 1200);
        pnh_.param<double>(
            "lidar_cluster_max_extent",
            lidar_tracker_config_.cluster_max_extent, 1.20);
        pnh_.param<double>(
            "lidar_cluster_max_diagonal",
            lidar_tracker_config_.cluster_max_diagonal, 1.60);
        pnh_.param<double>(
            "lidar_track_gate_distance",
            lidar_tracker_config_.track_gate_distance, 1.00);
        pnh_.param<double>(
            "lidar_track_gate_max_distance",
            lidar_tracker_config_.track_gate_max_distance, 0.50);
        pnh_.param<double>(
            "lidar_association_min_score_separation",
            lidar_tracker_config_.association_min_score_separation, 0.12);
        pnh_.param<double>(
            "lidar_track_protection_radius",
            lidar_tracker_config_.track_protection_radius, 0.75);
        pnh_.param<double>(
            "lidar_lost_time", lidar_tracker_config_.lidar_lost_time, 2.0);
        pnh_.param<double>(
            "lidar_covariance_lost_threshold",
            lidar_tracker_config_.covariance_lost_threshold, 2.25);
        pnh_.param<double>(
            "lidar_visual_confirmation_gate",
            lidar_tracker_config_.visual_confirmation_gate, 1.25);
        pnh_.param<int>(
            "lidar_visual_confirm_hits",
            lidar_tracker_config_.visual_confirm_hits, 5);
        pnh_.param<int>(
            "lidar_visual_confirm_window",
            lidar_tracker_config_.visual_confirm_window, 7);
        pnh_.param<double>(
            "lidar_visual_confirm_max_gap",
            lidar_tracker_config_.visual_confirm_max_gap, 0.25);
        pnh_.param<double>(
            "lidar_visual_tentative_gate",
            lidar_tracker_config_.visual_tentative_gate, 0.60);
        pnh_.param<double>(
            "lidar_visual_tentative_max_speed",
            lidar_tracker_config_.visual_tentative_max_speed, 2.0);
        pnh_.param<double>(
            "lidar_visual_acquire_max_range",
            lidar_tracker_config_.visual_acquire_max_range, 8.0);
        pnh_.param<double>(
            "lidar_visual_max_age",
            lidar_tracker_config_.visual_max_age, 1.0);
        pnh_.param<bool>(
            "lidar_require_recent_visual_confirmation",
            lidar_tracker_config_.require_recent_visual_confirmation, true);
        pnh_.param<double>(
            "lidar_max_target_speed",
            lidar_tracker_config_.max_target_speed, 4.0);
        pnh_.param<double>(
            "lidar_max_target_acceleration",
            lidar_tracker_config_.max_target_acceleration, 8.0);
        pnh_.param<double>(
            "lidar_measurement_position_slack",
            lidar_tracker_config_.measurement_position_slack, 0.25);
        pnh_.param<double>(
            "lidar_measurement_max_update_distance",
            lidar_tracker_config_.measurement_max_update_distance, 0.45);
        pnh_.param<double>(
            "lidar_measurement_velocity_slack",
            lidar_tracker_config_.measurement_velocity_slack, 0.8);
        pnh_.param<double>(
            "lidar_measurement_std",
            lidar_tracker_config_.lidar_measurement_std, 0.10);
        pnh_.param<double>(
            "lidar_smooth_model_accel_std",
            lidar_tracker_config_.smooth_model_accel_std, 0.7);
        pnh_.param<double>(
            "lidar_maneuver_model_accel_std",
            lidar_tracker_config_.maneuver_model_accel_std, 3.0);
        pnh_.param<bool>(
            "lidar_enforce_range_point_count",
            lidar_tracker_config_.enforce_range_point_count, false);
        pnh_.param<double>(
            "lidar_target_physical_size",
            lidar_tracker_config_.target_physical_size, 0.55);
        pnh_.param<double>(
            "lidar_point_density_at_1m",
            lidar_tracker_config_.point_density_at_1m, 300.0);
        pnh_.param<double>(
            "lidar_point_count_min_ratio",
            lidar_tracker_config_.point_count_min_ratio, 0.15);
        pnh_.param<double>(
            "lidar_point_count_max_ratio",
            lidar_tracker_config_.point_count_max_ratio, 5.0);
        lidar_target_physical_size_ = lidar_tracker_config_.target_physical_size;
        pnh_.param<double>("lidar_visual_match_margin", lidar_visual_match_margin_, 0.15);
        pnh_.param<double>("lidar_candidate_max_age", lidar_candidate_max_age_, 0.25);
        pnh_.param<int>("target_class_id", target_class_id_, 0);
        pnh_.param<double>(
            "lidar_visual_max_center_distance",
            visual_lidar_match_config_.max_normalized_center_distance, 0.35);
        pnh_.param<double>(
            "lidar_visual_min_rank_separation",
            visual_lidar_match_config_.minimum_rank_separation, 0.12);
        pnh_.param<bool>(
            "lidar_require_single_detection_on_acquire",
            visual_lidar_match_config_.require_single_detection_on_acquire, true);
        pnh_.param<double>(
            "lidar_frustum_cluster_radius",
            frustum_lidar_config_.cluster_radius, 0.14);
        pnh_.param<int>(
            "lidar_frustum_min_points", frustum_lidar_config_.min_points, 3);
        pnh_.param<int>(
            "lidar_frustum_max_points", frustum_lidar_config_.max_points, 800);
        pnh_.param<double>(
            "lidar_frustum_max_extent", frustum_lidar_config_.max_extent, 0.80);
        pnh_.param<double>(
            "lidar_frustum_max_diagonal", frustum_lidar_config_.max_diagonal, 1.00);
        pnh_.param<double>(
            "lidar_frustum_max_center_distance",
            frustum_lidar_config_.max_normalized_center_distance, 0.55);
        pnh_.param<double>(
            "lidar_frustum_min_rank_separation",
            frustum_lidar_config_.minimum_rank_separation, 0.15);
        pnh_.param<double>(
            "lidar_visual_filter_max_age", lidar_visual_filter_max_age_, 0.25);
        pnh_.param<double>(
            "lidar_visual_filter_margin", lidar_visual_filter_margin_, 0.20);
        lidar_tracker_.configure(lidar_tracker_config_);

        if (publish_planner_goal_ &&
            (!publish_target_point_ || !enable_target_tracking_ || !output_world_frame_)) {
            ROS_WARN(
                "Disabling planner goal output: publish_planner_goal requires "
                "publish_target_point, enable_target_tracking and output_world_frame.");
            publish_planner_goal_ = false;
        }
        if (publish_planner_goal_ && !(planner_goal_publish_rate_ > 0.0)) {
            throw std::runtime_error("planner_goal_publish_rate must be > 0");
        }
        if (!(planner_goal_standoff_distance_ >= 0.0) ||
            !std::isfinite(planner_goal_standoff_distance_)) {
            throw std::runtime_error("planner_goal_standoff_distance must be finite and >= 0");
        }
        if (!(target_mask_expand_scale_ >= 1.0) ||
            !std::isfinite(target_mask_expand_scale_)) {
            throw std::runtime_error("target mask expansion must be finite and >= 1");
        }

        yolo_trt_detector::TrackReacquisitionConfig reacquisition_config;
        reacquisition_config.init_confidence = track_init_confidence_;
        reacquisition_config.update_confidence = track_update_confidence_;
        reacquisition_config.confirm_hits = track_confirm_hits_;
        reacquisition_config.confirm_window = track_confirm_window_;
        reacquisition_config.init_gate_distance = track_init_gate_distance_;
        reacquisition_config.identity_memory_seconds = track_identity_memory_time_;
        reacquisition_config.reacquire_distance = track_reacquire_distance_;
        reacquisition_config.max_measurement_jump = track_max_measurement_jump_;
        reacquisition_policy_.configure(reacquisition_config);
        target_publish_limiter_.configure(tracked_target_publish_rate_);

        configureStaticCameraExtrinsic();
        loadEngine();

        image_sub_.subscribe(nh_, image_topic_, 2);
        depth_sub_.subscribe(nh_, depth_topic_, 2);
        SyncPolicy sync_policy(5);
        sync_policy.setMaxIntervalDuration(ros::Duration(max_pair_dt_));
        sync_.reset(new Sync(sync_policy));
        sync_->connectInput(image_sub_, depth_sub_);
        sync_->registerCallback(boost::bind(
            &YoloTrtNode::pairedCallback, this,
            boost::placeholders::_1, boost::placeholders::_2));
        if (publish_target_point_) {
            camera_info_sub_ =
                nh_.subscribe(camera_info_topic_, 1, &YoloTrtNode::cameraInfoCallback, this);
            if (output_world_frame_) {
                odom_sub_ = nh_.subscribe(odom_topic_, 20, &YoloTrtNode::odomCallback, this);
            }
            if (use_vins_extrinsic_) {
                extrinsic_sub_ =
                    nh_.subscribe(extrinsic_topic_, 20, &YoloTrtNode::extrinsicCallback, this);
            }
            if (enable_lidar_target_tracking_) {
                lidar_cloud_sub_ = nh_.subscribe(
                    lidar_cloud_topic_, 2,
                    &YoloTrtNode::lidarCloudCallback, this,
                    ros::TransportHints().tcpNoDelay());
            }
        }
        annotated_pub_    = it_.advertise("yolo_trt/annotated_image", 1);
        target_mask_pub_  = it_.advertise(target_mask_topic_, 1);
        marker_pub_       = nh_.advertise<visualization_msgs::MarkerArray>("yolo_trt/markers", 1);
        if (publish_target_point_) {
            target_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(target_topic_, 1);
            raw_target_point_pub_ =
                nh_.advertise<geometry_msgs::PointStamped>(raw_target_topic_, 1);
            tracked_odom_pub_ = nh_.advertise<nav_msgs::Odometry>(tracked_odom_topic_, 1);
            tracker_status_pub_ = nh_.advertise<std_msgs::String>(tracker_status_topic_, 1, true);
            if (publish_planner_goal_) {
                planner_goal_pub_ =
                    nh_.advertise<geometry_msgs::PoseStamped>(planner_goal_topic_, 1);
            }
        }

        ROS_INFO_STREAM(
            "yolo_trt_node ready, engine="
            << engine_file_ << ", image_topic=" << image_topic_ << ", depth_topic=" << depth_topic_
            << ", camera_info_topic=" << camera_info_topic_ << ", odom_topic=" << odom_topic_
            << ", extrinsic_topic=" << extrinsic_topic_ << ", publish_target_point="
            << publish_target_point_ << ", target_topic=" << target_topic_
            << ", output_world_frame=" << output_world_frame_
            << ", target_tracking=" << enable_target_tracking_
            << ", lidar_target_tracking=" << enable_lidar_target_tracking_
            << ", lidar_cloud_topic=" << lidar_cloud_topic_
            << ", publish_planner_goal=" << publish_planner_goal_
            << ", planner_goal_topic=" << planner_goal_topic_
            << ", planner_goal_standoff_distance=" << planner_goal_standoff_distance_
            << ", rotate_input_180=" << rotate_input_180_
            << ". Depth must already be aligned to the color image.");
        if (enable_target_tracking_ && !output_world_frame_) {
            ROS_WARN("Target tracking is enabled in a moving body frame; world-frame output is recommended");
        }
    }

    ~YoloTrtNode() {
        for (void* ptr : device_buffers_) {
            if (ptr) {
                cudaFree(ptr);
            }
        }
        if (stream_) {
            cudaStreamDestroy(stream_);
        }
    }

  private:
    void configureStaticCameraExtrinsic() {
        camera_rotation_body_depth_.setValue(
            camera_rotation_values_[0], camera_rotation_values_[1], camera_rotation_values_[2],
            camera_rotation_values_[3], camera_rotation_values_[4], camera_rotation_values_[5],
            camera_rotation_values_[6], camera_rotation_values_[7], camera_rotation_values_[8]);
        camera_translation_body_depth_.setValue(camera_tx_, camera_ty_, camera_tz_);

        if (!use_camera_rotation_matrix_) {
            return;
        }

        constexpr double kTolerance = 1e-3;
        for (int r = 0; r < 3; ++r) {
            const tf2::Vector3 row = camera_rotation_body_depth_.getRow(r);
            if (!std::isfinite(row.x()) || !std::isfinite(row.y()) ||
                !std::isfinite(row.z()) || std::abs(row.length2() - 1.0) > kTolerance) {
                throw std::runtime_error("camera rotation matrix contains an invalid row");
            }
            for (int other = r + 1; other < 3; ++other) {
                if (std::abs(row.dot(camera_rotation_body_depth_.getRow(other))) > kTolerance) {
                    throw std::runtime_error("camera rotation matrix rows are not orthogonal");
                }
            }
        }
        const double determinant = camera_rotation_body_depth_.determinant();
        if (!std::isfinite(determinant) || std::abs(determinant - 1.0) > kTolerance) {
            throw std::runtime_error("camera rotation matrix determinant is not +1");
        }
        if (!std::isfinite(camera_tx_) || !std::isfinite(camera_ty_) ||
            !std::isfinite(camera_tz_)) {
            throw std::runtime_error("camera translation contains a non-finite value");
        }

        ROS_INFO_STREAM(
            "Using calibrated depth-optical-to-body transform, det=" << determinant
            << ", translation=[" << camera_tx_ << ", " << camera_ty_ << ", " << camera_tz_
            << "] m");
    }

    void loadEngine() {
        engine_data_ = readFile(engine_file_);
        runtime_.reset(nvinfer1::createInferRuntime(logger_));
        if (!runtime_) {
            throw std::runtime_error("failed to create TensorRT runtime");
        }
        engine_.reset(runtime_->deserializeCudaEngine(engine_data_.data(), engine_data_.size()));
        if (!engine_) {
            throw std::runtime_error("failed to deserialize TensorRT engine");
        }
        context_.reset(engine_->createExecutionContext());
        if (!context_) {
            throw std::runtime_error("failed to create TensorRT execution context");
        }

        nvinfer1::Dims input_dims;
        nvinfer1::Dims output_dims;
#if NV_TENSORRT_MAJOR >= 10
        if (engine_->getNbIOTensors() != 2 ||
            engine_->getTensorIOMode("images") != nvinfer1::TensorIOMode::kINPUT ||
            engine_->getTensorIOMode("output0") != nvinfer1::TensorIOMode::kOUTPUT ||
            engine_->getTensorDataType("images") != nvinfer1::DataType::kFLOAT ||
            engine_->getTensorDataType("output0") != nvinfer1::DataType::kFLOAT ||
            engine_->getTensorFormat("images") != nvinfer1::TensorFormat::kLINEAR ||
            engine_->getTensorFormat("output0") != nvinfer1::TensorFormat::kLINEAR) {
            throw std::runtime_error("expected float32 TensorRT tensors `images` and `output0`");
        }
        input_dims = engine_->getTensorShape("images");
        output_dims = engine_->getTensorShape("output0");
#else
        input_index_  = engine_->getBindingIndex("images");
        output_index_ = engine_->getBindingIndex("output0");
        if (engine_->getNbBindings() != 2 || input_index_ < 0 || output_index_ < 0 ||
            engine_->getBindingDataType(input_index_) != nvinfer1::DataType::kFLOAT ||
            engine_->getBindingDataType(output_index_) != nvinfer1::DataType::kFLOAT ||
            engine_->getBindingFormat(input_index_) != nvinfer1::TensorFormat::kLINEAR ||
            engine_->getBindingFormat(output_index_) != nvinfer1::TensorFormat::kLINEAR) {
            throw std::runtime_error("expected float32 TensorRT bindings `images` and `output0`");
        }
        input_dims = engine_->getBindingDimensions(input_index_);
        output_dims = engine_->getBindingDimensions(output_index_);
#endif
        if (input_dims.nbDims != 4 || input_dims.d[0] != 1 || input_dims.d[1] != 5 ||
            input_dims.d[2] != 480 || input_dims.d[3] != 640 ||
            output_dims.nbDims != 3 || output_dims.d[0] != 1 ||
            output_dims.d[1] != 5 || output_dims.d[2] != 6300) {
            throw std::runtime_error("engine must have input 1x5x480x640 and output 1x5x6300");
        }
        input_h_                         = input_dims.d[2];
        input_w_                         = input_dims.d[3];
        output_channels_                 = output_dims.d[1];
        output_count_                    = output_dims.d[2];
        input_size_                      = 1;
        for (int i = 0; i < input_dims.nbDims; ++i) {
            input_size_ *= input_dims.d[i];
        }
        output_size_ = 1;
        for (int i = 0; i < output_dims.nbDims; ++i) {
            output_size_ *= output_dims.d[i];
        }

        host_input_.resize(input_size_);
        host_output_.resize(output_size_);
#if NV_TENSORRT_MAJOR >= 10
        device_buffers_.resize(2, nullptr);
        input_index_ = 0;
        output_index_ = 1;
#else
        device_buffers_.resize(engine_->getNbBindings(), nullptr);
#endif
        checkCuda(
            cudaMalloc(&device_buffers_[input_index_], input_size_ * sizeof(float)),
            "cudaMalloc input");
        checkCuda(
            cudaMalloc(&device_buffers_[output_index_], output_size_ * sizeof(float)),
            "cudaMalloc output");
        checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

        ROS_INFO_STREAM(
            "TensorRT engine loaded: input=1x5x" << input_h_ << "x" << input_w_ << ", output=1x"
                                                 << output_channels_ << "x" << output_count_);
    }

    bool projectWorldPointToOrientedImage(
        const cv::Point3d& world_point, const ros::Time& stamp,
        const sensor_msgs::CameraInfo& camera_info,
        cv::Point2f& pixel, double& optical_depth) const {
        nav_msgs::Odometry odom;
        if (!interpolateOdometry(stamp, odom)) return false;
        return projectWorldPointWithOdom(
            world_point, odom, camera_info, pixel, optical_depth);
    }

    bool projectWorldPointWithOdom(
        const cv::Point3d& world_point, const nav_msgs::Odometry& odom,
        const sensor_msgs::CameraInfo& camera_info,
        cv::Point2f& pixel, double& optical_depth) const {
        const geometry_msgs::Quaternion& q_msg = odom.pose.pose.orientation;
        tf2::Quaternion q_world_body(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
        if (q_world_body.length2() < 1e-12) return false;
        q_world_body.normalize();
        const geometry_msgs::Point& p_msg = odom.pose.pose.position;
        const tf2::Vector3 point_world(world_point.x, world_point.y, world_point.z);
        const tf2::Vector3 body_position(p_msg.x, p_msg.y, p_msg.z);
        const tf2::Vector3 point_body =
            tf2::Matrix3x3(q_world_body).transpose() * (point_world - body_position);
        const tf2::Vector3 point_camera =
            camera_rotation_body_depth_.transpose() *
            (point_body - camera_translation_body_depth_);
        optical_depth = point_camera.z();
        if (!(optical_depth > target_min_depth_) ||
            optical_depth > lidar_tracker_config_.max_range) return false;
        const double fx = camera_info.K[0];
        const double fy = camera_info.K[4];
        const double cx = camera_info.K[2];
        const double cy = camera_info.K[5];
        if (!(fx > 0.0) || !(fy > 0.0)) return false;
        double u = fx * point_camera.x() / optical_depth + cx;
        double v = fy * point_camera.y() / optical_depth + cy;
        const int width = static_cast<int>(camera_info.width);
        const int height = static_cast<int>(camera_info.height);
        if (rotate_input_180_) {
            u = width - 1.0 - u;
            v = height - 1.0 - v;
        }
        if (!std::isfinite(u) || !std::isfinite(v) || u < 0.0 || v < 0.0 ||
            u >= width || v >= height) return false;
        pixel = cv::Point2f(static_cast<float>(u), static_cast<float>(v));
        return true;
    }

    bool confirmLidarIdentityFromYolo(
        const std_msgs::Header& header,
        const std::vector<Detection>& detections) {
        sensor_msgs::CameraInfo camera_info;
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            if (!has_camera_info_) return false;
            camera_info = latest_camera_info_;
        }
        std::vector<yolo_trt_detector::LidarPoint> lidar_points;
        ros::Time lidar_stamp;
        {
            std::lock_guard<std::mutex> lock(lidar_points_mutex_);
            lidar_points = latest_lidar_points_;
            lidar_stamp = latest_lidar_points_stamp_;
        }
        const ros::Time effective_image_stamp =
            header.stamp.isZero() ? ros::Time::now() : header.stamp;
        const double image_stamp = effective_image_stamp.toSec();
        if (lidar_points.empty() || lidar_stamp.isZero() ||
            std::abs((effective_image_stamp - lidar_stamp).toSec()) >
                lidar_candidate_max_age_) {
            lidar_tracker_.noteVisualFrameWithoutIdentity(image_stamp);
            return false;
        }
        nav_msgs::Odometry image_odom;
        if (!interpolateOdometry(effective_image_stamp, image_odom)) {
            lidar_tracker_.noteVisualFrameWithoutIdentity(image_stamp);
            return false;
        }
        const yolo_trt_detector::LidarTrackEstimate current =
            lidar_tracker_.estimate(image_stamp);
        struct ProjectedMatch {
            yolo_trt_detector::VisualLidarMatchCandidate selection;
            cv::Point2f pixel;
            double depth = 0.0;
            yolo_trt_detector::LidarCluster cluster;
        };
        std::vector<size_t> eligible_detections;
        const double required_score = current.identity_confirmed
            ? track_update_confidence_ : track_init_confidence_;
        for (size_t detection_index = 0;
             detection_index < detections.size(); ++detection_index) {
            const Detection& detection = detections[detection_index];
            if (detection.class_id == target_class_id_ &&
                detection.score >= required_score) {
                eligible_detections.push_back(detection_index);
            }
        }

        std::vector<yolo_trt_detector::ProjectedLidarPoint> projected_points;
        projected_points.reserve(lidar_points.size());
        for (const auto& lidar_point : lidar_points) {
            cv::Point2f pixel;
            double depth = 0.0;
            const cv::Point3d position(lidar_point.x, lidar_point.y, lidar_point.z);
            if (!projectWorldPointWithOdom(
                    position, image_odom, camera_info, pixel, depth)) continue;
            yolo_trt_detector::ProjectedLidarPoint projected;
            projected.position = position;
            projected.pixel = cv::Point2d(pixel.x, pixel.y);
            projected.optical_depth = depth;
            projected_points.push_back(projected);
        }

        std::vector<ProjectedMatch> projected_matches;
        size_t frustum_cluster_count = 0;
        for (const size_t detection_index : eligible_detections) {
            const Detection& detection = detections[detection_index];
            const int margin_x = static_cast<int>(std::round(
                detection.box.width * lidar_visual_match_margin_));
            const int margin_y = static_cast<int>(std::round(
                detection.box.height * lidar_visual_match_margin_));
            const cv::Rect expanded(
                detection.box.x - margin_x, detection.box.y - margin_y,
                detection.box.width + 2 * margin_x,
                detection.box.height + 2 * margin_y);
            const auto frustum_clusters =
                yolo_trt_detector::clusterPointsInsideDetection(
                    projected_points, expanded, frustum_lidar_config_);
            frustum_cluster_count += frustum_clusters.size();
            const int selected_cluster =
                yolo_trt_detector::selectForegroundFrustumCluster(
                    frustum_clusters, expanded, frustum_lidar_config_);
            if (selected_cluster < 0) continue;
            const auto& candidate = frustum_clusters[selected_cluster];
            double rank = candidate.normalized_center_distance;
            if (current.valid) {
                const cv::Point3d difference =
                    candidate.cluster.centroid - current.position;
                rank += 0.5 * std::sqrt(difference.dot(difference));
            }
            ProjectedMatch match;
            match.selection = {
                detection_index, static_cast<size_t>(selected_cluster),
                candidate.normalized_center_distance, rank};
            match.pixel = cv::Point2f(
                static_cast<float>(candidate.pixel_centroid.x),
                static_cast<float>(candidate.pixel_centroid.y));
            match.depth = candidate.optical_depth;
            match.cluster = candidate.cluster;
            projected_matches.push_back(match);
        }

        std::vector<yolo_trt_detector::VisualLidarMatchCandidate> selections;
        selections.reserve(projected_matches.size());
        for (const auto& match : projected_matches) {
            selections.push_back(match.selection);
        }
        const int selected_index = yolo_trt_detector::selectVisualLidarMatch(
            selections, eligible_detections.size(), current.identity_confirmed,
            visual_lidar_match_config_);
        if (selected_index < 0) {
            lidar_tracker_.noteVisualFrameWithoutIdentity(image_stamp);
            const auto state = lidar_tracker_.estimate(image_stamp);
            if (state.state == yolo_trt_detector::LidarTrackState::kTentative) {
                track_state_ = TrackState::TENTATIVE;
            }
            ROS_WARN_STREAM_THROTTLE(
                1.0, "Rejecting ambiguous visual/LiDAR identity frame: detections="
                << eligible_detections.size() << " projected_pairs="
                << projected_matches.size() << " frustum_clusters="
                << frustum_cluster_count);
            if (!current.identity_confirmed) track_visual_box_valid_ = false;
            return false;
        }
        const ProjectedMatch& selected = projected_matches[selected_index];
        const Detection* best_detection =
            &detections[selected.selection.detection_index];
        const yolo_trt_detector::LidarCluster* best_cluster = &selected.cluster;
        const cv::Point2f best_pixel = selected.pixel;
        const double best_depth = selected.depth;
        const cv::Point2f previous_center = track_visual_center_;
        const ros::Time previous_stamp = last_track_visual_stamp_;
        track_visual_box_ = best_detection->box;
        track_visual_center_ = best_pixel;
        track_visual_box_valid_ = true;
        track_visual_score_ = best_detection->score;
        track_visual_depth_ = static_cast<float>(best_depth);
        track_visual_depth_mad_ = 0.0f;
        track_visual_depth_samples_ = best_cluster->point_count;
        track_visual_position_.setValue(
            best_cluster->centroid.x, best_cluster->centroid.y,
            best_cluster->centroid.z);
        track_visual_frame_ = frame_id_.empty() ? global_frame_id_ : frame_id_;
        last_track_visual_stamp_ = effective_image_stamp;
        if (!lidar_tracker_.confirmIdentity(
                *best_cluster, best_detection->score, image_stamp)) {
            const auto state = lidar_tracker_.estimate(image_stamp);
            if (state.state == yolo_trt_detector::LidarTrackState::kTentative) {
                track_state_ = TrackState::TENTATIVE;
            }
            return false;
        }

        if (!previous_stamp.isZero() && header.stamp > previous_stamp) {
            const double dt = (header.stamp - previous_stamp).toSec();
            track_visual_velocity_px_ = (best_pixel - previous_center) /
                static_cast<float>(std::max(1e-3, dt));
        } else {
            track_visual_velocity_px_ = cv::Point2f(0.0f, 0.0f);
        }
        last_yolo_confirmation_stamp_ = effective_image_stamp;
        last_confirmed_yolo_box_ = best_detection->box;
        last_confirmed_yolo_box_valid_ = true;
        track_state_ = TrackState::CONFIRMED;
        publishPoint(
            raw_target_point_pub_, header, track_visual_frame_,
            track_visual_position_);
        return true;
    }

    void updateLidarProjectedVisualBox(
        const std_msgs::Header& header, const cv::Size& image_size) {
        if (!enable_lidar_target_tracking_) return;
        if (!last_yolo_confirmation_stamp_.isZero() &&
            (header.stamp - last_yolo_confirmation_stamp_).toSec() < 0.20) return;
        const double stamp = header.stamp.isZero()
            ? ros::Time::now().toSec() : header.stamp.toSec();
        const auto estimate = lidar_tracker_.estimate(stamp);
        if (!estimate.valid) return;
        sensor_msgs::CameraInfo camera_info;
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            if (!has_camera_info_) return;
            camera_info = latest_camera_info_;
        }
        cv::Point2f pixel;
        double depth = 0.0;
        if (!projectWorldPointToOrientedImage(
                estimate.position, header.stamp, camera_info, pixel, depth)) return;
        const double focal = 0.5 * (camera_info.K[0] + camera_info.K[4]);
        const int side = std::max(8, std::min(
            std::max(image_size.width, image_size.height),
            static_cast<int>(std::round(
                focal * lidar_target_physical_size_ / depth))));
        track_visual_box_ = cv::Rect(
            static_cast<int>(std::round(pixel.x - side * 0.5)),
            static_cast<int>(std::round(pixel.y - side * 0.5)), side, side);
        track_visual_center_ = pixel;
        track_visual_velocity_px_ = cv::Point2f(0.0f, 0.0f);
        track_visual_box_valid_ = true;
        track_visual_depth_ = static_cast<float>(depth);
        track_visual_depth_samples_ = 0;
        track_visual_position_.setValue(
            estimate.position.x, estimate.position.y, estimate.position.z);
        track_visual_frame_ = frame_id_.empty() ? global_frame_id_ : frame_id_;
        last_track_visual_stamp_ = header.stamp;
    }

    void publishLidarTrackedState(
        const std_msgs::Header& source_header,
        const yolo_trt_detector::LidarTrackEstimate& estimate) {
        if (!estimate.valid) {
            track_state_ = estimate.state ==
                    yolo_trt_detector::LidarTrackState::kTentative
                ? TrackState::TENTATIVE : TrackState::LOST;
            if (track_state_ == TrackState::LOST) {
                track_visual_box_valid_ = false;
                last_confirmed_yolo_box_valid_ = false;
            }
            publishTrackerStatus();
            return;
        }
        track_state_ = estimate.state == yolo_trt_detector::LidarTrackState::kCoasting
            ? TrackState::COASTING : TrackState::CONFIRMED;
        std_msgs::Header header = source_header;
        if (header.stamp.isZero()) header.stamp = ros::Time::now();
        const std::string output_frame = frame_id_.empty()
            ? (header.frame_id.empty() ? global_frame_id_ : header.frame_id)
            : frame_id_;
        header.frame_id = output_frame;
        if (!target_publish_limiter_.shouldPublish(header.stamp.toSec())) {
            publishTrackerStatus();
            return;
        }
        const tf2::Vector3 position(
            estimate.position.x, estimate.position.y, estimate.position.z);
        publishPoint(target_point_pub_, header, output_frame, position);
        publishPlannerGoal(header, output_frame, position);
        nav_msgs::Odometry tracked;
        tracked.header = header;
        tracked.child_frame_id = "tracked_target";
        tracked.pose.pose.position.x = estimate.position.x;
        tracked.pose.pose.position.y = estimate.position.y;
        tracked.pose.pose.position.z = estimate.position.z;
        tracked.pose.pose.orientation.w = 1.0;
        tracked.twist.twist.linear.x = estimate.velocity.x;
        tracked.twist.twist.linear.y = estimate.velocity.y;
        tracked.twist.twist.linear.z = estimate.velocity.z;
        tracked.pose.covariance[0] = estimate.position_variance[0];
        tracked.pose.covariance[7] = estimate.position_variance[1];
        tracked.pose.covariance[14] = estimate.position_variance[2];
        tracked.twist.covariance[0] = estimate.velocity_variance[0];
        tracked.twist.covariance[7] = estimate.velocity_variance[1];
        tracked.twist.covariance[14] = estimate.velocity_variance[2];
        tracked_odom_pub_.publish(tracked);
        publishTrackerStatus();
    }

    void lidarCloudCallback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        if (!enable_lidar_target_tracking_) return;
        nav_msgs::Odometry odom;
        if (!interpolateOdometry(msg->header.stamp, odom)) {
            ROS_WARN_THROTTLE(1.0, "LiDAR target tracker is waiting for synchronized odometry.");
            return;
        }
        std::vector<yolo_trt_detector::LidarPoint> points;
        points.reserve(static_cast<size_t>(msg->width) * msg->height);
        try {
            sensor_msgs::PointCloud2ConstIterator<float> x(*msg, "x");
            sensor_msgs::PointCloud2ConstIterator<float> y(*msg, "y");
            sensor_msgs::PointCloud2ConstIterator<float> z(*msg, "z");
            for (; x != x.end(); ++x, ++y, ++z) {
                yolo_trt_detector::LidarPoint point;
                point.x = *x;
                point.y = *y;
                point.z = *z;
                points.push_back(point);
            }
        } catch (const std::runtime_error& error) {
            ROS_WARN_STREAM_THROTTLE(
                1.0, "Invalid LiDAR point cloud: " << error.what());
            return;
        }
        const geometry_msgs::Point& p = odom.pose.pose.position;
        const ros::Time effective_cloud_stamp =
            msg->header.stamp.isZero() ? ros::Time::now() : msg->header.stamp;
        const double stamp = effective_cloud_stamp.toSec();
        {
            std::lock_guard<std::mutex> lock(lidar_points_mutex_);
            latest_lidar_points_ = points;
            latest_lidar_points_stamp_ = effective_cloud_stamp;
        }
        yolo_trt_detector::LidarTargetTracker::CandidateFilter visual_filter;
        const auto before_update = lidar_tracker_.estimate(stamp);
        const double visual_age = last_yolo_confirmation_stamp_.isZero()
            ? std::numeric_limits<double>::infinity()
            : (effective_cloud_stamp - last_yolo_confirmation_stamp_).toSec();
        if (before_update.identity_confirmed && last_confirmed_yolo_box_valid_ &&
            visual_age >= 0.0 && visual_age <= lidar_visual_filter_max_age_) {
            sensor_msgs::CameraInfo camera_info;
            bool camera_info_available = false;
            {
                std::lock_guard<std::mutex> lock(depth_mutex_);
                camera_info = latest_camera_info_;
                camera_info_available = has_camera_info_;
            }
            const int margin_x = static_cast<int>(std::round(
                last_confirmed_yolo_box_.width * lidar_visual_filter_margin_));
            const int margin_y = static_cast<int>(std::round(
                last_confirmed_yolo_box_.height * lidar_visual_filter_margin_));
            const cv::Rect allowed_box(
                last_confirmed_yolo_box_.x - margin_x,
                last_confirmed_yolo_box_.y - margin_y,
                last_confirmed_yolo_box_.width + 2 * margin_x,
                last_confirmed_yolo_box_.height + 2 * margin_y);
            if (camera_info_available) {
                visual_filter = [this, odom, camera_info, allowed_box](
                    const yolo_trt_detector::LidarCluster& candidate) {
                    cv::Point2f pixel;
                    double depth = 0.0;
                    return projectWorldPointWithOdom(
                               candidate.centroid, odom, camera_info,
                               pixel, depth) && allowed_box.contains(pixel);
                };
            }
        }
        last_lidar_estimate_ = lidar_tracker_.processCloud(
            points, cv::Point3d(p.x, p.y, p.z), stamp, visual_filter);
        publishLidarTrackedState(msg->header, last_lidar_estimate_);
        ROS_INFO_STREAM_THROTTLE(
            2.0, "LiDAR target: candidates=" << lidar_tracker_.candidates().size()
            << " state=" << trackStateName()
            << " visual_age=" << last_lidar_estimate_.last_visual_confirmation_age
            << " lidar_age=" << last_lidar_estimate_.last_lidar_update_age
            << " IMM=[" << last_lidar_estimate_.model_probability[0] << ", "
            << last_lidar_estimate_.model_probability[1] << "]");
    }

    void pairedCallback(
        const sensor_msgs::ImageConstPtr& msg,
        const sensor_msgs::ImageConstPtr& depth_msg) {
        cv_bridge::CvImageConstPtr cv_ptr;
        cv_bridge::CvImageConstPtr depth_ptr;
        try {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            depth_ptr = cv_bridge::toCvShare(depth_msg);
        } catch (const cv_bridge::Exception& e) {
            ROS_WARN_STREAM_THROTTLE(1.0, "cv_bridge exception: " << e.what());
            return;
        }
        if (depth_msg->encoding != sensor_msgs::image_encodings::TYPE_16UC1 &&
            depth_msg->encoding != sensor_msgs::image_encodings::MONO16) {
            ROS_WARN_STREAM_THROTTLE(1.0, "RGB-D model requires uint16 depth, got " << depth_msg->encoding);
            return;
        }
        if (depth_ptr->image.type() != CV_16UC1 || cv_ptr->image.size() != depth_ptr->image.size()) {
            ROS_WARN_STREAM_THROTTLE(1.0, "color/depth types or sizes do not match");
            return;
        }
        if (std::abs((msg->header.stamp - depth_msg->header.stamp).toSec()) > max_pair_dt_) {
            ROS_WARN_STREAM_THROTTLE(1.0, "color/depth timestamps are too far apart");
            return;
        }

        const ros::Time now = ros::Time::now();
        if (detect_hz_ > 0 && (now - last_infer_time_).toSec() < 1.0 / detect_hz_) {
            return;
        }
        last_infer_time_ = now;

        const cv::Mat sensor_depth = depth_ptr->image;
        yolo_trt_detector::OrientedRgbdFrame oriented =
            yolo_trt_detector::orientRgbdInputs(
                cv_ptr->image, sensor_depth, rotate_input_180_);
        cv::Mat image = oriented.image;
        cv::Mat depth = oriented.depth;
        LetterboxInfo letterbox = preprocess(image, depth);
        if (!infer()) {
            ROS_WARN_STREAM_THROTTLE(1.0, "TensorRT inference failed");
            return;
        }

        std::vector<Detection> detections = postprocess(image.size(), letterbox);
        if (publish_target_point_) {
            {
                std::lock_guard<std::mutex> lock(depth_mutex_);
                // Keep the sensor-native depth image. Detection coordinates are mapped back
                // before depth lookup, so CameraInfo and the calibrated extrinsic stay unchanged.
                latest_depth_ = sensor_depth.clone();
                latest_depth_encoding_ = depth_msg->encoding;
                latest_depth_stamp_ = depth_msg->header.stamp;
            }
            publishTargetPoint(msg->header, detections);
        }
        logRejectionSummary();
        updateLidarProjectedVisualBox(msg->header, image.size());
        publishTargetMask(msg->header, image.size());
        drawDetections(image, detections, msg->header.stamp);
        publishAnnotatedImage(msg->header, image);
        publishMarkers(msg->header, image.size(), detections);
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg) {
        std::lock_guard<std::mutex> lock(depth_mutex_);
        latest_camera_info_ = *msg;
        has_camera_info_    = true;
    }

    void odomCallback(const nav_msgs::OdometryConstPtr& msg) {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        latest_odom_ = *msg;
        has_odom_    = true;
        if (!odom_buffer_.empty() && msg->header.stamp < odom_buffer_.back().header.stamp) {
            odom_buffer_.clear();
        }
        odom_buffer_.push_back(*msg);
        const ros::Time oldest_allowed = msg->header.stamp - ros::Duration(odom_buffer_duration_);
        while (odom_buffer_.size() > 2 && odom_buffer_.front().header.stamp < oldest_allowed) {
            odom_buffer_.pop_front();
        }
    }

    void extrinsicCallback(const nav_msgs::OdometryConstPtr& msg) {
        std::lock_guard<std::mutex> lock(extrinsic_mutex_);
        latest_extrinsic_ = *msg;
        has_extrinsic_    = true;
    }

    LetterboxInfo preprocess(const cv::Mat& image, const cv::Mat& depth) {
        LetterboxInfo info;
        // Match the float32 RGB-D-confidence training configuration. Raw depth is in millimeters.
        constexpr float depth_min = 400.0f;
        constexpr float depth_max = 10000.0f;
        constexpr float confidence_near_full = 600.0f;
        constexpr float confidence_far_full = 6000.0f;
        cv::Mat depth_f32(depth.size(), CV_32FC1);
        cv::Mat confidence_f32(depth.size(), CV_32FC1);
        for (int y = 0; y < depth.rows; ++y) {
            const uint16_t* raw = depth.ptr<uint16_t>(y);
            float* normalized = depth_f32.ptr<float>(y);
            float* confidence = confidence_f32.ptr<float>(y);
            for (int x = 0; x < depth.cols; ++x) {
                const bool is_valid = raw[x] != 0 && raw[x] != 65535;
                const float raw_mm = static_cast<float>(raw[x]);
                const float near = std::max(0.0f, std::min(
                    1.0f, (raw_mm - depth_min) / (confidence_near_full - depth_min)));
                const float far = std::max(0.0f, std::min(
                    1.0f, (depth_max - raw_mm) / (depth_max - confidence_far_full)));
                confidence[x] = is_valid ? std::min(near, far) : 0.0f;
                const float clipped = std::max(depth_min, std::min(depth_max, raw_mm));
                // Keep continuous depth where confidence is positive; the network applies the gate.
                normalized[x] = confidence[x] > 0.0f ? (clipped - depth_min) / (depth_max - depth_min) : 0.0f;
            }
        }

        info.scale = std::min(1.0f, std::min(
            static_cast<float>(input_w_) / image.cols, static_cast<float>(input_h_) / image.rows));
        const int resized_w = static_cast<int>(std::round(image.cols * info.scale));
        const int resized_h = static_cast<int>(std::round(image.rows * info.scale));
        const int pad_x = (input_w_ - resized_w) / 2;
        const int pad_y = (input_h_ - resized_h) / 2;
        info.pad_x = static_cast<float>(pad_x);
        info.pad_y = static_cast<float>(pad_y);

        cv::Mat rgb;
        cv::cvtColor(image, rgb, cv::COLOR_BGR2RGB);
        cv::Mat rgb_resized, depth_resized, confidence_resized;
        cv::resize(rgb, rgb_resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_LINEAR);
        cv::resize(depth_f32, depth_resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_NEAREST);
        cv::resize(confidence_f32, confidence_resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_NEAREST);

        const cv::Rect content(pad_x, pad_y, resized_w, resized_h);
        cv::Mat rgb_padded(input_h_, input_w_, CV_8UC3, cv::Scalar::all(114));
        cv::Mat depth_padded(input_h_, input_w_, CV_32FC1, cv::Scalar::all(0));
        cv::Mat confidence_padded(input_h_, input_w_, CV_32FC1, cv::Scalar::all(0));
        rgb_resized.copyTo(rgb_padded(content));
        depth_resized.copyTo(depth_padded(content));
        confidence_resized.copyTo(confidence_padded(content));

        const int channel_size = input_h_ * input_w_;
        for (int y = 0; y < input_h_; ++y) {
            const cv::Vec3b* row = rgb_padded.ptr<cv::Vec3b>(y);
            const float* depth_row = depth_padded.ptr<float>(y);
            const float* confidence_row = confidence_padded.ptr<float>(y);
            for (int x = 0; x < input_w_; ++x) {
                for (int c = 0; c < 3; ++c) {
                    host_input_[c * channel_size + y * input_w_ + x] = row[x][c] / 255.0f;
                }
                // Depth and confidence are already in [0, 1]; only RGB needs division by 255.
                host_input_[3 * channel_size + y * input_w_ + x] = depth_row[x];
                host_input_[4 * channel_size + y * input_w_ + x] = confidence_row[x];
            }
        }
        return info;
    }

    bool infer() {
        checkCuda(
            cudaMemcpyAsync(
                device_buffers_[input_index_], host_input_.data(), input_size_ * sizeof(float),
                cudaMemcpyHostToDevice, stream_),
            "cudaMemcpyAsync input");
#if NV_TENSORRT_MAJOR >= 10
        const bool addresses_ok =
            context_->setTensorAddress("images", device_buffers_[input_index_]) &&
            context_->setTensorAddress("output0", device_buffers_[output_index_]);
        const bool ok = addresses_ok && context_->enqueueV3(stream_);
#else
        const bool ok = context_->enqueueV2(device_buffers_.data(), stream_, nullptr);
#endif
        if (!ok) {
            return false;
        }
        checkCuda(
            cudaMemcpyAsync(
                host_output_.data(), device_buffers_[output_index_], output_size_ * sizeof(float),
                cudaMemcpyDeviceToHost, stream_),
            "cudaMemcpyAsync output");
        checkCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
        return true;
    }

    std::vector<Detection> postprocess(const cv::Size& image_size, const LetterboxInfo& letterbox) {
        std::vector<Detection> detections;
        if (output_channels_ < 5) {
            return detections;
        }

        for (int i = 0; i < output_count_; ++i) {
            const float score = host_output_[4 * output_count_ + i];
            if (!std::isfinite(score) || score < conf_threshold_) {
                ++rejection_counters_.confidence;
                continue;
            }

            const float cx = host_output_[0 * output_count_ + i];
            const float cy = host_output_[1 * output_count_ + i];
            const float w  = host_output_[2 * output_count_ + i];
            const float h  = host_output_[3 * output_count_ + i];
            if (!std::isfinite(cx) || !std::isfinite(cy) ||
                !std::isfinite(w) || !std::isfinite(h)) {
                continue;
            }

            float x1 = (cx - w * 0.5f - letterbox.pad_x) / letterbox.scale;
            float y1 = (cy - h * 0.5f - letterbox.pad_y) / letterbox.scale;
            float x2 = (cx + w * 0.5f - letterbox.pad_x) / letterbox.scale;
            float y2 = (cy + h * 0.5f - letterbox.pad_y) / letterbox.scale;

            x1 = std::max(0.0f, std::min(x1, static_cast<float>(image_size.width - 1)));
            y1 = std::max(0.0f, std::min(y1, static_cast<float>(image_size.height - 1)));
            x2 = std::max(0.0f, std::min(x2, static_cast<float>(image_size.width - 1)));
            y2 = std::max(0.0f, std::min(y2, static_cast<float>(image_size.height - 1)));

            const int box_w = static_cast<int>(x2 - x1);
            const int box_h = static_cast<int>(y2 - y1);
            if (box_w <= 1 || box_h <= 1) {
                continue;
            }

            Detection det;
            det.box   = cv::Rect(static_cast<int>(x1), static_cast<int>(y1), box_w, box_h);
            det.score = score;
            detections.push_back(det);
        }
        detections = nms(std::move(detections), nms_threshold_);
        if (detections.size() > 300) {
            detections.resize(300);
        }
        return detections;
    }

    void drawDetections(
        cv::Mat& image, const std::vector<Detection>& detections, const ros::Time& stamp) {
        // Draw every post-NMS candidate in gray. The selected track is overlaid below with a
        // state-specific color, so an operator can distinguish detections from the control target.
        for (const Detection& det : detections) {
            cv::rectangle(image, det.box, cv::Scalar(140, 140, 140), 1);
            char label[64];
            std::snprintf(label, sizeof(label), "candidate %.2f", det.score);
            cv::putText(
                image, label, cv::Point(det.box.x, std::max(18, det.box.y - 4)),
                cv::FONT_HERSHEY_SIMPLEX, 0.45, cv::Scalar(170, 170, 170), 1, cv::LINE_AA);
        }

        if (!enable_target_tracking_ || track_state_ == TrackState::LOST ||
            !track_visual_box_valid_) {
            return;
        }

        cv::Rect tracked_box = predictedTrackBox(stamp);
        tracked_box &= cv::Rect(0, 0, image.cols, image.rows);
        if (tracked_box.width <= 1 || tracked_box.height <= 1) {
            return;
        }

        cv::Scalar color(0, 255, 0);  // CONFIRMED: green
        if (track_state_ == TrackState::TENTATIVE) {
            color = cv::Scalar(0, 255, 255);  // yellow
        } else if (track_state_ == TrackState::COASTING) {
            color = cv::Scalar(0, 165, 255);  // orange
        }
        cv::rectangle(image, tracked_box, color, 3);

        char state_line[160];
        std::snprintf(
            state_line, sizeof(state_line), "%s conf=%.2f depth=%.2fm MAD=%.3fm",
            trackStateName(), track_visual_score_, track_visual_depth_, track_visual_depth_mad_);
        char position_line[192];
        std::snprintf(
            position_line, sizeof(position_line), "%s=[%.2f %.2f %.2f] samples=%d",
            track_visual_frame_.empty() ? "point" : track_visual_frame_.c_str(),
            track_visual_position_.x(), track_visual_position_.y(),
            track_visual_position_.z(), track_visual_depth_samples_);

        const int text_x = std::max(2, tracked_box.x);
        const int first_y = std::max(20, tracked_box.y - 26);
        const int second_y = std::max(38, tracked_box.y - 8);
        drawTextWithBackground(image, state_line, cv::Point(text_x, first_y), color);
        drawTextWithBackground(image, position_line, cv::Point(text_x, second_y), color);
    }

    void publishTargetMask(const std_msgs::Header& header, const cv::Size& image_size) {
        cv::Mat mask(image_size, CV_8UC1, cv::Scalar(0));
        const bool track_is_usable =
            track_state_ == TrackState::CONFIRMED || track_state_ == TrackState::COASTING;
        // The tracked target is a navigation goal, not an obstacle.  Repair
        // its depth footprint at every valid distance.  The tracker state is
        // still required so unrelated detections/background are never masked.
        if (track_is_usable && track_visual_box_valid_) {
            cv::Rect box = predictedTrackBox(header.stamp);
            const double scale = target_mask_expand_scale_;
            const cv::Point2d center(
                box.x + 0.5 * box.width, box.y + 0.5 * box.height);
            const int width = std::max(1, static_cast<int>(std::round(box.width * scale)));
            const int height = std::max(1, static_cast<int>(std::round(box.height * scale)));
            box = cv::Rect(
                static_cast<int>(std::round(center.x - 0.5 * width)),
                static_cast<int>(std::round(center.y - 0.5 * height)), width, height);
            box &= cv::Rect(0, 0, image_size.width, image_size.height);
            if (box.width > 0 && box.height > 0) {
                cv::rectangle(mask, box, cv::Scalar(255), cv::FILLED);
            }
        }

        cv_bridge::CvImage output;
        output.header = header;
        output.encoding = sensor_msgs::image_encodings::MONO8;
        output.image = mask;
        target_mask_pub_.publish(output.toImageMsg());
    }

    static void drawTextWithBackground(
        cv::Mat& image, const std::string& text, const cv::Point& origin,
        const cv::Scalar& color) {
        int baseline = 0;
        const double font_scale = 0.48;
        const int thickness = 1;
        const cv::Size text_size = cv::getTextSize(
            text, cv::FONT_HERSHEY_SIMPLEX, font_scale, thickness, &baseline);
        const cv::Rect background(
            std::max(0, origin.x - 2), std::max(0, origin.y - text_size.height - 3),
            std::min(text_size.width + 4, image.cols - std::max(0, origin.x - 2)),
            std::min(text_size.height + baseline + 5,
                     image.rows - std::max(0, origin.y - text_size.height - 3)));
        if (background.width > 0 && background.height > 0) {
            cv::rectangle(image, background, cv::Scalar(20, 20, 20), cv::FILLED);
        }
        cv::putText(
            image, text, origin, cv::FONT_HERSHEY_SIMPLEX, font_scale,
            color, thickness, cv::LINE_AA);
    }

    void publishAnnotatedImage(const std_msgs::Header& header, const cv::Mat& image) {
        cv_bridge::CvImage out;
        out.header   = header;
        out.encoding = sensor_msgs::image_encodings::BGR8;
        out.image    = image;
        annotated_pub_.publish(out.toImageMsg());
    }

    void publishMarkers(
        const std_msgs::Header& header, const cv::Size& image_size,
        const std::vector<Detection>& detections) {
        visualization_msgs::MarkerArray markers;
        visualization_msgs::Marker clear;
        clear.action = visualization_msgs::Marker::DELETEALL;
        markers.markers.push_back(clear);

        int id = 0;
        for (const Detection& det : detections) {
            visualization_msgs::Marker marker;
            marker.header          = header;
            marker.header.frame_id = frame_id_.empty() ? header.frame_id : frame_id_;
            marker.ns              = "yolo_trt";
            marker.id              = id++;
            marker.type            = visualization_msgs::Marker::TEXT_VIEW_FACING;
            marker.action          = visualization_msgs::Marker::ADD;
            marker.pose.position.x = (det.box.x + det.box.width * 0.5) / image_size.width - 0.5;
            marker.pose.position.y = (det.box.y + det.box.height * 0.5) / image_size.height - 0.5;
            marker.pose.position.z = 1.0;
            marker.pose.orientation.w = 1.0;
            marker.scale.z            = 0.12;
            marker.color.r            = 0.0f;
            marker.color.g            = 1.0f;
            marker.color.b            = 0.0f;
            marker.color.a            = 1.0f;
            marker.text               = "target " + std::to_string(det.score).substr(0, 4);
            markers.markers.push_back(marker);
        }
        marker_pub_.publish(markers);
    }

    void publishTargetPoint(
        const std_msgs::Header& header, const std::vector<Detection>& detections) {
        if (enable_lidar_target_tracking_) {
            // Image detections only select/confirm a projected LiDAR cluster.
            // D455 depth is deliberately not used as a 3D measurement in this
            // mode, so losing the image cannot stop LiDAR state updates.
            confirmLidarIdentityFromYolo(header, detections);
            publishTrackerStatus();
            return;
        }
        cv::Mat depth;
        std::string depth_encoding;
        sensor_msgs::CameraInfo camera_info;
        {
            std::lock_guard<std::mutex> lock(depth_mutex_);
            if (latest_depth_.empty() || !has_camera_info_) {
                ROS_WARN_STREAM_THROTTLE(1.0, "waiting for depth image and camera info");
                return;
            }
            depth          = latest_depth_.clone();
            depth_encoding = latest_depth_encoding_;
            camera_info    = latest_camera_info_;
        }
        std::vector<TargetMeasurement> measurements;
        std::string output_frame = frame_id_.empty() ? "base_link" : frame_id_;
        for (const Detection& detection : detections) {
            if (detection.score < track_update_confidence_) {
                ++rejection_counters_.confidence;
                continue;
            }
            TargetMeasurement measurement;
            std::string measurement_frame;
            if (makeTargetMeasurement(
                    header, detection, depth, depth_encoding, camera_info,
                    measurement, measurement_frame)) {
                measurements.push_back(measurement);
                output_frame = measurement_frame;
            }
        }

        if (!enable_target_tracking_) {
            if (!measurements.empty()) {
                publishPoint(raw_target_point_pub_, header, output_frame, measurements.front().position);
                publishPoint(target_point_pub_, header, output_frame, measurements.front().position);
            }
            return;
        }
        updateTracker(header, output_frame, measurements);
    }

    bool makeTargetMeasurement(
        const std_msgs::Header& header, const Detection& detection, const cv::Mat& depth,
        const std::string& depth_encoding, const sensor_msgs::CameraInfo& camera_info,
        TargetMeasurement& measurement, std::string& output_frame) {
        const float oriented_u = detection.box.x + detection.box.width * 0.5f;
        const float oriented_v = detection.box.y + detection.box.height * 0.5f;
        const cv::Point2f sensor_pixel = yolo_trt_detector::sensorPixelFromOriented(
            oriented_u, oriented_v, depth.cols, depth.rows, rotate_input_180_);

        const DepthEstimate depth_estimate = robustDepth(
            depth, depth_encoding, sensor_pixel.x, sensor_pixel.y,
            detection.box.width, detection.box.height);
        if (!depth_estimate.valid) {
            if (depth_estimate.valid_samples < depth_min_samples_) {
                ++rejection_counters_.insufficient_depth;
            } else {
                ++rejection_counters_.excessive_mad;
            }
            return false;
        }

        const double fx = camera_info.K[0];
        const double fy = camera_info.K[4];
        const double cx = camera_info.K[2];
        const double cy = camera_info.K[5];
        if (!std::isfinite(fx) || !std::isfinite(fy) || fx <= 0.0 || fy <= 0.0) {
            ROS_WARN_STREAM_THROTTLE(1.0, "invalid camera intrinsics");
            return false;
        }

        const double z = depth_estimate.depth;
        const tf2::Vector3 target_camera(
            (sensor_pixel.x - cx) * z / fx, (sensor_pixel.y - cy) * z / fy, z);

        tf2::Matrix3x3 rotation_body_depth = camera_rotation_body_depth_;
        tf2::Vector3 translation_body_depth = camera_translation_body_depth_;
        if (use_vins_extrinsic_) {
            nav_msgs::Odometry extrinsic;
            {
                std::lock_guard<std::mutex> lock(extrinsic_mutex_);
                if (!has_extrinsic_) {
                    ROS_WARN_STREAM_THROTTLE(1.0, "waiting for dynamic camera extrinsic");
                    return false;
                }
                extrinsic = latest_extrinsic_;
            }
            const geometry_msgs::Quaternion& q_ext = extrinsic.pose.pose.orientation;
            tf2::Quaternion q_body_depth(q_ext.x, q_ext.y, q_ext.z, q_ext.w);
            if (q_body_depth.length2() < 1e-12) {
                ROS_WARN_STREAM_THROTTLE(1.0, "dynamic camera extrinsic has invalid quaternion");
                return false;
            }
            q_body_depth.normalize();
            rotation_body_depth = tf2::Matrix3x3(q_body_depth);
            const geometry_msgs::Point& t_ext = extrinsic.pose.pose.position;
            translation_body_depth.setValue(t_ext.x, t_ext.y, t_ext.z);
        }

        tf2::Vector3 target_output =
            rotation_body_depth * target_camera + translation_body_depth;
        output_frame = frame_id_.empty() ? "base_link" : frame_id_;
        if (output_world_frame_) {
            nav_msgs::Odometry odom;
            if (!interpolateOdometry(header.stamp, odom)) {
                ++rejection_counters_.odom_time_mismatch;
                ROS_WARN_STREAM_THROTTLE(1.0, "no odometry close to RGB-D timestamp");
                return false;
            }
            const geometry_msgs::Quaternion& q_msg = odom.pose.pose.orientation;
            tf2::Quaternion q_world_body(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
            if (q_world_body.length2() < 1e-12) {
                ROS_WARN_STREAM_THROTTLE(1.0, "odometry has invalid quaternion");
                return false;
            }
            q_world_body.normalize();
            const geometry_msgs::Point& p_msg = odom.pose.pose.position;
            target_output = tf2::Matrix3x3(q_world_body) * target_output +
                            tf2::Vector3(p_msg.x, p_msg.y, p_msg.z);
            output_frame = frame_id_.empty()
                               ? (odom.header.frame_id.empty() ? global_frame_id_
                                                               : odom.header.frame_id)
                               : frame_id_;
        }

        measurement.detection     = detection;
        measurement.position      = target_output;
        measurement.depth         = depth_estimate.depth;
        measurement.depth_mad     = depth_estimate.mad;
        measurement.depth_samples = depth_estimate.valid_samples;
        return true;
    }

    bool interpolateOdometry(const ros::Time& stamp, nav_msgs::Odometry& result) const {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        if (odom_buffer_.empty()) {
            return false;
        }
        if (stamp.isZero()) {
            result = odom_buffer_.back();
            return true;
        }
        if (stamp <= odom_buffer_.front().header.stamp) {
            if ((odom_buffer_.front().header.stamp - stamp).toSec() > odom_max_time_offset_) {
                return false;
            }
            result = odom_buffer_.front();
            return true;
        }
        if (stamp >= odom_buffer_.back().header.stamp) {
            if ((stamp - odom_buffer_.back().header.stamp).toSec() > odom_max_time_offset_) {
                return false;
            }
            result = odom_buffer_.back();
            return true;
        }

        for (size_t i = 1; i < odom_buffer_.size(); ++i) {
            const nav_msgs::Odometry& next = odom_buffer_[i];
            if (next.header.stamp < stamp) {
                continue;
            }
            const nav_msgs::Odometry& previous = odom_buffer_[i - 1];
            const double interval = (next.header.stamp - previous.header.stamp).toSec();
            if (interval <= 0.0) {
                return false;
            }
            const double alpha = std::max(
                0.0, std::min(1.0, (stamp - previous.header.stamp).toSec() / interval));
            result = previous;
            result.header.stamp = stamp;
            result.pose.pose.position.x =
                previous.pose.pose.position.x * (1.0 - alpha) + next.pose.pose.position.x * alpha;
            result.pose.pose.position.y =
                previous.pose.pose.position.y * (1.0 - alpha) + next.pose.pose.position.y * alpha;
            result.pose.pose.position.z =
                previous.pose.pose.position.z * (1.0 - alpha) + next.pose.pose.position.z * alpha;

            const geometry_msgs::Quaternion& q0_msg = previous.pose.pose.orientation;
            const geometry_msgs::Quaternion& q1_msg = next.pose.pose.orientation;
            tf2::Quaternion q0(q0_msg.x, q0_msg.y, q0_msg.z, q0_msg.w);
            tf2::Quaternion q1(q1_msg.x, q1_msg.y, q1_msg.z, q1_msg.w);
            if (q0.length2() < 1e-12 || q1.length2() < 1e-12) {
                return false;
            }
            q0.normalize();
            q1.normalize();
            const tf2::Quaternion q = q0.slerp(q1, alpha).normalized();
            result.pose.pose.orientation.x = q.x();
            result.pose.pose.orientation.y = q.y();
            result.pose.pose.orientation.z = q.z();
            result.pose.pose.orientation.w = q.w();
            return true;
        }
        return false;
    }

    static float median(std::vector<float> values) {
        if (values.empty()) {
            return 0.0f;
        }
        const size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        return values[middle];
    }

    DepthEstimate robustDepth(
        const cv::Mat& depth, const std::string& encoding, float u, float v,
        int box_width, int box_height) const {
        DepthEstimate result;
        const int center_x = static_cast<int>(std::round(u));
        const int center_y = static_cast<int>(std::round(v));
        const int half_width = std::max(
            std::max(0, depth_roi_radius_),
            static_cast<int>(std::round(box_width * depth_roi_scale_ * 0.5)));
        const int half_height = std::max(
            std::max(0, depth_roi_radius_),
            static_cast<int>(std::round(box_height * depth_roi_scale_ * 0.5)));

        std::vector<float> values;
        for (int y = std::max(0, center_y - half_height);
             y <= std::min(depth.rows - 1, center_y + half_height); ++y) {
            for (int x = std::max(0, center_x - half_width);
                 x <= std::min(depth.cols - 1, center_x + half_width); ++x) {
                float value = 0.0f;
                if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
                    encoding == sensor_msgs::image_encodings::MONO16) {
                    const uint16_t raw = depth.at<uint16_t>(y, x);
                    if (raw == 0 || raw == 65535) {
                        continue;
                    }
                    value = raw * depth_scale_;
                } else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
                    value = depth.at<float>(y, x);
                } else {
                    ROS_WARN_STREAM_THROTTLE(1.0, "unsupported depth encoding: " << encoding);
                    return result;
                }
                if (std::isfinite(value) && value >= target_min_depth_ &&
                    value <= target_max_depth_) {
                    values.push_back(value);
                }
            }
        }
        if (static_cast<int>(values.size()) < depth_min_samples_) {
            return result;
        }

        const float coarse_median = median(values);
        const float cluster_radius = std::max(
            static_cast<float>(depth_cluster_abs_),
            static_cast<float>(depth_cluster_rel_ * coarse_median));
        std::vector<float> cluster;
        cluster.reserve(values.size());
        for (const float value : values) {
            if (std::abs(value - coarse_median) <= cluster_radius) {
                cluster.push_back(value);
            }
        }
        if (static_cast<int>(cluster.size()) < depth_min_samples_) {
            return result;
        }

        result.depth = median(cluster);
        std::vector<float> absolute_deviations;
        absolute_deviations.reserve(cluster.size());
        for (const float value : cluster) {
            absolute_deviations.push_back(std::abs(value - result.depth));
        }
        result.mad = median(absolute_deviations);
        result.valid_samples = static_cast<int>(cluster.size());
        result.valid = std::isfinite(result.depth) && std::isfinite(result.mad) &&
                       result.mad <= depth_mad_max_;
        return result;
    }

    enum class TrackState { LOST, TENTATIVE, CONFIRMED, COASTING };

    const char* trackStateName() const {
        switch (track_state_) {
            case TrackState::TENTATIVE: return "TENTATIVE";
            case TrackState::CONFIRMED: return "CONFIRMED";
            case TrackState::COASTING: return "COASTING";
            default: return "LOST";
        }
    }

    void initializeTracker(const TargetMeasurement& measurement, const ros::Time& stamp) {
        tracker_ = cv::KalmanFilter(6, 3, 0, CV_32F);
        tracker_.measurementMatrix = cv::Mat::zeros(3, 6, CV_32F);
        tracker_.measurementMatrix.at<float>(0, 0) = 1.0f;
        tracker_.measurementMatrix.at<float>(1, 1) = 1.0f;
        tracker_.measurementMatrix.at<float>(2, 2) = 1.0f;
        tracker_.statePost = cv::Mat::zeros(6, 1, CV_32F);
        tracker_.statePost.at<float>(0) = measurement.position.x();
        tracker_.statePost.at<float>(1) = measurement.position.y();
        tracker_.statePost.at<float>(2) = measurement.position.z();
        tracker_.errorCovPost = cv::Mat::zeros(6, 6, CV_32F);
        const float position_variance =
            static_cast<float>(track_initial_position_std_ * track_initial_position_std_);
        const float velocity_variance =
            static_cast<float>(track_initial_velocity_std_ * track_initial_velocity_std_);
        for (int i = 0; i < 3; ++i) {
            tracker_.errorCovPost.at<float>(i, i) = position_variance;
            tracker_.errorCovPost.at<float>(i + 3, i + 3) = velocity_variance;
        }
        setMeasurementNoise(measurement);
        track_state_ = TrackState::TENTATIVE;
        tracker_initialized_ = true;
        track_hits_.clear();
        recordTrackHit(true);
        last_filter_stamp_ = stamp;
        last_measurement_stamp_ = stamp;
    }

    static yolo_trt_detector::TrackReacquisitionPolicy::Position policyPosition(
        const tf2::Vector3& position) {
        return {{position.x(), position.y(), position.z()}};
    }

    yolo_trt_detector::TrackReacquisitionPolicy::Position trackerPosition(
        int offset) const {
        return {{tracker_.statePost.at<float>(offset + 0),
                 tracker_.statePost.at<float>(offset + 1),
                 tracker_.statePost.at<float>(offset + 2)}};
    }

    void resetTracker(const ros::Time& lost_stamp = ros::Time()) {
        const bool preserve_identity = tracker_initialized_ &&
            (track_state_ == TrackState::CONFIRMED ||
             track_state_ == TrackState::COASTING);
        const ros::Time effective_stamp = lost_stamp.isZero() ? last_filter_stamp_ : lost_stamp;
        if (preserve_identity && !effective_stamp.isZero()) {
            reacquisition_policy_.rememberConfirmed(
                trackerPosition(0), trackerPosition(3), effective_stamp.toSec());
            reacquisition_policy_.markLost(effective_stamp.toSec());
        } else {
            reacquisition_policy_.clearTentative();
        }
        tracker_initialized_ = false;
        track_state_ = TrackState::LOST;
        track_hits_.clear();
        last_filter_stamp_ = ros::Time();
        last_measurement_stamp_ = ros::Time();
        track_visual_box_valid_ = false;
        track_visual_velocity_px_ = cv::Point2f(0.0f, 0.0f);
        last_track_visual_stamp_ = ros::Time();
        last_confirmed_yolo_box_valid_ = false;
        has_last_planner_goal_ = false;
        last_planner_goal_stamp_ = ros::Time();
    }

    void updateTrackVisualization(
        const TargetMeasurement& measurement, const ros::Time& stamp,
        const std::string& frame) {
        const cv::Point2f center(
            measurement.detection.box.x + measurement.detection.box.width * 0.5f,
            measurement.detection.box.y + measurement.detection.box.height * 0.5f);
        if (track_visual_box_valid_ && !last_track_visual_stamp_.isZero()) {
            const double dt = (stamp - last_track_visual_stamp_).toSec();
            if (std::isfinite(dt) && dt > 1e-3 && dt < 1.0) {
                const cv::Point2f measured_velocity =
                    (center - track_visual_center_) * static_cast<float>(1.0 / dt);
                track_visual_velocity_px_ =
                    track_visual_velocity_px_ * 0.5f + measured_velocity * 0.5f;
            }
        } else {
            track_visual_velocity_px_ = cv::Point2f(0.0f, 0.0f);
        }
        track_visual_box_ = measurement.detection.box;
        track_visual_center_ = center;
        track_visual_score_ = measurement.detection.score;
        track_visual_depth_ = measurement.depth;
        track_visual_depth_mad_ = measurement.depth_mad;
        track_visual_depth_samples_ = measurement.depth_samples;
        track_visual_position_ = measurement.position;
        track_visual_frame_ = frame;
        track_visual_box_valid_ = true;
        last_track_visual_stamp_ = stamp;
    }

    void updateTrackVisualPositionFromFilter() {
        if (!tracker_initialized_) {
            return;
        }
        track_visual_position_.setValue(
            tracker_.statePost.at<float>(0), tracker_.statePost.at<float>(1),
            tracker_.statePost.at<float>(2));
    }

    cv::Rect predictedTrackBox(const ros::Time& stamp) const {
        if (!track_visual_box_valid_) {
            return cv::Rect();
        }
        double prediction_time = 0.0;
        if (!last_track_visual_stamp_.isZero()) {
            prediction_time = std::max(
                0.0, std::min(track_coast_publish_time_,
                              (stamp - last_track_visual_stamp_).toSec()));
        }
        const cv::Point2f center = track_visual_center_ +
            track_visual_velocity_px_ * static_cast<float>(prediction_time);
        return cv::Rect(
            static_cast<int>(std::round(center.x - track_visual_box_.width * 0.5f)),
            static_cast<int>(std::round(center.y - track_visual_box_.height * 0.5f)),
            track_visual_box_.width, track_visual_box_.height);
    }

    void recordTrackHit(bool hit) {
        track_hits_.push_back(hit);
        while (static_cast<int>(track_hits_.size()) > std::max(1, track_confirm_window_)) {
            track_hits_.pop_front();
        }
    }

    int recentTrackHits() const {
        return static_cast<int>(std::count(track_hits_.begin(), track_hits_.end(), true));
    }

    void setMeasurementNoise(const TargetMeasurement& measurement) {
        tracker_.measurementNoiseCov = cv::Mat::zeros(3, 3, CV_32F);
        const double base_variance = track_measurement_std_ * track_measurement_std_;
        const double depth_variance = measurement.depth_mad * measurement.depth_mad;
        tracker_.measurementNoiseCov.at<float>(0, 0) =
            static_cast<float>(base_variance + 0.25 * depth_variance);
        tracker_.measurementNoiseCov.at<float>(1, 1) =
            static_cast<float>(base_variance + 0.25 * depth_variance);
        tracker_.measurementNoiseCov.at<float>(2, 2) =
            static_cast<float>(base_variance + depth_variance);
    }

    void predictTracker(const ros::Time& stamp) {
        double dt = (stamp - last_filter_stamp_).toSec();
        if (!std::isfinite(dt) || dt < 0.0 || dt > 1.0) {
            resetTracker(stamp);
            return;
        }
        dt = std::max(1e-3, dt);
        tracker_.transitionMatrix = cv::Mat::eye(6, 6, CV_32F);
        for (int axis = 0; axis < 3; ++axis) {
            tracker_.transitionMatrix.at<float>(axis, axis + 3) = static_cast<float>(dt);
        }

        tracker_.processNoiseCov = cv::Mat::zeros(6, 6, CV_32F);
        const double q = track_process_accel_std_ * track_process_accel_std_;
        const float q_pp = static_cast<float>(0.25 * dt * dt * dt * dt * q);
        const float q_pv = static_cast<float>(0.5 * dt * dt * dt * q);
        const float q_vv = static_cast<float>(dt * dt * q);
        for (int axis = 0; axis < 3; ++axis) {
            tracker_.processNoiseCov.at<float>(axis, axis) = q_pp;
            tracker_.processNoiseCov.at<float>(axis, axis + 3) = q_pv;
            tracker_.processNoiseCov.at<float>(axis + 3, axis) = q_pv;
            tracker_.processNoiseCov.at<float>(axis + 3, axis + 3) = q_vv;
        }
        tracker_.predict();
        last_filter_stamp_ = stamp;
    }

    int associateMeasurement(const std::vector<TargetMeasurement>& measurements) {
        int best_index = -1;
        double best_distance = track_gate_mahalanobis_;
        for (size_t i = 0; i < measurements.size(); ++i) {
            const TargetMeasurement& measurement = measurements[i];
            if (measurement.detection.score < track_update_confidence_) {
                ++rejection_counters_.confidence;
                continue;
            }
            const cv::Mat innovation = (cv::Mat_<float>(3, 1) <<
                measurement.position.x() - tracker_.statePre.at<float>(0),
                measurement.position.y() - tracker_.statePre.at<float>(1),
                measurement.position.z() - tracker_.statePre.at<float>(2));
            if (track_state_ == TrackState::TENTATIVE &&
                cv::norm(innovation) > track_init_gate_distance_) {
                ++rejection_counters_.association;
                continue;
            }
            if (cv::norm(innovation) > track_max_measurement_jump_) {
                ++rejection_counters_.jump;
                continue;
            }
            setMeasurementNoise(measurement);
            const cv::Mat innovation_covariance =
                tracker_.measurementMatrix * tracker_.errorCovPre *
                tracker_.measurementMatrix.t() + tracker_.measurementNoiseCov;
            cv::Mat inverse_covariance;
            if (!cv::invert(innovation_covariance, inverse_covariance, cv::DECOMP_SVD)) {
                continue;
            }
            const cv::Mat distance_matrix =
                innovation.t() * inverse_covariance * innovation;
            const double distance = distance_matrix.at<float>(0, 0);
            if (std::isfinite(distance) && distance < best_distance) {
                best_distance = distance;
                best_index = static_cast<int>(i);
            }
        }
        if (best_index < 0 && !measurements.empty()) {
            ++rejection_counters_.association;
        }
        return best_index;
    }

    void updateTracker(
        const std_msgs::Header& header, const std::string& output_frame,
        const std::vector<TargetMeasurement>& measurements) {
        if (!tracker_initialized_) {
            const bool retaining_identity =
                reacquisition_policy_.hasRetainedIdentity(header.stamp.toSec());
            const TargetMeasurement* initial = nullptr;
            double best_rank = retaining_identity ? INFINITY : -INFINITY;
            for (const auto& measurement : measurements) {
                const double rank = retaining_identity
                    ? reacquisition_policy_.reacquisitionDistance(
                          policyPosition(measurement.position), header.stamp.toSec())
                    : measurement.detection.score;
                if ((retaining_identity && rank < best_rank) ||
                    (!retaining_identity && rank > best_rank)) {
                    best_rank = rank;
                    initial = &measurement;
                }
            }
            if (initial == nullptr) {
                publishTrackerStatus();
                return;
            }
            const auto decision = reacquisition_policy_.observeCandidate(
                policyPosition(initial->position), initial->detection.score,
                header.stamp.toSec());
            if (decision == yolo_trt_detector::ReacquisitionDecision::kRejected) {
                publishTrackerStatus();
                return;
            }
            initializeTracker(*initial, header.stamp);
            if (decision == yolo_trt_detector::ReacquisitionDecision::kReacquired) {
                const auto& velocity = reacquisition_policy_.retainedVelocity();
                for (int axis = 0; axis < 3; ++axis) {
                    tracker_.statePost.at<float>(axis + 3) =
                        static_cast<float>(velocity[axis]);
                }
                track_state_ = TrackState::CONFIRMED;
            } else if (decision ==
                       yolo_trt_detector::ReacquisitionDecision::kConfirmedNew) {
                track_state_ = TrackState::CONFIRMED;
            }
            updateTrackVisualization(*initial, header.stamp, output_frame);
            publishPoint(raw_target_point_pub_, header, output_frame, initial->position);
            if (track_state_ == TrackState::CONFIRMED) {
                publishTrackedState(header, output_frame);
            }
            publishTrackerStatus();
            return;
        }

        predictTracker(header.stamp);
        if (!tracker_initialized_) {
            publishTrackerStatus();
            return;
        }

        const int measurement_index = associateMeasurement(measurements);
        if (measurement_index >= 0) {
            const TargetMeasurement& measurement = measurements[measurement_index];
            setMeasurementNoise(measurement);
            const cv::Mat observed = (cv::Mat_<float>(3, 1) <<
                measurement.position.x(), measurement.position.y(), measurement.position.z());
            tracker_.correct(observed);
            last_measurement_stamp_ = header.stamp;
            recordTrackHit(true);
            updateTrackVisualization(measurement, header.stamp, output_frame);
            updateTrackVisualPositionFromFilter();
            publishPoint(raw_target_point_pub_, header, output_frame, measurement.position);
            if (track_state_ == TrackState::TENTATIVE) {
                const auto decision = reacquisition_policy_.observeCandidate(
                    policyPosition(measurement.position), measurement.detection.score,
                    header.stamp.toSec());
                if (decision == yolo_trt_detector::ReacquisitionDecision::kConfirmedNew) {
                    track_state_ = TrackState::CONFIRMED;
                }
            } else if (track_state_ == TrackState::COASTING) {
                track_state_ = TrackState::CONFIRMED;
            }
            if (track_state_ == TrackState::CONFIRMED) {
                reacquisition_policy_.rememberConfirmed(
                    trackerPosition(0), trackerPosition(3), header.stamp.toSec());
            }
        } else {
            tracker_.statePost = tracker_.statePre.clone();
            tracker_.errorCovPost = tracker_.errorCovPre.clone();
            updateTrackVisualPositionFromFilter();
            recordTrackHit(false);
            if (track_state_ == TrackState::TENTATIVE) {
                reacquisition_policy_.observeMiss();
            }
            const double missing_time = (header.stamp - last_measurement_stamp_).toSec();
            if (track_state_ == TrackState::CONFIRMED && missing_time > 0.0) {
                track_state_ = TrackState::COASTING;
            }
            if ((track_state_ == TrackState::TENTATIVE &&
                 static_cast<int>(track_hits_.size()) >= std::max(1, track_confirm_window_) &&
                 recentTrackHits() < std::max(1, track_confirm_hits_)) ||
                missing_time > track_lost_time_) {
                resetTracker(header.stamp);
                publishTrackerStatus();
                return;
            }
        }

        const double measurement_age = (header.stamp - last_measurement_stamp_).toSec();
        if (track_state_ == TrackState::CONFIRMED ||
            (track_state_ == TrackState::COASTING &&
             measurement_age <= track_coast_publish_time_)) {
            publishTrackedState(header, output_frame);
        }
        publishTrackerStatus();
    }

    void publishPoint(
        ros::Publisher& publisher, const std_msgs::Header& header,
        const std::string& frame, const tf2::Vector3& position) const {
        geometry_msgs::PointStamped point;
        point.header = header;
        point.header.frame_id = frame;
        point.point.x = position.x();
        point.point.y = position.y();
        point.point.z = position.z();
        publisher.publish(point);
    }

    void publishTrackedState(const std_msgs::Header& header, const std::string& frame) {
        const ros::Time publish_stamp =
            header.stamp.isZero() ? ros::Time::now() : header.stamp;
        if (!target_publish_limiter_.shouldPublish(publish_stamp.toSec())) {
            return;
        }
        const tf2::Vector3 position(
            tracker_.statePost.at<float>(0), tracker_.statePost.at<float>(1),
            tracker_.statePost.at<float>(2));
        publishPoint(target_point_pub_, header, frame, position);
        publishPlannerGoal(header, frame, position);

        nav_msgs::Odometry tracked;
        tracked.header = header;
        tracked.header.frame_id = frame;
        tracked.child_frame_id = "tracked_target";
        tracked.pose.pose.position.x = position.x();
        tracked.pose.pose.position.y = position.y();
        tracked.pose.pose.position.z = position.z();
        tracked.pose.pose.orientation.w = 1.0;
        tracked.twist.twist.linear.x = tracker_.statePost.at<float>(3);
        tracked.twist.twist.linear.y = tracker_.statePost.at<float>(4);
        tracked.twist.twist.linear.z = tracker_.statePost.at<float>(5);
        tracked.pose.covariance[0] = tracker_.errorCovPost.at<float>(0, 0);
        tracked.pose.covariance[7] = tracker_.errorCovPost.at<float>(1, 1);
        tracked.pose.covariance[14] = tracker_.errorCovPost.at<float>(2, 2);
        tracked.twist.covariance[0] = tracker_.errorCovPost.at<float>(3, 3);
        tracked.twist.covariance[7] = tracker_.errorCovPost.at<float>(4, 4);
        tracked.twist.covariance[14] = tracker_.errorCovPost.at<float>(5, 5);
        tracked_odom_pub_.publish(tracked);
    }

    void logRejectionSummary() const {
        ROS_INFO_STREAM_THROTTLE(
            5.0,
            "YOLO rejection counters: confidence=" << rejection_counters_.confidence
            << " depth_samples=" << rejection_counters_.insufficient_depth
            << " depth_mad=" << rejection_counters_.excessive_mad
            << " odom_time=" << rejection_counters_.odom_time_mismatch
            << " association=" << rejection_counters_.association
            << " jump=" << rejection_counters_.jump);
    }

    void publishPlannerGoal(
        const std_msgs::Header& header, const std::string& frame,
        const tf2::Vector3& position) {
        if (!publish_planner_goal_) return;
        if (frame != global_frame_id_) {
            ROS_WARN_THROTTLE(
                1.0,
                "Skipping planner goal: YOLO target frame '%s' is not global frame '%s'.",
                frame.c_str(), global_frame_id_.c_str());
            return;
        }

        const ros::Time stamp = header.stamp.isZero() ? ros::Time::now() : header.stamp;
        if (has_last_planner_goal_) {
            const double elapsed = (stamp - last_planner_goal_stamp_).toSec();
            if (elapsed >= 0.0 && elapsed < 1.0 / planner_goal_publish_rate_) return;
        }

        tf2::Vector3 goal_position = position;
        tf2::Vector3 vehicle_position;
        if (planner_goal_standoff_distance_ > 0.0) {
            nav_msgs::Odometry current_odom;
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                if (!has_odom_) {
                    ROS_WARN_THROTTLE(1.0, "Skipping planner goal: no current vehicle odometry.");
                    return;
                }
                current_odom = latest_odom_;
            }
            vehicle_position.setValue(
                current_odom.pose.pose.position.x,
                current_odom.pose.pose.position.y,
                current_odom.pose.pose.position.z);
            if (!yolo_trt_detector::calculateStandoffGoal(
                    vehicle_position, position, planner_goal_standoff_distance_,
                    goal_position)) {
                ROS_WARN_THROTTLE(1.0, "Skipping planner goal: invalid target or odometry position.");
                return;
            }
        }

        geometry_msgs::PoseStamped goal;
        goal.header          = header;
        goal.header.stamp    = stamp;
        goal.header.frame_id = frame;
        goal.pose.position.x = goal_position.x();
        goal.pose.position.y = goal_position.y();
        goal.pose.position.z = goal_position.z();
        goal.pose.orientation.w = 1.0;
        planner_goal_pub_.publish(goal);

        last_planner_goal_stamp_ = stamp;
        has_last_planner_goal_   = true;
        ROS_INFO_THROTTLE(
            1.0,
            "Published YOLO standoff goal to %s: goal=(%.2f %.2f %.2f), "
            "target=(%.2f %.2f %.2f), distance=%.2f m, frame=%s",
            planner_goal_topic_.c_str(), goal_position.x(), goal_position.y(),
            goal_position.z(), position.x(), position.y(), position.z(),
            planner_goal_standoff_distance_, frame.c_str());
    }

    void publishTrackerStatus() const {
        std_msgs::String status;
        status.data = trackStateName();
        tracker_status_pub_.publish(status);
    }

    struct RuntimeDeleter {
        void operator()(nvinfer1::IRuntime* ptr) const {
            if (ptr) {
#if NV_TENSORRT_MAJOR >= 10
                delete ptr;
#else
                ptr->destroy();
#endif
            }
        }
    };
    struct EngineDeleter {
        void operator()(nvinfer1::ICudaEngine* ptr) const {
            if (ptr) {
#if NV_TENSORRT_MAJOR >= 10
                delete ptr;
#else
                ptr->destroy();
#endif
            }
        }
    };
    struct ContextDeleter {
        void operator()(nvinfer1::IExecutionContext* ptr) const {
            if (ptr) {
#if NV_TENSORRT_MAJOR >= 10
                delete ptr;
#else
                ptr->destroy();
#endif
            }
        }
    };

    ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    image_transport::ImageTransport it_;
    typedef message_filters::sync_policies::ApproximateTime<
        sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
    typedef message_filters::Synchronizer<SyncPolicy> Sync;
    message_filters::Subscriber<sensor_msgs::Image> image_sub_;
    message_filters::Subscriber<sensor_msgs::Image> depth_sub_;
    std::unique_ptr<Sync> sync_;
    ros::Subscriber camera_info_sub_;
    ros::Subscriber odom_sub_;
    ros::Subscriber extrinsic_sub_;
    ros::Subscriber lidar_cloud_sub_;
    image_transport::Publisher annotated_pub_;
    image_transport::Publisher target_mask_pub_;
    ros::Publisher marker_pub_;
    ros::Publisher target_point_pub_;
    ros::Publisher raw_target_point_pub_;
    ros::Publisher tracked_odom_pub_;
    ros::Publisher tracker_status_pub_;
    ros::Publisher planner_goal_pub_;

    std::string engine_file_;
    std::string image_topic_;
    std::string depth_topic_;
    std::string camera_info_topic_;
    std::string odom_topic_;
    std::string extrinsic_topic_;
    std::string target_topic_;
    std::string target_mask_topic_;
    std::string raw_target_topic_;
    std::string tracked_odom_topic_;
    std::string tracker_status_topic_;
    std::string planner_goal_topic_;
    std::string lidar_cloud_topic_ = "/cloud_registered";
    std::string frame_id_;
    std::string global_frame_id_;
    float conf_threshold_    = 0.25f;
    float nms_threshold_     = 0.70f;
    float depth_scale_       = 0.001f;
    double max_pair_dt_      = 0.05;
    int depth_roi_radius_    = 3;
    bool rotate_input_180_   = false;
    bool use_vins_extrinsic_ = true;
    bool publish_target_point_ = true;
    bool use_camera_rotation_matrix_ = false;
    bool output_world_frame_ = false;
    bool enable_target_tracking_ = true;
    bool enable_lidar_target_tracking_ = false;
    bool publish_planner_goal_ = false;
    double camera_tx_        = 0.0;
    double camera_ty_        = 0.0;
    double camera_tz_        = 0.0;
    double camera_roll_      = 0.0;
    double camera_pitch_     = 0.0;
    double camera_yaw_       = 0.0;
    std::array<double, 9> camera_rotation_values_{};
    tf2::Matrix3x3 camera_rotation_body_depth_;
    tf2::Vector3 camera_translation_body_depth_;
    int detect_hz_           = 30;
    double depth_roi_scale_ = 0.35;
    int depth_min_samples_ = 12;
    double depth_cluster_abs_ = 0.25;
    double depth_cluster_rel_ = 0.08;
    double depth_mad_max_ = 0.25;
    double target_min_depth_ = 0.25;
    double target_max_depth_ = 15.0;
    double target_mask_expand_scale_ = 1.20;
    double odom_buffer_duration_ = 2.0;
    double odom_max_time_offset_ = 0.10;
    double track_init_confidence_ = 0.55;
    double track_update_confidence_ = 0.25;
    int track_confirm_hits_ = 3;
    int track_confirm_window_ = 5;
    double track_gate_mahalanobis_ = 11.34;
    double track_init_gate_distance_ = 0.8;
    double track_coast_publish_time_ = 0.30;
    double track_lost_time_ = 0.50;
    double track_identity_memory_time_ = 2.0;
    double track_reacquire_distance_ = 1.5;
    double track_max_measurement_jump_ = 1.0;
    double track_process_accel_std_ = 1.5;
    double track_measurement_std_ = 0.08;
    double track_initial_position_std_ = 0.15;
    double track_initial_velocity_std_ = 1.0;
    double planner_goal_publish_rate_ = 10.0;
    double tracked_target_publish_rate_ = 10.0;
    double planner_goal_standoff_distance_ = 0.0;
    double lidar_target_physical_size_ = 0.55;
    double lidar_visual_match_margin_ = 0.15;
    double lidar_candidate_max_age_ = 0.25;
    double lidar_visual_filter_max_age_ = 0.25;
    double lidar_visual_filter_margin_ = 0.20;
    int target_class_id_ = 0;
    yolo_trt_detector::VisualLidarMatchConfig visual_lidar_match_config_;
    yolo_trt_detector::FrustumLidarConfig frustum_lidar_config_;
    ros::Time last_infer_time_;
    std::mutex depth_mutex_;
    std::mutex lidar_points_mutex_;
    mutable std::mutex odom_mutex_;
    std::mutex extrinsic_mutex_;
    cv::Mat latest_depth_;
    std::string latest_depth_encoding_;
    ros::Time latest_depth_stamp_;
    sensor_msgs::CameraInfo latest_camera_info_;
    bool has_camera_info_ = false;
    std::vector<yolo_trt_detector::LidarPoint> latest_lidar_points_;
    ros::Time latest_lidar_points_stamp_;
    nav_msgs::Odometry latest_odom_;
    bool has_odom_ = false;
    std::deque<nav_msgs::Odometry> odom_buffer_;
    nav_msgs::Odometry latest_extrinsic_;
    bool has_extrinsic_ = false;
    cv::KalmanFilter tracker_;
    TrackState track_state_ = TrackState::LOST;
    bool tracker_initialized_ = false;
    bool has_last_planner_goal_ = false;
    ros::Time last_planner_goal_stamp_;
    std::deque<bool> track_hits_;
    ros::Time last_filter_stamp_;
    ros::Time last_measurement_stamp_;
    cv::Rect track_visual_box_;
    cv::Point2f track_visual_center_{0.0f, 0.0f};
    cv::Point2f track_visual_velocity_px_{0.0f, 0.0f};
    bool track_visual_box_valid_ = false;
    float track_visual_score_ = 0.0f;
    float track_visual_depth_ = 0.0f;
    float track_visual_depth_mad_ = 0.0f;
    int track_visual_depth_samples_ = 0;
    tf2::Vector3 track_visual_position_{0.0, 0.0, 0.0};
    std::string track_visual_frame_;
    ros::Time last_track_visual_stamp_;
    ros::Time last_yolo_confirmation_stamp_;
    cv::Rect last_confirmed_yolo_box_;
    bool last_confirmed_yolo_box_valid_ = false;
    yolo_trt_detector::LidarTrackerConfig lidar_tracker_config_;
    yolo_trt_detector::LidarTargetTracker lidar_tracker_;
    yolo_trt_detector::LidarTrackEstimate last_lidar_estimate_;
    yolo_trt_detector::TrackReacquisitionPolicy reacquisition_policy_;
    yolo_trt_detector::PublicationLimiter target_publish_limiter_;
    struct RejectionCounters {
        uint64_t confidence = 0;
        uint64_t insufficient_depth = 0;
        uint64_t excessive_mad = 0;
        uint64_t odom_time_mismatch = 0;
        uint64_t association = 0;
        uint64_t jump = 0;
    } rejection_counters_;

    TrtLogger logger_;
    std::vector<char> engine_data_;
    std::unique_ptr<nvinfer1::IRuntime, RuntimeDeleter> runtime_;
    std::unique_ptr<nvinfer1::ICudaEngine, EngineDeleter> engine_;
    std::unique_ptr<nvinfer1::IExecutionContext, ContextDeleter> context_;
    cudaStream_t stream_ = nullptr;
    std::vector<void*> device_buffers_;
    std::vector<float> host_input_;
    std::vector<float> host_output_;

    int input_index_     = -1;
    int output_index_    = -1;
    int input_h_         = 480;
    int input_w_         = 640;
    int output_channels_ = 5;
    int output_count_    = 6300;
    size_t input_size_   = 0;
    size_t output_size_  = 0;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "yolo_trt_node");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

    try {
        YoloTrtNode node(nh, pnh);
        ros::spin();
    } catch (const std::exception& e) {
        ROS_FATAL_STREAM("yolo_trt_node failed: " << e.what());
        return 1;
    }
    return 0;
}
