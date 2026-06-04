#include <Eigen/Core>
#include <Eigen/Geometry>
#include <chrono>
#include <geometry_msgs/PoseStamped.h>
#include <limits>
#include <memory>
#include <nav_msgs/Odometry.h>
#include <opencv2/imgproc.hpp>
#include <ros/ros.h>
#ifdef YOPO_HAVE_OPENCV_PHOTO
#include <opencv2/photo.hpp>
#endif
#include "yopo_planner/YopoLog.h"
#include "yopo_planner/yopo_engine.h"
#include "yopo_planner/yopo_planner.h"
#include <quadrotor_msgs/PositionCommand.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Header.h>

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
        pnh_.param("ctrl_dt", params.ctrl_dt, params.ctrl_dt);
        pnh_.param("arrive_distance", params.arrive_distance, params.arrive_distance);
        pnh_.param("min_depth", params.min_depth, params.min_depth);
        pnh_.param("max_depth", params.max_depth, params.max_depth);
        pnh_.param("verbose", verbose_, false);
        pnh_.param("visualize", visualize_, true);
        pnh_.param("wait_for_traj_start_trigger", wait_for_traj_start_trigger_, false);

        double goal_x = 50.0;
        double goal_y = 0.0;
        double goal_z = 2.0;
        pnh_.param("goal_x", goal_x, goal_x);
        pnh_.param("goal_y", goal_y, goal_y);
        pnh_.param("goal_z", goal_z, goal_z);

        planner_.reset(new YopoPlanner(params));
        planner_->setGoal(Eigen::Vector3d(goal_x, goal_y, goal_z));

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

        std::string odom_topic       = "/sim/odom";
        std::string depth_topic      = "/depth_image";
        std::string ctrl_topic       = "/so3_control/pos_cmd";
        std::string traj_start_topic = "/traj_start_trigger";
        std::string extrinsic_topic  = "/vins_fusion/extrinsic";
        pnh_.param<std::string>("odom_topic", odom_topic, odom_topic);
        pnh_.param<std::string>("depth_topic", depth_topic, depth_topic);
        pnh_.param<std::string>("ctrl_topic", ctrl_topic, ctrl_topic);
        pnh_.param<std::string>("traj_start_topic", traj_start_topic, traj_start_topic);
        pnh_.param<std::string>("extrinsic_topic", extrinsic_topic, extrinsic_topic);

        control_enabled_ = !wait_for_traj_start_trigger_;

        ctrl_pub_      = nh_.advertise<quadrotor_msgs::PositionCommand>(ctrl_topic, 1);
        best_traj_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/yopo_net/best_traj_visual", 1);
        all_trajs_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/yopo_net/trajs_visual", 1);
        lattice_traj_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/yopo_net/lattice_trajs_visual", 1);
        log_pub_ = nh_.advertise<yopo_planner::YopoLog>("/yopo_log/log", 10);

        odom_sub_ = nh_.subscribe(
            odom_topic, 1, &YopoPlannerNode::odomCallback, this,
            ros::TransportHints().tcpNoDelay());
        depth_sub_ = nh_.subscribe(
            depth_topic, 1, &YopoPlannerNode::depthCallback, this,
            ros::TransportHints().tcpNoDelay());
        goal_sub_ =
            nh_.subscribe("/move_base_simple/goal", 1, &YopoPlannerNode::goalCallback, this);
        traj_start_sub_ =
            nh_.subscribe(traj_start_topic, 1, &YopoPlannerNode::trajStartCallback, this);
        extrinsic_sub_ = nh_.subscribe(
            extrinsic_topic, 1, &YopoPlannerNode::extrinsicCallback, this,
            ros::TransportHints().tcpNoDelay());
        ctrl_timer_ = nh_.createTimer(
            ros::Duration(planner_->params().ctrl_dt), &YopoPlannerNode::controlTimer, this);

        warmUp();
        if (wait_for_traj_start_trigger_) {
            ROS_INFO(
                "YOPO waiting for %s before publishing control commands.",
                traj_start_topic.c_str());
        }
        if (planner_->requiresCameraExtrinsic()) {
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
        planner_->updateOdometry(*msg);
        if (planner_->arrived()) {
            ROS_INFO_THROTTLE(2.0, "YOPO planner arrived near goal.");
        }
    }

    void extrinsicCallback(const nav_msgs::Odometry::ConstPtr& msg) {
        const Eigen::Quaterniond q(
            msg->pose.pose.orientation.w, msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y, msg->pose.pose.orientation.z);
        planner_->setCameraToBodyExtrinsic(q.normalized().toRotationMatrix());
        if (!extrinsic_ready_logged_) {
            ROS_INFO("YOPO received camera-to-body extrinsic.");
            extrinsic_ready_logged_ = true;
        }
    }

    void goalCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        planner_->setGoal(
            Eigen::Vector3d(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z));
        ROS_INFO(
            "YOPO new goal: %.2f %.2f %.2f", msg->pose.position.x, msg->pose.position.y,
            msg->pose.position.z);
    }

    void trajStartCallback(const geometry_msgs::PoseStamped::ConstPtr& msg) {
        (void)msg;
        if (!control_enabled_) {
            ROS_INFO("YOPO received traj start trigger, start publishing control commands.");
        }
        control_enabled_ = true;
    }

    void depthCallback(const sensor_msgs::Image::ConstPtr& msg) {
        if (!planner_->odomInitialized()) return;
        if (planner_->requiresCameraExtrinsic() && !planner_->cameraExtrinsicReady()) {
            ROS_WARN_THROTTLE(1.0, "YOPO waiting for camera-to-body extrinsic.");
            return;
        }

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

    bool prepareDepth(
        const sensor_msgs::Image& msg, std::array<float, 1 * 1 * 96 * 160>* depth_input) const {
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

        cv::Mat resized;
        const auto& params = planner_->params();
        if (depth_meters.rows != params.image_height || depth_meters.cols != params.image_width) {
            cv::resize(
                depth_meters, resized, cv::Size(params.image_width, params.image_height), 0.0, 0.0,
                cv::INTER_NEAREST);
        } else {
            resized = depth_meters;
        }

        cv::Mat normalized(params.image_height, params.image_width, CV_32FC1);
        cv::Mat invalid_mask(params.image_height, params.image_width, CV_8UC1);
        const float min_norm = static_cast<float>(params.min_depth / params.max_depth);
        for (int r = 0; r < params.image_height; ++r) {
            const float* src = resized.ptr<float>(r);
            float* dst       = normalized.ptr<float>(r);
            uint8_t* mask    = invalid_mask.ptr<uint8_t>(r);
            for (int c = 0; c < params.image_width; ++c) {
                float v            = src[c];
                const bool invalid = std::isnan(v) || v < params.min_depth;
                if (std::isnan(v)) v = 0.0f;
                v = std::min(v, static_cast<float>(params.max_depth)) /
                    static_cast<float>(params.max_depth);
                dst[c]  = v;
                mask[c] = (invalid || v < min_norm) ? 255 : 0;
            }
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

    void controlTimer(const ros::TimerEvent&) {
        if (!control_enabled_) return;
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
    ros::Subscriber odom_sub_;
    ros::Subscriber depth_sub_;
    ros::Subscriber goal_sub_;
    ros::Subscriber traj_start_sub_;
    ros::Subscriber extrinsic_sub_;
    ros::Timer ctrl_timer_;

    bool verbose_                     = false;
    bool visualize_                   = true;
    bool wait_for_traj_start_trigger_ = false;
    bool control_enabled_             = true;
    bool extrinsic_ready_logged_      = false;
    double depth_fps_                 = 30.0;
    int count_                        = 0;
    double time_forward_              = 0.0;
    double time_process_              = 0.0;
    double time_prepare_              = 0.0;
    double time_interpolation_        = 0.0;
    double time_visualize_            = 0.0;
};

}  // namespace yopo_planner

int main(int argc, char** argv) {
    ros::init(argc, argv, "yopo_planner_node");
    yopo_planner::YopoPlannerNode node;
    ros::spin();
    return 0;
}
