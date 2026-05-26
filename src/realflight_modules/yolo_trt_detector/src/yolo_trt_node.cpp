#include <algorithm>
#include <cmath>
#include <fstream>
#include <memory>
#include <mutex>
#include <numeric>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cv_bridge/cv_bridge.h>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
#include <nav_msgs/Odometry.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{
//nvinfer1是NVIDIA TensorRT的核心命名空间，包含了TensorRT的所有主要类和函数。
//ILogger是TensorRT提供的一个接口类，用于处理日志消息。定义“合同模板”
//TrtLogger是一个自定义的日志处理类，继承（public）自nvinfer1::ILogger。“合同”的具体实现
class TrtLogger : public nvinfer1::ILogger
{
public://public\protcted\private
  void log(Severity severity, const char* msg) noexcept override
  {
    if (severity <= Severity::kWARNING)
    {
      ROS_WARN_STREAM("[TensorRT] " << msg);
    }
  }
};

struct Detection
{
  cv::Rect box;
  float score = 0.0f;
  int class_id = 0;
};

struct LetterboxInfo
{
  float scale = 1.0f;
  float pad_x = 0.0f;
  float pad_y = 0.0f;
};

void checkCuda(cudaError_t status, const std::string& where)
{
  if (status != cudaSuccess)
  {
    throw std::runtime_error(where + ": " + cudaGetErrorString(status));
  }
}

std::vector<char> readFile(const std::string& path)
{
  std::ifstream file(path, std::ios::binary);
  if (!file)
  {
    throw std::runtime_error("failed to open engine file: " + path);
  }
  file.seekg(0, std::ios::end);
  const std::streamsize size = file.tellg();
  file.seekg(0, std::ios::beg);
  std::vector<char> data(size);
  if (!file.read(data.data(), size))
  {
    throw std::runtime_error("failed to read engine file: " + path);
  }
  return data;
}

float iou(const cv::Rect& a, const cv::Rect& b)
{
  const int inter_area = (a & b).area();
  const int union_area = a.area() + b.area() - inter_area;
  return union_area > 0 ? static_cast<float>(inter_area) / static_cast<float>(union_area) : 0.0f;
}

std::vector<Detection> nms(std::vector<Detection> detections, float threshold)
{
  std::sort(detections.begin(), detections.end(),
            [](const Detection& lhs, const Detection& rhs) { return lhs.score > rhs.score; });

  std::vector<Detection> kept;
  std::vector<bool> removed(detections.size(), false);
  for (size_t i = 0; i < detections.size(); ++i)
  {
    if (removed[i])
    {
      continue;
    }
    kept.push_back(detections[i]);
    for (size_t j = i + 1; j < detections.size(); ++j)
    {
      if (!removed[j] && iou(detections[i].box, detections[j].box) > threshold)
      {
        removed[j] = true;
      }
    }
  }
  return kept;
}

}  // namespace

class YoloTrtNode
{
public:
  YoloTrtNode(ros::NodeHandle& nh, ros::NodeHandle& pnh)
    : nh_(nh), pnh_(pnh), it_(nh_)
  {
    pnh_.param<std::string>("engine_file", engine_file_, "models/best_yolo11s_p2.engine");
    pnh_.param<std::string>("image_topic", image_topic_, "/camera/infra1/image_rect_raw");
    pnh_.param<std::string>("depth_topic", depth_topic_, "/camera/depth/image_rect_raw");
    pnh_.param<std::string>("camera_info_topic", camera_info_topic_, "/camera/infra1/camera_info");
    pnh_.param<std::string>("odom_topic", odom_topic_, "/kf_fusion/kf_imu_odom");
    pnh_.param<std::string>("extrinsic_topic", extrinsic_topic_, "/vins_fusion/extrinsic");
    pnh_.param<std::string>("target_topic", target_topic_, "yolo_trt/target_point");
    pnh_.param<std::string>("frame_id", frame_id_, "");
    pnh_.param<std::string>("global_frame_id", global_frame_id_, "world");
    pnh_.param<float>("conf_threshold", conf_threshold_, 0.25f);
    pnh_.param<float>("nms_threshold", nms_threshold_, 0.45f);
    pnh_.param<float>("depth_scale", depth_scale_, 0.001f);
    pnh_.param<int>("depth_roi_radius", depth_roi_radius_, 3);
    pnh_.param<bool>("use_vins_extrinsic", use_vins_extrinsic_, true);
    pnh_.param<double>("camera_tx", camera_tx_, 0.0);
    pnh_.param<double>("camera_ty", camera_ty_, 0.0);
    pnh_.param<double>("camera_tz", camera_tz_, 0.0);
    pnh_.param<double>("camera_roll", camera_roll_, 0.0);
    pnh_.param<double>("camera_pitch", camera_pitch_, 0.0);
    pnh_.param<double>("camera_yaw", camera_yaw_, 0.0);
    pnh_.param<int>("detect_hz", detect_hz_, 30);

    loadEngine();

    image_sub_ = it_.subscribe(image_topic_, 1, &YoloTrtNode::imageCallback, this);
    depth_sub_ = nh_.subscribe(depth_topic_, 1, &YoloTrtNode::depthCallback, this);
    camera_info_sub_ = nh_.subscribe(camera_info_topic_, 1, &YoloTrtNode::cameraInfoCallback, this);
    odom_sub_ = nh_.subscribe(odom_topic_, 20, &YoloTrtNode::odomCallback, this);
    if (use_vins_extrinsic_)
    {
      extrinsic_sub_ = nh_.subscribe(extrinsic_topic_, 20, &YoloTrtNode::extrinsicCallback, this);
    }
    annotated_pub_ = it_.advertise("yolo_trt/annotated_image", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("yolo_trt/markers", 1);
    target_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(target_topic_, 1);

    ROS_INFO_STREAM("yolo_trt_node ready, engine=" << engine_file_ << ", image_topic=" << image_topic_
                    << ", depth_topic=" << depth_topic_ << ", camera_info_topic=" << camera_info_topic_
                    << ", odom_topic=" << odom_topic_ << ", extrinsic_topic=" << extrinsic_topic_
                    << ", target_topic=" << target_topic_);
  }

  ~YoloTrtNode()
  {
    for (void* ptr : device_buffers_)
    {
      if (ptr)
      {
        cudaFree(ptr);
      }
    }
    if (stream_)
    {
      cudaStreamDestroy(stream_);
    }
  }

private:
  void loadEngine()
  {
    engine_data_ = readFile(engine_file_);
    runtime_.reset(nvinfer1::createInferRuntime(logger_));
    if (!runtime_)
    {
      throw std::runtime_error("failed to create TensorRT runtime");
    }
    engine_.reset(runtime_->deserializeCudaEngine(engine_data_.data(), engine_data_.size()));
    if (!engine_)
    {
      throw std::runtime_error("failed to deserialize TensorRT engine");
    }
    context_.reset(engine_->createExecutionContext());
    if (!context_)
    {
      throw std::runtime_error("failed to create TensorRT execution context");
    }

    input_index_ = engine_->getBindingIndex("images");
    output_index_ = engine_->getBindingIndex("output0");
    if (input_index_ < 0 || output_index_ < 0)
    {
      throw std::runtime_error("expected TensorRT bindings `images` and `output0`");
    }

    const nvinfer1::Dims input_dims = engine_->getBindingDimensions(input_index_);
    const nvinfer1::Dims output_dims = engine_->getBindingDimensions(output_index_);
    input_h_ = input_dims.d[2];
    input_w_ = input_dims.d[3];
    output_channels_ = output_dims.d[1];
    output_count_ = output_dims.d[2];
    input_size_ = 1;
    for (int i = 0; i < input_dims.nbDims; ++i)
    {
      input_size_ *= input_dims.d[i];
    }
    output_size_ = 1;
    for (int i = 0; i < output_dims.nbDims; ++i)
    {
      output_size_ *= output_dims.d[i];
    }

    host_input_.resize(input_size_);
    host_output_.resize(output_size_);
    device_buffers_.resize(engine_->getNbBindings(), nullptr);
    checkCuda(cudaMalloc(&device_buffers_[input_index_], input_size_ * sizeof(float)), "cudaMalloc input");
    checkCuda(cudaMalloc(&device_buffers_[output_index_], output_size_ * sizeof(float)), "cudaMalloc output");
    checkCuda(cudaStreamCreate(&stream_), "cudaStreamCreate");

    ROS_INFO_STREAM("TensorRT engine loaded: input=1x3x" << input_h_ << "x" << input_w_
                    << ", output=1x" << output_channels_ << "x" << output_count_);
  }

  void imageCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (const cv_bridge::Exception& e)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "cv_bridge exception: " << e.what());
      return;
    }

    const ros::Time now = ros::Time::now();
    if (detect_hz_ > 0 && (now - last_infer_time_).toSec() < 1.0 / detect_hz_)
    {
      return;
    }
    last_infer_time_ = now;

    cv::Mat image = cv_ptr->image.clone();
    LetterboxInfo letterbox = preprocess(image);
    if (!infer())
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "TensorRT inference failed");
      return;
    }

    std::vector<Detection> detections = postprocess(image.size(), letterbox);
    drawDetections(image, detections);
    publishAnnotatedImage(msg->header, image);
    publishMarkers(msg->header, image.size(), detections);
    publishTargetPoint(msg->header, detections);
  }

  void depthCallback(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvShare(msg);
    }
    catch (const cv_bridge::Exception& e)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "depth cv_bridge exception: " << e.what());
      return;
    }

    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_depth_ = cv_ptr->image.clone();
    latest_depth_encoding_ = msg->encoding;
    latest_depth_stamp_ = msg->header.stamp;
  }

  void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(depth_mutex_);
    latest_camera_info_ = *msg;
    has_camera_info_ = true;
  }

  void odomCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(odom_mutex_);
    latest_odom_ = *msg;
    has_odom_ = true;
  }

  void extrinsicCallback(const nav_msgs::OdometryConstPtr& msg)
  {
    std::lock_guard<std::mutex> lock(extrinsic_mutex_);
    latest_extrinsic_ = *msg;
    has_extrinsic_ = true;
  }

  LetterboxInfo preprocess(const cv::Mat& image)
  {
    LetterboxInfo info;
    info.scale = std::min(static_cast<float>(input_w_) / image.cols,
                          static_cast<float>(input_h_) / image.rows);
    const int resized_w = static_cast<int>(std::round(image.cols * info.scale));
    const int resized_h = static_cast<int>(std::round(image.rows * info.scale));
    info.pad_x = (input_w_ - resized_w) / 2.0f;
    info.pad_y = (input_h_ - resized_h) / 2.0f;

    cv::Mat resized;
    cv::resize(image, resized, cv::Size(resized_w, resized_h));
    cv::Mat letterboxed(input_h_, input_w_, CV_8UC3, cv::Scalar(114, 114, 114));
    resized.copyTo(letterboxed(cv::Rect(static_cast<int>(info.pad_x), static_cast<int>(info.pad_y), resized_w, resized_h)));
    cv::cvtColor(letterboxed, letterboxed, cv::COLOR_BGR2RGB);

    const int channel_size = input_h_ * input_w_;
    for (int y = 0; y < input_h_; ++y)
    {
      const cv::Vec3b* row = letterboxed.ptr<cv::Vec3b>(y);
      for (int x = 0; x < input_w_; ++x)
      {
        for (int c = 0; c < 3; ++c)
        {
          host_input_[c * channel_size + y * input_w_ + x] = row[x][c] / 255.0f;
        }
      }
    }
    return info;
  }

  bool infer()
  {
    checkCuda(cudaMemcpyAsync(device_buffers_[input_index_], host_input_.data(),
                              input_size_ * sizeof(float), cudaMemcpyHostToDevice, stream_),
              "cudaMemcpyAsync input");
    const bool ok = context_->enqueueV2(device_buffers_.data(), stream_, nullptr);
    if (!ok)
    {
      return false;
    }
    checkCuda(cudaMemcpyAsync(host_output_.data(), device_buffers_[output_index_],
                              output_size_ * sizeof(float), cudaMemcpyDeviceToHost, stream_),
              "cudaMemcpyAsync output");
    checkCuda(cudaStreamSynchronize(stream_), "cudaStreamSynchronize");
    return true;
  }

  std::vector<Detection> postprocess(const cv::Size& image_size, const LetterboxInfo& letterbox)
  {
    std::vector<Detection> detections;
    if (output_channels_ < 5)
    {
      return detections;
    }

    for (int i = 0; i < output_count_; ++i)
    {
      const float score = host_output_[4 * output_count_ + i];
      if (score < conf_threshold_)
      {
        continue;
      }

      const float cx = host_output_[0 * output_count_ + i];
      const float cy = host_output_[1 * output_count_ + i];
      const float w = host_output_[2 * output_count_ + i];
      const float h = host_output_[3 * output_count_ + i];

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
      if (box_w <= 1 || box_h <= 1)
      {
        continue;
      }

      Detection det;
      det.box = cv::Rect(static_cast<int>(x1), static_cast<int>(y1), box_w, box_h);
      det.score = score;
      detections.push_back(det);
    }
    return nms(std::move(detections), nms_threshold_);
  }

  void drawDetections(cv::Mat& image, const std::vector<Detection>& detections)
  {
    for (const Detection& det : detections)
    {
      cv::rectangle(image, det.box, cv::Scalar(0, 255, 0), 2);
      char label[64];
      std::snprintf(label, sizeof(label), "target %.2f", det.score);
      const int base_line = 0;
      cv::putText(image, label, cv::Point(det.box.x, std::max(20, det.box.y - base_line - 4)),
                  cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
    }
  }

  void publishAnnotatedImage(const std_msgs::Header& header, const cv::Mat& image)
  {
    cv_bridge::CvImage out;
    out.header = header;
    out.encoding = sensor_msgs::image_encodings::BGR8;
    out.image = image;
    annotated_pub_.publish(out.toImageMsg());
  }

  void publishMarkers(const std_msgs::Header& header, const cv::Size& image_size,
                      const std::vector<Detection>& detections)
  {
    visualization_msgs::MarkerArray markers;
    visualization_msgs::Marker clear;
    clear.action = visualization_msgs::Marker::DELETEALL;
    markers.markers.push_back(clear);

    int id = 0;
    for (const Detection& det : detections)
    {
      visualization_msgs::Marker marker;
      marker.header = header;
      marker.header.frame_id = frame_id_.empty() ? header.frame_id : frame_id_;
      marker.ns = "yolo_trt";
      marker.id = id++;
      marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
      marker.action = visualization_msgs::Marker::ADD;
      marker.pose.position.x = (det.box.x + det.box.width * 0.5) / image_size.width - 0.5;
      marker.pose.position.y = (det.box.y + det.box.height * 0.5) / image_size.height - 0.5;
      marker.pose.position.z = 1.0;
      marker.pose.orientation.w = 1.0;
      marker.scale.z = 0.12;
      marker.color.r = 0.0f;
      marker.color.g = 1.0f;
      marker.color.b = 0.0f;
      marker.color.a = 1.0f;
      marker.text = "target " + std::to_string(det.score).substr(0, 4);
      markers.markers.push_back(marker);
    }
    marker_pub_.publish(markers);
  }

  void publishTargetPoint(const std_msgs::Header& header, const std::vector<Detection>& detections)
  {
    if (detections.empty())
    {
      return;
    }

    const Detection& target = detections.front();
    const float u = target.box.x + target.box.width * 0.5f;
    const float v = target.box.y + target.box.height * 0.5f;

    cv::Mat depth;
    std::string depth_encoding;
    sensor_msgs::CameraInfo camera_info;
    {
      std::lock_guard<std::mutex> lock(depth_mutex_);
      if (latest_depth_.empty() || !has_camera_info_)
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "waiting for depth image and camera info");
        return;
      }
      depth = latest_depth_.clone();
      depth_encoding = latest_depth_encoding_;
      camera_info = latest_camera_info_;
    }

    const float z = medianDepth(depth, depth_encoding, u, v);
    if (z <= 0.0f || !std::isfinite(z))
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "invalid depth at target center");
      return;
    }

    const double fx = camera_info.K[0];
    const double fy = camera_info.K[4];
    const double cx = camera_info.K[2];
    const double cy = camera_info.K[5];
    if (fx == 0.0 || fy == 0.0)
    {
      ROS_WARN_STREAM_THROTTLE(1.0, "invalid camera intrinsics");
      return;
    }

    geometry_msgs::PointStamped point;
    point.header = header;
    point.header.frame_id = frame_id_.empty() ? global_frame_id_ : frame_id_;

    nav_msgs::Odometry odom;
    {
      std::lock_guard<std::mutex> lock(odom_mutex_);
      if (!has_odom_)
      {
        ROS_WARN_STREAM_THROTTLE(1.0, "waiting for odometry");
        return;
      }
      odom = latest_odom_;
    }

    const tf2::Vector3 target_c((u - cx) * z / fx, (v - cy) * z / fy, z);
    tf2::Quaternion q_b_c;
    tf2::Vector3 t_b_c;
    if (use_vins_extrinsic_)
    {
      nav_msgs::Odometry extrinsic;
      {
        std::lock_guard<std::mutex> lock(extrinsic_mutex_);
        if (!has_extrinsic_)
        {
          ROS_WARN_STREAM_THROTTLE(1.0, "waiting for VINS camera extrinsic");
          return;
        }
        extrinsic = latest_extrinsic_;
      }
      const geometry_msgs::Quaternion& q_ext = extrinsic.pose.pose.orientation;
      q_b_c = tf2::Quaternion(q_ext.x, q_ext.y, q_ext.z, q_ext.w);
      const geometry_msgs::Point& t_ext = extrinsic.pose.pose.position;
      t_b_c = tf2::Vector3(t_ext.x, t_ext.y, t_ext.z);
    }
    else
    {
      q_b_c.setRPY(camera_roll_, camera_pitch_, camera_yaw_);
      t_b_c = tf2::Vector3(camera_tx_, camera_ty_, camera_tz_);
    }
    q_b_c.normalize();
    const tf2::Matrix3x3 R_b_c(q_b_c);
    const tf2::Vector3 target_b = R_b_c * target_c + t_b_c;

    const geometry_msgs::Quaternion& q_msg = odom.pose.pose.orientation;
    tf2::Quaternion q_w_b(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
    q_w_b.normalize();
    const tf2::Matrix3x3 R_w_b(q_w_b);
    const geometry_msgs::Point& p_msg = odom.pose.pose.position;
    const tf2::Vector3 p_w_b(p_msg.x, p_msg.y, p_msg.z);
    const tf2::Vector3 target_w = R_w_b * target_b + p_w_b;

    point.header.frame_id = frame_id_.empty() ? (odom.header.frame_id.empty() ? global_frame_id_ : odom.header.frame_id) : frame_id_;
    point.point.x = target_w.x();
    point.point.y = target_w.y();
    point.point.z = target_w.z();
    target_point_pub_.publish(point);
  }

  float medianDepth(const cv::Mat& depth, const std::string& encoding, float u, float v) const
  {
    std::vector<float> values;
    const int center_x = static_cast<int>(std::round(u));
    const int center_y = static_cast<int>(std::round(v));
    const int radius = std::max(0, depth_roi_radius_);

    for (int y = std::max(0, center_y - radius); y <= std::min(depth.rows - 1, center_y + radius); ++y)
    {
      for (int x = std::max(0, center_x - radius); x <= std::min(depth.cols - 1, center_x + radius); ++x)
      {
        float value = 0.0f;
        if (encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
            encoding == sensor_msgs::image_encodings::MONO16)
        {
          value = depth.at<uint16_t>(y, x) * depth_scale_;
        }
        else if (encoding == sensor_msgs::image_encodings::TYPE_32FC1)
        {
          value = depth.at<float>(y, x);
        }
        else
        {
          ROS_WARN_STREAM_THROTTLE(1.0, "unsupported depth encoding: " << encoding);
          return 0.0f;
        }

        if (value > 0.0f && std::isfinite(value))
        {
          values.push_back(value);
        }
      }
    }

    if (values.empty())
    {
      return 0.0f;
    }

    const size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    return values[middle];
  }

  struct RuntimeDeleter
  {
    void operator()(nvinfer1::IRuntime* ptr) const { if (ptr) ptr->destroy(); }
  };
  struct EngineDeleter
  {
    void operator()(nvinfer1::ICudaEngine* ptr) const { if (ptr) ptr->destroy(); }
  };
  struct ContextDeleter
  {
    void operator()(nvinfer1::IExecutionContext* ptr) const { if (ptr) ptr->destroy(); }
  };

  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  ros::Subscriber depth_sub_;
  ros::Subscriber camera_info_sub_;
  ros::Subscriber odom_sub_;
  ros::Subscriber extrinsic_sub_;
  image_transport::Publisher annotated_pub_;
  ros::Publisher marker_pub_;
  ros::Publisher target_point_pub_;

  std::string engine_file_;
  std::string image_topic_;
  std::string depth_topic_;
  std::string camera_info_topic_;
  std::string odom_topic_;
  std::string extrinsic_topic_;
  std::string target_topic_;
  std::string frame_id_;
  std::string global_frame_id_;
  float conf_threshold_ = 0.25f;
  float nms_threshold_ = 0.45f;
  float depth_scale_ = 0.001f;
  int depth_roi_radius_ = 3;
  bool use_vins_extrinsic_ = true;
  double camera_tx_ = 0.0;
  double camera_ty_ = 0.0;
  double camera_tz_ = 0.0;
  double camera_roll_ = 0.0;
  double camera_pitch_ = 0.0;
  double camera_yaw_ = 0.0;
  int detect_hz_ = 30;
  ros::Time last_infer_time_;
  std::mutex depth_mutex_;
  std::mutex odom_mutex_;
  std::mutex extrinsic_mutex_;
  cv::Mat latest_depth_;
  std::string latest_depth_encoding_;
  ros::Time latest_depth_stamp_;
  sensor_msgs::CameraInfo latest_camera_info_;
  bool has_camera_info_ = false;
  nav_msgs::Odometry latest_odom_;
  bool has_odom_ = false;
  nav_msgs::Odometry latest_extrinsic_;
  bool has_extrinsic_ = false;

  TrtLogger logger_;
  std::vector<char> engine_data_;
  std::unique_ptr<nvinfer1::IRuntime, RuntimeDeleter> runtime_;
  std::unique_ptr<nvinfer1::ICudaEngine, EngineDeleter> engine_;
  std::unique_ptr<nvinfer1::IExecutionContext, ContextDeleter> context_;
  cudaStream_t stream_ = nullptr;
  std::vector<void*> device_buffers_;
  std::vector<float> host_input_;
  std::vector<float> host_output_;

  int input_index_ = -1;
  int output_index_ = -1;
  int input_h_ = 640;
  int input_w_ = 640;
  int output_channels_ = 5;
  int output_count_ = 8400;
  size_t input_size_ = 0;
  size_t output_size_ = 0;
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "yolo_trt_node");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  try
  {
    YoloTrtNode node(nh, pnh);
    ros::spin();
  }
  catch (const std::exception& e)
  {
    ROS_FATAL_STREAM("yolo_trt_node failed: " << e.what());
    return 1;
  }
  return 0;
}
