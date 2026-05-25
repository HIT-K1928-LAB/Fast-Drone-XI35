#include <algorithm>
#include <fstream>
#include <memory>
#include <numeric>
#include <string>
#include <vector>

#include <NvInfer.h>
#include <cuda_runtime_api.h>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <visualization_msgs/MarkerArray.h>

namespace
{

class TrtLogger : public nvinfer1::ILogger
{
public:
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
    pnh_.param<std::string>("engine_file", engine_file_, "code/best.engine");
    pnh_.param<std::string>("image_topic", image_topic_, "/camera/image_raw");
    pnh_.param<std::string>("frame_id", frame_id_, "camera");
    pnh_.param<float>("conf_threshold", conf_threshold_, 0.25f);
    pnh_.param<float>("nms_threshold", nms_threshold_, 0.45f);
    pnh_.param<int>("detect_hz", detect_hz_, 30);

    loadEngine();

    image_sub_ = it_.subscribe(image_topic_, 1, &YoloTrtNode::imageCallback, this);
    annotated_pub_ = it_.advertise("yolo_trt/annotated_image", 1);
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("yolo_trt/markers", 1);

    ROS_INFO_STREAM("yolo_trt_node ready, engine=" << engine_file_ << ", image_topic=" << image_topic_);
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
  image_transport::Publisher annotated_pub_;
  ros::Publisher marker_pub_;

  std::string engine_file_;
  std::string image_topic_;
  std::string frame_id_;
  float conf_threshold_ = 0.25f;
  float nms_threshold_ = 0.45f;
  int detect_hz_ = 30;
  ros::Time last_infer_time_;

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
