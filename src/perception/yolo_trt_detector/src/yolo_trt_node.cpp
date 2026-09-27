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
#include <fstream>
#include <geometry_msgs/PointStamped.h>
#include <image_transport/image_transport.h>
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
#include <sensor_msgs/image_encodings.h>
#include <string>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <yolo_trt_detector/image_orientation.h>

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
        pnh_.param<std::string>("engine_file", engine_file_, "models/best_rgbd.engine");
        pnh_.param<std::string>("image_topic", image_topic_, "/camera/color/image_raw");
        pnh_.param<std::string>("depth_topic", depth_topic_, "/camera/aligned_depth_to_color/image_raw");
        pnh_.param<std::string>(
            "camera_info_topic", camera_info_topic_, "/camera/color/camera_info");
        pnh_.param<std::string>("odom_topic", odom_topic_, "/kf_fusion/kf_imu_odom");
        pnh_.param<std::string>("extrinsic_topic", extrinsic_topic_, "/vins_fusion/extrinsic");
        pnh_.param<std::string>("target_topic", target_topic_, "yolo_trt/target_point");
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
        }
        annotated_pub_    = it_.advertise("yolo_trt/annotated_image", 1);
        marker_pub_       = nh_.advertise<visualization_msgs::MarkerArray>("yolo_trt/markers", 1);
        if (publish_target_point_) {
            target_point_pub_ = nh_.advertise<geometry_msgs::PointStamped>(target_topic_, 1);
        }

        ROS_INFO_STREAM(
            "yolo_trt_node ready, engine="
            << engine_file_ << ", image_topic=" << image_topic_ << ", depth_topic=" << depth_topic_
            << ", camera_info_topic=" << camera_info_topic_ << ", odom_topic=" << odom_topic_
            << ", extrinsic_topic=" << extrinsic_topic_ << ", publish_target_point="
            << publish_target_point_ << ", target_topic=" << target_topic_
            << ", output_world_frame=" << output_world_frame_
            << ", rotate_input_180=" << rotate_input_180_
            << ". Depth must already be aligned to the color image.");
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
        drawDetections(image, detections);
        publishAnnotatedImage(msg->header, image);
        publishMarkers(msg->header, image.size(), detections);
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
    }

    void extrinsicCallback(const nav_msgs::OdometryConstPtr& msg) {
        std::lock_guard<std::mutex> lock(extrinsic_mutex_);
        latest_extrinsic_ = *msg;
        has_extrinsic_    = true;
    }

    LetterboxInfo preprocess(const cv::Mat& image, const cv::Mat& depth) {
        LetterboxInfo info;
        // Match tools/outdoor_rgbd/predict_outdoor_rgbd.py: quantize raw depth before resizing.
        cv::Mat depth_u8(depth.size(), CV_8UC1);
        cv::Mat valid_u8(depth.size(), CV_8UC1);
        for (int y = 0; y < depth.rows; ++y) {
            const uint16_t* raw = depth.ptr<uint16_t>(y);
            uint8_t* normalized = depth_u8.ptr<uint8_t>(y);
            uint8_t* valid = valid_u8.ptr<uint8_t>(y);
            for (int x = 0; x < depth.cols; ++x) {
                const bool is_valid = raw[x] != 0 && raw[x] != 65535;
                valid[x] = is_valid ? 255 : 0;
                const float clipped = std::max(2000.0f, std::min(30000.0f, static_cast<float>(raw[x])));
                const float linear = (clipped - 2000.0f) / 28000.0f;
                normalized[x] = is_valid ? static_cast<uint8_t>(linear * 255.0f) : 0;
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
        cv::Mat rgb_resized, depth_resized, valid_resized;
        cv::resize(rgb, rgb_resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_LINEAR);
        cv::resize(depth_u8, depth_resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_LINEAR);
        cv::resize(valid_u8, valid_resized, cv::Size(resized_w, resized_h), 0, 0, cv::INTER_NEAREST);

        const cv::Rect content(pad_x, pad_y, resized_w, resized_h);
        cv::Mat rgb_padded(input_h_, input_w_, CV_8UC3, cv::Scalar::all(0));
        cv::Mat depth_padded(input_h_, input_w_, CV_8UC1, cv::Scalar::all(0));
        cv::Mat valid_padded(input_h_, input_w_, CV_8UC1, cv::Scalar::all(0));
        rgb_resized.copyTo(rgb_padded(content));
        depth_resized.copyTo(depth_padded(content));
        valid_resized.copyTo(valid_padded(content));

        const int channel_size = input_h_ * input_w_;
        for (int y = 0; y < input_h_; ++y) {
            const cv::Vec3b* row = rgb_padded.ptr<cv::Vec3b>(y);
            const uint8_t* depth_row = depth_padded.ptr<uint8_t>(y);
            const uint8_t* valid_row = valid_padded.ptr<uint8_t>(y);
            for (int x = 0; x < input_w_; ++x) {
                for (int c = 0; c < 3; ++c) {
                    host_input_[c * channel_size + y * input_w_ + x] = row[x][c] / 255.0f;
                }
                host_input_[3 * channel_size + y * input_w_ + x] = depth_row[x] / 255.0f;
                host_input_[4 * channel_size + y * input_w_ + x] = valid_row[x] / 255.0f;
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

    void drawDetections(cv::Mat& image, const std::vector<Detection>& detections) {
        for (const Detection& det : detections) {
            cv::rectangle(image, det.box, cv::Scalar(0, 255, 0), 2);
            char label[64];
            std::snprintf(label, sizeof(label), "target %.2f", det.score);
            const int base_line = 0;
            cv::putText(
                image, label, cv::Point(det.box.x, std::max(20, det.box.y - base_line - 4)),
                cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
        }
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
        if (detections.empty()) {
            return;
        }

        const Detection& target = detections.front();
        const float oriented_u  = target.box.x + target.box.width * 0.5f;
        const float oriented_v  = target.box.y + target.box.height * 0.5f;

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

        const cv::Point2f sensor_pixel = yolo_trt_detector::sensorPixelFromOriented(
            oriented_u, oriented_v, depth.cols, depth.rows, rotate_input_180_);
        const float u = sensor_pixel.x;
        const float v = sensor_pixel.y;

        const float z = medianDepth(depth, depth_encoding, u, v);
        if (z <= 0.0f || !std::isfinite(z)) {
            ROS_WARN_STREAM_THROTTLE(1.0, "invalid depth at target center");
            return;
        }

        const double fx = camera_info.K[0];
        const double fy = camera_info.K[4];
        const double cx = camera_info.K[2];
        const double cy = camera_info.K[5];
        if (fx == 0.0 || fy == 0.0) {
            ROS_WARN_STREAM_THROTTLE(1.0, "invalid camera intrinsics");
            return;
        }

        const tf2::Vector3 target_c((u - cx) * z / fx, (v - cy) * z / fy, z);

        tf2::Matrix3x3 rotation_body_depth = camera_rotation_body_depth_;
        tf2::Vector3 translation_body_depth = camera_translation_body_depth_;
        if (use_vins_extrinsic_) {
            nav_msgs::Odometry extrinsic;
            {
                std::lock_guard<std::mutex> lock(extrinsic_mutex_);
                if (!has_extrinsic_) {
                    ROS_WARN_STREAM_THROTTLE(1.0, "waiting for dynamic camera extrinsic");
                    return;
                }
                extrinsic = latest_extrinsic_;
            }
            const geometry_msgs::Quaternion& q_ext = extrinsic.pose.pose.orientation;
            tf2::Quaternion q_body_depth(q_ext.x, q_ext.y, q_ext.z, q_ext.w);
            if (q_body_depth.length2() < 1e-12) {
                ROS_WARN_STREAM_THROTTLE(1.0, "dynamic camera extrinsic has invalid quaternion");
                return;
            }
            q_body_depth.normalize();
            rotation_body_depth = tf2::Matrix3x3(q_body_depth);
            const geometry_msgs::Point& t_ext = extrinsic.pose.pose.position;
            translation_body_depth.setValue(t_ext.x, t_ext.y, t_ext.z);
        }

        const tf2::Vector3 target_b =
            rotation_body_depth * target_c + translation_body_depth;
        tf2::Vector3 target_output = target_b;
        std::string output_frame   = frame_id_.empty() ? "base_link" : frame_id_;

        if (output_world_frame_) {
            nav_msgs::Odometry odom;
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                if (!has_odom_) {
                    ROS_WARN_STREAM_THROTTLE(1.0, "waiting for odometry");
                    return;
                }
                odom = latest_odom_;
            }
            const geometry_msgs::Quaternion& q_msg = odom.pose.pose.orientation;
            tf2::Quaternion q_world_body(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
            if (q_world_body.length2() < 1e-12) {
                ROS_WARN_STREAM_THROTTLE(1.0, "odometry has invalid quaternion");
                return;
            }
            q_world_body.normalize();
            const geometry_msgs::Point& p_msg = odom.pose.pose.position;
            const tf2::Vector3 position_world_body(p_msg.x, p_msg.y, p_msg.z);
            target_output = tf2::Matrix3x3(q_world_body) * target_b + position_world_body;
            output_frame = frame_id_.empty()
                               ? (odom.header.frame_id.empty() ? global_frame_id_
                                                               : odom.header.frame_id)
                               : frame_id_;
        }

        geometry_msgs::PointStamped point;
        point.header          = header;
        point.header.frame_id = output_frame;
        point.point.x         = target_output.x();
        point.point.y         = target_output.y();
        point.point.z         = target_output.z();

        target_point_pub_.publish(point);
    }

    float medianDepth(const cv::Mat& depth, const std::string& encoding, float u, float v) const {
        std::vector<float> values;
        const int center_x = static_cast<int>(std::round(u));
        const int center_y = static_cast<int>(std::round(v));
        const int radius   = std::max(0, depth_roi_radius_);

        for (int y = std::max(0, center_y - radius);
             y <= std::min(depth.rows - 1, center_y + radius); ++y) {
            for (int x = std::max(0, center_x - radius);
                 x <= std::min(depth.cols - 1, center_x + radius); ++x) {
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
                    return 0.0f;
                }

                if (value > 0.0f && std::isfinite(value)) {
                    values.push_back(value);
                }
            }
        }

        if (values.empty()) {
            return 0.0f;
        }

        const size_t middle = values.size() / 2;
        std::nth_element(values.begin(), values.begin() + middle, values.end());
        return values[middle];
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
