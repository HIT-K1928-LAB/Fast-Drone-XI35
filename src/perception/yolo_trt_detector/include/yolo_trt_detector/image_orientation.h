#pragma once

#include <opencv2/core.hpp>

namespace yolo_trt_detector {

struct OrientedRgbdFrame {
    cv::Mat image;
    cv::Mat depth;
};

inline OrientedRgbdFrame orientRgbdInputs(
    const cv::Mat& image, const cv::Mat& depth, bool rotate_180) {
    OrientedRgbdFrame frame;
    if (rotate_180) {
        cv::rotate(image, frame.image, cv::ROTATE_180);
        cv::rotate(depth, frame.depth, cv::ROTATE_180);
    } else {
        frame.image = image.clone();
        frame.depth = depth;
    }
    return frame;
}

inline cv::Point2f sensorPixelFromOriented(
    float u, float v, int width, int height, bool rotate_180) {
    if (!rotate_180) {
        return cv::Point2f(u, v);
    }
    return cv::Point2f(
        static_cast<float>(width - 1) - u,
        static_cast<float>(height - 1) - v);
}

}  // namespace yolo_trt_detector
