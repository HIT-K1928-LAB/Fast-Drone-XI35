#include <cstdint>

#include <gtest/gtest.h>
#include <opencv2/core.hpp>

#include <yolo_trt_detector/image_orientation.h>

TEST(ImageOrientation, RotatesRgbAndDepthTogetherWithoutChangingSensorInputs) {
    cv::Mat image(2, 3, CV_8UC3);
    cv::Mat depth(2, 3, CV_16UC1);
    for (int y = 0; y < image.rows; ++y) {
        for (int x = 0; x < image.cols; ++x) {
            const uint8_t value = static_cast<uint8_t>(10 * y + x);
            image.at<cv::Vec3b>(y, x) = cv::Vec3b(value, value + 20, value + 40);
            depth.at<uint16_t>(y, x) = static_cast<uint16_t>(1000 + 10 * y + x);
        }
    }

    const yolo_trt_detector::OrientedRgbdFrame rotated =
        yolo_trt_detector::orientRgbdInputs(image, depth, true);

    EXPECT_EQ(image.at<cv::Vec3b>(1, 2), rotated.image.at<cv::Vec3b>(0, 0));
    EXPECT_EQ(depth.at<uint16_t>(1, 2), rotated.depth.at<uint16_t>(0, 0));
    EXPECT_EQ(0, image.at<cv::Vec3b>(0, 0)[0]);
    EXPECT_EQ(1000, depth.at<uint16_t>(0, 0));
}

TEST(ImageOrientation, MapsRotatedDetectionPixelBackToSensorImage) {
    const cv::Point2f sensor_pixel =
        yolo_trt_detector::sensorPixelFromOriented(0.0f, 0.0f, 3, 2, true);

    EXPECT_FLOAT_EQ(2.0f, sensor_pixel.x);
    EXPECT_FLOAT_EQ(1.0f, sensor_pixel.y);
}

TEST(ImageOrientation, LeavesInputsAndPixelsUnchangedWhenDisabled) {
    const cv::Mat image(2, 3, CV_8UC3, cv::Scalar(1, 2, 3));
    const cv::Mat depth(2, 3, CV_16UC1, cv::Scalar(1234));
    const yolo_trt_detector::OrientedRgbdFrame unchanged =
        yolo_trt_detector::orientRgbdInputs(image, depth, false);
    const cv::Point2f unchanged_pixel =
        yolo_trt_detector::sensorPixelFromOriented(1.25f, 0.5f, 3, 2, false);

    EXPECT_EQ(image.at<cv::Vec3b>(0, 0), unchanged.image.at<cv::Vec3b>(0, 0));
    EXPECT_EQ(depth.at<uint16_t>(0, 0), unchanged.depth.at<uint16_t>(0, 0));
    EXPECT_FLOAT_EQ(1.25f, unchanged_pixel.x);
    EXPECT_FLOAT_EQ(0.5f, unchanged_pixel.y);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
