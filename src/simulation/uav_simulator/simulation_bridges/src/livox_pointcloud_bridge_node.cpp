#include "simulation_bridges/converters.h"

#include <livox_ros_driver2/CustomMsg.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <string>

class LivoxPointCloudBridgeNode {
public:
    LivoxPointCloudBridgeNode() : private_nh_("~") {
        private_nh_.param<std::string>("input_topic", input_topic_, "/scan");
        private_nh_.param<std::string>("output_topic", output_topic_, "/livox/lidar");
        private_nh_.param<double>("scan_period", scan_period_, 0.1);
        private_nh_.param<int>("line_count", line_count_, 4);

        scan_period_ = std::max(0.0, scan_period_);
        line_count_ = std::max(1, std::min(255, line_count_));
        scan_period_ns_ = static_cast<std::uint32_t>(
            std::min(4.0e9, std::round(scan_period_ * 1.0e9)));

        custom_pub_ = nh_.advertise<livox_ros_driver2::CustomMsg>(output_topic_, 10);
        pointcloud_sub_ = nh_.subscribe(
            input_topic_, 10, &LivoxPointCloudBridgeNode::pointCloudCallback, this,
            ros::TransportHints().tcpNoDelay());

        ROS_INFO_STREAM(
            "Livox simulation bridge: input=" << input_topic_ << " output=" << output_topic_
                                               << " scan_period=" << scan_period_
                                               << " line_count=" << line_count_);
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud::ConstPtr &cloud) {
        livox_ros_driver2::CustomMsg custom;
        simulation_bridges::pointCloudToLivoxCustom(
            *cloud, scan_period_ns_, static_cast<std::uint32_t>(line_count_), &custom);
        custom_pub_.publish(custom);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber pointcloud_sub_;
    ros::Publisher custom_pub_;
    std::string input_topic_;
    std::string output_topic_;
    double scan_period_{};
    int line_count_{};
    std::uint32_t scan_period_ns_{};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "livox_pointcloud_bridge");
    LivoxPointCloudBridgeNode node;
    ros::spin();
    return 0;
}
