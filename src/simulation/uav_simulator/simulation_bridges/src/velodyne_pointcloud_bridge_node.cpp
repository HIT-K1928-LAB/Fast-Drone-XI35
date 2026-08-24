#include "simulation_bridges/velodyne_pointcloud_converter.h"

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>

#include <string>

class VelodynePointCloudBridgeNode {
public:
    VelodynePointCloudBridgeNode() : private_nh_("~") {
        private_nh_.param<std::string>(
            "input_topic", input_topic_, "/iris_0/velodyne_points");
        private_nh_.param<std::string>(
            "output_topic", output_topic_,
            "/iris_0/velodyne_points_fastlivo");
        private_nh_.param<double>("scan_period", scan_period_, 0.1);

        publisher_ = nh_.advertise<sensor_msgs::PointCloud2>(output_topic_, 10);
        subscriber_ = nh_.subscribe(
            input_topic_, 10, &VelodynePointCloudBridgeNode::pointCloudCallback,
            this, ros::TransportHints().tcpNoDelay());

        ROS_INFO_STREAM(
            "Velodyne FAST-LIVO2 bridge: input=" << input_topic_
                                                  << " output=" << output_topic_
                                                  << " scan_period=" << scan_period_);
    }

private:
    void pointCloudCallback(const sensor_msgs::PointCloud2::ConstPtr &input) {
        sensor_msgs::PointCloud2 output;
        std::string error;
        if (!simulation_bridges::addVelodyneTimeField(
                *input, scan_period_, &output, &error)) {
            ROS_ERROR_THROTTLE(
                1.0, "Failed to convert Velodyne point cloud: %s", error.c_str());
            return;
        }
        publisher_.publish(output);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber subscriber_;
    ros::Publisher publisher_;
    std::string input_topic_;
    std::string output_topic_;
    double scan_period_{};
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "velodyne_pointcloud_bridge");
    VelodynePointCloudBridgeNode node;
    ros::spin();
    return 0;
}
