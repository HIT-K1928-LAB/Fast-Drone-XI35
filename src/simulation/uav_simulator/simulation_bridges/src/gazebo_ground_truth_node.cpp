#include "simulation_bridges/converters.h"

#include <gazebo_msgs/ModelStates.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <string>

class GazeboGroundTruthNode {
public:
    GazeboGroundTruthNode() : private_nh_("~") {
        private_nh_.param<std::string>("model_name", model_name_, "iris_0");
        private_nh_.param<std::string>("input_topic", input_topic_, "/gazebo/model_states");
        private_nh_.param<std::string>(
            "output_topic", output_topic_, "/gazebo/iris_0/odometry");
        private_nh_.param<std::string>("frame_id", frame_id_, "world");
        private_nh_.param<std::string>("child_frame_id", child_frame_id_, "base_link");

        odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_topic_, 10);
        model_states_sub_ = nh_.subscribe(
            input_topic_, 1, &GazeboGroundTruthNode::modelStatesCallback, this,
            ros::TransportHints().tcpNoDelay());

        ROS_INFO_STREAM(
            "Gazebo ground truth: model=" << model_name_ << " input=" << input_topic_
                                          << " output=" << output_topic_);
    }

private:
    void modelStatesCallback(const gazebo_msgs::ModelStates::ConstPtr &states) {
        nav_msgs::Odometry odom;
        if (!simulation_bridges::modelStateToOdometry(
                *states, model_name_, ros::Time::now(), frame_id_, child_frame_id_, &odom)) {
            ROS_WARN_THROTTLE(
                2.0, "Gazebo model '%s' is not available in %s", model_name_.c_str(),
                input_topic_.c_str());
            return;
        }
        odom_pub_.publish(odom);
    }

    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber model_states_sub_;
    ros::Publisher odom_pub_;
    std::string model_name_;
    std::string input_topic_;
    std::string output_topic_;
    std::string frame_id_;
    std::string child_frame_id_;
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "gazebo_ground_truth");
    GazeboGroundTruthNode node;
    ros::spin();
    return 0;
}
