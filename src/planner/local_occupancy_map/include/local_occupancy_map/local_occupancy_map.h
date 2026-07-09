#pragma once

#include <Eigen/Eigen>
#include <climits>
#include <cv_bridge/cv_bridge.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <nav_msgs/Odometry.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <string>
#include <vector>

namespace local_occupancy_map {

class LocalOccupancyMap {
  public:
    LocalOccupancyMap();

    void init(ros::NodeHandle& nh);

  private:
    struct MapParams {
        double resolution                  = 0.15;
        double resolution_inv              = 1.0 / 0.15;
        Eigen::Vector3d map_origin         = Eigen::Vector3d(-10.0, -10.0, -0.01);
        Eigen::Vector3d map_size           = Eigen::Vector3d(20.0, 20.0, 4.0);
        Eigen::Vector3i map_voxel_num      = Eigen::Vector3i::Zero();
        Eigen::Vector3d map_min_boundary   = Eigen::Vector3d::Zero();
        Eigen::Vector3d map_max_boundary   = Eigen::Vector3d::Zero();
        Eigen::Vector3d local_update_range = Eigen::Vector3d(5.5, 5.5, 4.5);

        double fx                     = 387.229248046875;
        double fy                     = 387.229248046875;
        double cx                     = 321.04638671875;
        double cy                     = 243.44969177246094;
        double k_depth_scaling_factor = 1000.0;
        int skip_pixel                = 2;

        bool use_depth_filter         = true;
        double depth_filter_mindist   = 0.2;
        double depth_filter_tolerance = 0.0;
        double depth_filter_maxdist   = 5.0;
        int depth_filter_margin       = 2;
        double max_ray_length         = 5.0;
        double min_ray_length         = 0.3;

        double obstacles_inflation           = 0.12;
        int local_map_margin                 = 30;
        double visualization_truncate_height = 3.0;
        double ground_height                 = -0.01;

        double p_hit             = 0.65;
        double p_miss            = 0.35;
        double p_min             = 0.12;
        double p_max             = 0.90;
        double p_occ             = 0.80;
        double prob_hit_log      = 0.0;
        double prob_miss_log     = 0.0;
        double clamp_min_log     = 0.0;
        double clamp_max_log     = 0.0;
        double min_occupancy_log = 0.0;
        double unknown_flag      = 0.01;

        std::string frame_id = "world";
        std::string camera_extrinsic_config;
        std::string camera_extrinsic_key = "body_T_cam0";
    };

    struct MapData {
        std::vector<double> occupancy_buffer;
        std::vector<char> occupancy_buffer_inflate;

        cv::Mat depth_image;
        cv::Mat last_depth_image;
        Eigen::Vector3d camera_pos      = Eigen::Vector3d::Zero();
        Eigen::Vector3d last_camera_pos = Eigen::Vector3d::Zero();
        Eigen::Matrix3d camera_r        = Eigen::Matrix3d::Identity();
        Eigen::Matrix3d last_camera_r   = Eigen::Matrix3d::Identity();
        Eigen::Matrix4d cam2body        = Eigen::Matrix4d::Identity();

        std::vector<Eigen::Vector3d> proj_points;
        int proj_points_cnt             = 0;
        Eigen::Vector3i local_bound_min = Eigen::Vector3i::Zero();
        Eigen::Vector3i local_bound_max = Eigen::Vector3i::Zero();

        std::vector<short> count_hit;
        std::vector<short> count_hit_and_miss;
        std::vector<char> flag_traverse;
        std::vector<char> flag_rayend;
        char raycast_num = 0;
        std::queue<Eigen::Vector3i> cache_voxel;

        bool occ_need_update = false;
        bool local_updated   = false;
        bool has_first_depth = false;
    };

    using SyncPolicyImageOdom =
        message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, nav_msgs::Odometry>;
    using SyncImageOdom = message_filters::Synchronizer<SyncPolicyImageOdom>;

    void depthOdomCallback(
        const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom);
    void updateCallback(const ros::TimerEvent& event);
    void publishCallback(const ros::TimerEvent& event);

    void projectDepthImage();
    void raycastProcess();
    void clearAndInflateLocalMap();
    void publishMap();
    void publishInflatedMap();

    void loadCameraExtrinsic();
    void resetBuffer(const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos);
    int setCacheOccupancy(const Eigen::Vector3d& pos, int occ);
    bool isInMap(const Eigen::Vector3d& pos) const;
    bool isInMap(const Eigen::Vector3i& idx) const;
    void posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) const;
    void indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) const;
    int toAddress(const Eigen::Vector3i& id) const;
    int toAddress(int x, int y, int z) const;
    void boundIndex(Eigen::Vector3i& id) const;
    Eigen::Vector3d closestPointInMap(
        const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) const;
    void inflatePoint(const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts) const;
    double logit(double p) const;

    ros::NodeHandle node_;
    MapParams mp_;
    MapData md_;

    std::shared_ptr<message_filters::Subscriber<sensor_msgs::Image>> depth_sub_;
    std::shared_ptr<message_filters::Subscriber<nav_msgs::Odometry>> odom_sub_;
    std::shared_ptr<SyncImageOdom> sync_image_odom_;

    ros::Timer update_timer_;
    ros::Timer publish_timer_;
    ros::Publisher map_pub_;
    ros::Publisher map_inf_pub_;
};

}  // namespace local_occupancy_map
