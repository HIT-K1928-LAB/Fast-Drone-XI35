#include <algorithm>
#include <cmath>
#include <local_occupancy_map/local_occupancy_map.h>
#include <opencv2/core.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include <plan_env/raycast.h>
#include <sensor_msgs/image_encodings.h>
#include <stdexcept>

namespace local_occupancy_map {

LocalOccupancyMap::LocalOccupancyMap() = default;

void LocalOccupancyMap::init(ros::NodeHandle& nh) {
    node_ = nh;

    double map_origin_x = -10.0;
    double map_origin_y = -10.0;
    double map_origin_z = -0.01;
    double map_size_x   = 20.0;
    double map_size_y   = 20.0;
    double map_size_z   = 4.0;

    node_.param("resolution", mp_.resolution, mp_.resolution);
    node_.param("map_origin_x", map_origin_x, map_origin_x);
    node_.param("map_origin_y", map_origin_y, map_origin_y);
    node_.param("map_origin_z", map_origin_z, map_origin_z);
    node_.param("map_size_x", map_size_x, map_size_x);
    node_.param("map_size_y", map_size_y, map_size_y);
    node_.param("map_size_z", map_size_z, map_size_z);
    node_.param("local_update_range_x", mp_.local_update_range(0), mp_.local_update_range(0));
    node_.param("local_update_range_y", mp_.local_update_range(1), mp_.local_update_range(1));
    node_.param("local_update_range_z", mp_.local_update_range(2), mp_.local_update_range(2));

    node_.param("fx", mp_.fx, mp_.fx);
    node_.param("fy", mp_.fy, mp_.fy);
    node_.param("cx", mp_.cx, mp_.cx);
    node_.param("cy", mp_.cy, mp_.cy);
    node_.param("k_depth_scaling_factor", mp_.k_depth_scaling_factor, mp_.k_depth_scaling_factor);
    node_.param("skip_pixel", mp_.skip_pixel, mp_.skip_pixel);
    node_.param("use_depth_filter", mp_.use_depth_filter, mp_.use_depth_filter);
    node_.param("depth_filter_tolerance", mp_.depth_filter_tolerance, mp_.depth_filter_tolerance);
    node_.param("depth_filter_mindist", mp_.depth_filter_mindist, mp_.depth_filter_mindist);
    node_.param("depth_filter_maxdist", mp_.depth_filter_maxdist, mp_.depth_filter_maxdist);
    node_.param("depth_filter_margin", mp_.depth_filter_margin, mp_.depth_filter_margin);
    node_.param("min_ray_length", mp_.min_ray_length, mp_.min_ray_length);
    node_.param("max_ray_length", mp_.max_ray_length, mp_.max_ray_length);

    node_.param("obstacles_inflation", mp_.obstacles_inflation, mp_.obstacles_inflation);
    node_.param("local_map_margin", mp_.local_map_margin, mp_.local_map_margin);
    node_.param(
        "visualization_truncate_height", mp_.visualization_truncate_height,
        mp_.visualization_truncate_height);
    node_.param("ground_height", mp_.ground_height, mp_.ground_height);
    node_.param("frame_id", mp_.frame_id, mp_.frame_id);

    node_.param("p_hit", mp_.p_hit, mp_.p_hit);
    node_.param("p_miss", mp_.p_miss, mp_.p_miss);
    node_.param("p_min", mp_.p_min, mp_.p_min);
    node_.param("p_max", mp_.p_max, mp_.p_max);
    node_.param("p_occ", mp_.p_occ, mp_.p_occ);
    node_.param(
        "camera_extrinsic_config", mp_.camera_extrinsic_config, mp_.camera_extrinsic_config);
    node_.param("camera_extrinsic_key", mp_.camera_extrinsic_key, mp_.camera_extrinsic_key);

    double update_rate  = 20.0;
    double publish_rate = 10.0;
    node_.param("update_rate", update_rate, update_rate);
    node_.param("publish_rate", publish_rate, publish_rate);

    mp_.resolution_inv   = 1.0 / mp_.resolution;
    mp_.map_origin       = Eigen::Vector3d(map_origin_x, map_origin_y, map_origin_z);
    mp_.map_size         = Eigen::Vector3d(map_size_x, map_size_y, map_size_z);
    mp_.map_min_boundary = mp_.map_origin;
    mp_.map_max_boundary = mp_.map_origin + mp_.map_size;
    for (int i = 0; i < 3; ++i) {
        mp_.map_voxel_num(i) = std::ceil(mp_.map_size(i) / mp_.resolution);
    }

    mp_.prob_hit_log      = logit(mp_.p_hit);
    mp_.prob_miss_log     = logit(mp_.p_miss);
    mp_.clamp_min_log     = logit(mp_.p_min);
    mp_.clamp_max_log     = logit(mp_.p_max);
    mp_.min_occupancy_log = logit(mp_.p_occ);

    const int buffer_size = mp_.map_voxel_num(0) * mp_.map_voxel_num(1) * mp_.map_voxel_num(2);
    md_.occupancy_buffer.assign(buffer_size, mp_.clamp_min_log - mp_.unknown_flag);
    md_.occupancy_buffer_inflate.assign(buffer_size, 0);
    md_.count_hit_and_miss.assign(buffer_size, 0);
    md_.count_hit.assign(buffer_size, 0);
    md_.flag_rayend.assign(buffer_size, -1);
    md_.flag_traverse.assign(buffer_size, -1);
    md_.raycast_num = 0;
    md_.proj_points.resize(640 * 480 / std::max(1, mp_.skip_pixel * mp_.skip_pixel));
    md_.proj_points_cnt = 0;

    md_.cam2body << 0.0, 0.0, 1.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, -1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
        1.0;
    loadCameraExtrinsic();

    depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "depth", 20));
    odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "odom", 50));
    sync_image_odom_.reset(new SyncImageOdom(SyncPolicyImageOdom(50), *depth_sub_, *odom_sub_));
    sync_image_odom_->registerCallback(
        boost::bind(&LocalOccupancyMap::depthOdomCallback, this, _1, _2));

    update_timer_ = node_.createTimer(
        ros::Duration(1.0 / update_rate), &LocalOccupancyMap::updateCallback, this);
    publish_timer_ = node_.createTimer(
        ros::Duration(1.0 / publish_rate), &LocalOccupancyMap::publishCallback, this);

    map_pub_     = node_.advertise<sensor_msgs::PointCloud2>("occupancy", 10);
    map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("occupancy_inflate", 10);

    ROS_INFO_STREAM(
        "local_occupancy_map initialized. voxel_num=" << mp_.map_voxel_num.transpose()
                                                      << ", resolution=" << mp_.resolution);
}

void LocalOccupancyMap::depthOdomCallback(
    const sensor_msgs::ImageConstPtr& img, const nav_msgs::OdometryConstPtr& odom) {
    Eigen::Quaterniond body_q(
        odom->pose.pose.orientation.w, odom->pose.pose.orientation.x, odom->pose.pose.orientation.y,
        odom->pose.pose.orientation.z);
    Eigen::Matrix4d body2world   = Eigen::Matrix4d::Identity();
    body2world.block<3, 3>(0, 0) = body_q.toRotationMatrix();
    body2world(0, 3)             = odom->pose.pose.position.x;
    body2world(1, 3)             = odom->pose.pose.position.y;
    body2world(2, 3)             = odom->pose.pose.position.z;

    const Eigen::Matrix4d cam_t = body2world * md_.cam2body;
    md_.camera_pos              = cam_t.block<3, 1>(0, 3);
    md_.camera_r                = cam_t.block<3, 3>(0, 0);

    cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
    if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
        cv_ptr->image.convertTo(md_.depth_image, CV_16UC1, mp_.k_depth_scaling_factor);
    } else if (
        img->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
        img->encoding == sensor_msgs::image_encodings::MONO16) {
        cv_ptr->image.copyTo(md_.depth_image);
    } else {
        ROS_WARN_THROTTLE(1.0, "Unsupported depth encoding: %s", img->encoding.c_str());
        return;
    }

    md_.occ_need_update = true;
}

void LocalOccupancyMap::updateCallback(const ros::TimerEvent&) {
    if (!md_.occ_need_update || md_.depth_image.empty()) return;

    projectDepthImage();
    raycastProcess();
    if (md_.local_updated) clearAndInflateLocalMap();

    md_.occ_need_update = false;
    md_.local_updated   = false;
}

void LocalOccupancyMap::publishCallback(const ros::TimerEvent&) {
    publishInflatedMap();
    publishMap();
}

void LocalOccupancyMap::projectDepthImage() {
    md_.proj_points_cnt = 0;

    const int cols    = md_.depth_image.cols;
    const int rows    = md_.depth_image.rows;
    uint16_t* row_ptr = nullptr;
    double depth      = 0.0;

    if (!mp_.use_depth_filter) {
        const int skip = std::max(1, mp_.skip_pixel);
        for (int v = 0; v < rows; v += skip) {
            row_ptr = md_.depth_image.ptr<uint16_t>(v);

            for (int u = 0; u < cols; u += skip) {
                Eigen::Vector3d proj_pt;
                depth      = (*row_ptr++) / mp_.k_depth_scaling_factor;
                proj_pt(0) = (u - mp_.cx) * depth / mp_.fx;
                proj_pt(1) = (v - mp_.cy) * depth / mp_.fy;
                proj_pt(2) = depth;

                proj_pt                                = md_.camera_r * proj_pt + md_.camera_pos;
                md_.proj_points[md_.proj_points_cnt++] = proj_pt;
            }
        }
    } else {
        if (!md_.has_first_depth) {
            md_.has_first_depth = true;
        } else {
            Eigen::Vector3d pt_cur, pt_world, pt_reproj;
            Eigen::Matrix3d last_camera_r_inv = md_.last_camera_r.inverse();
            const double inv_factor           = 1.0 / mp_.k_depth_scaling_factor;

            for (int v = mp_.depth_filter_margin; v < rows - mp_.depth_filter_margin;
                 v += mp_.skip_pixel) {
                row_ptr = md_.depth_image.ptr<uint16_t>(v) + mp_.depth_filter_margin;

                for (int u = mp_.depth_filter_margin; u < cols - mp_.depth_filter_margin;
                     u += mp_.skip_pixel) {
                    depth   = (*row_ptr) * inv_factor;
                    row_ptr = row_ptr + mp_.skip_pixel;

                    if (*row_ptr == 0) {
                        depth = mp_.max_ray_length + 0.1;
                    } else if (depth < mp_.depth_filter_mindist) {
                        continue;
                    } else if (depth > mp_.depth_filter_maxdist) {
                        depth = mp_.max_ray_length + 0.1;
                    }

                    pt_cur(0) = (u - mp_.cx) * depth / mp_.fx;
                    pt_cur(1) = (v - mp_.cy) * depth / mp_.fy;
                    pt_cur(2) = depth;

                    pt_world                               = md_.camera_r * pt_cur + md_.camera_pos;
                    md_.proj_points[md_.proj_points_cnt++] = pt_world;

                    if (false) {
                        pt_reproj = last_camera_r_inv * (pt_world - md_.last_camera_pos);
                        double uu = pt_reproj.x() * mp_.fx / pt_reproj.z() + mp_.cx;
                        double vv = pt_reproj.y() * mp_.fy / pt_reproj.z() + mp_.cy;

                        if (uu >= 0 && uu < cols && vv >= 0 && vv < rows) {
                            if (std::fabs(
                                    md_.last_depth_image.at<uint16_t>((int)vv, (int)uu) *
                                        inv_factor -
                                    pt_reproj.z()) < mp_.depth_filter_tolerance) {
                                md_.proj_points[md_.proj_points_cnt++] = pt_world;
                            }
                        } else {
                            md_.proj_points[md_.proj_points_cnt++] = pt_world;
                        }
                    }
                }
            }
        }
    }

    md_.last_camera_pos  = md_.camera_pos;
    md_.last_camera_r    = md_.camera_r;
    md_.last_depth_image = md_.depth_image;
}

void LocalOccupancyMap::raycastProcess() {
    if (md_.proj_points_cnt == 0) return;

    md_.raycast_num += 1;

    int vox_idx;
    double length;

    double min_x = mp_.map_max_boundary(0);
    double min_y = mp_.map_max_boundary(1);
    double min_z = mp_.map_max_boundary(2);

    double max_x = mp_.map_min_boundary(0);
    double max_y = mp_.map_min_boundary(1);
    double max_z = mp_.map_min_boundary(2);

    RayCaster raycaster;
    Eigen::Vector3d half = Eigen::Vector3d(0.5, 0.5, 0.5);
    Eigen::Vector3d ray_pt, pt_w;

    for (int i = 0; i < md_.proj_points_cnt; ++i) {
        pt_w = md_.proj_points[i];

        if (!isInMap(pt_w)) {
            pt_w = closestPointInMap(pt_w, md_.camera_pos);

            length = (pt_w - md_.camera_pos).norm();
            if (length > mp_.max_ray_length) {
                pt_w = (pt_w - md_.camera_pos) / length * mp_.max_ray_length + md_.camera_pos;
            }
            vox_idx = setCacheOccupancy(pt_w, 0);
        } else {
            length = (pt_w - md_.camera_pos).norm();

            if (length > mp_.max_ray_length) {
                pt_w    = (pt_w - md_.camera_pos) / length * mp_.max_ray_length + md_.camera_pos;
                vox_idx = setCacheOccupancy(pt_w, 0);
            } else {
                vox_idx = setCacheOccupancy(pt_w, 1);
            }
        }

        max_x = std::max(max_x, pt_w(0));
        max_y = std::max(max_y, pt_w(1));
        max_z = std::max(max_z, pt_w(2));

        min_x = std::min(min_x, pt_w(0));
        min_y = std::min(min_y, pt_w(1));
        min_z = std::min(min_z, pt_w(2));

        if (vox_idx != -10000) {
            if (md_.flag_rayend[vox_idx] == md_.raycast_num) {
                continue;
            } else {
                md_.flag_rayend[vox_idx] = md_.raycast_num;
            }
        }

        raycaster.setInput(pt_w / mp_.resolution, md_.camera_pos / mp_.resolution);

        while (raycaster.step(ray_pt)) {
            Eigen::Vector3d tmp = (ray_pt + half) * mp_.resolution;
            length              = (tmp - md_.camera_pos).norm();

            vox_idx = setCacheOccupancy(tmp, 0);

            if (vox_idx != -10000) {
                if (md_.flag_traverse[vox_idx] == md_.raycast_num) {
                    break;
                } else {
                    md_.flag_traverse[vox_idx] = md_.raycast_num;
                }
            }
        }
    }

    min_x = std::min(min_x, md_.camera_pos(0));
    min_y = std::min(min_y, md_.camera_pos(1));
    min_z = std::min(min_z, md_.camera_pos(2));

    max_x = std::max(max_x, md_.camera_pos(0));
    max_y = std::max(max_y, md_.camera_pos(1));
    max_z = std::max(max_z, md_.camera_pos(2));
    max_z = std::max(max_z, mp_.ground_height);

    posToIndex(Eigen::Vector3d(max_x, max_y, max_z), md_.local_bound_max);
    posToIndex(Eigen::Vector3d(min_x, min_y, min_z), md_.local_bound_min);
    boundIndex(md_.local_bound_min);
    boundIndex(md_.local_bound_max);
    md_.local_updated = true;

    Eigen::Vector3d local_range_min = md_.camera_pos - mp_.local_update_range;
    Eigen::Vector3d local_range_max = md_.camera_pos + mp_.local_update_range;
    Eigen::Vector3i min_id, max_id;
    posToIndex(local_range_min, min_id);
    posToIndex(local_range_max, max_id);
    boundIndex(min_id);
    boundIndex(max_id);

    while (!md_.cache_voxel.empty()) {
        Eigen::Vector3i idx = md_.cache_voxel.front();
        int idx_ctns        = toAddress(idx);
        md_.cache_voxel.pop();

        double log_odds_update =
            md_.count_hit[idx_ctns] >= md_.count_hit_and_miss[idx_ctns] - md_.count_hit[idx_ctns]
                ? mp_.prob_hit_log
                : mp_.prob_miss_log;

        md_.count_hit[idx_ctns] = md_.count_hit_and_miss[idx_ctns] = 0;

        if (log_odds_update >= 0 && md_.occupancy_buffer[idx_ctns] >= mp_.clamp_max_log) {
            continue;
        } else if (log_odds_update <= 0 && md_.occupancy_buffer[idx_ctns] <= mp_.clamp_min_log) {
            md_.occupancy_buffer[idx_ctns] = mp_.clamp_min_log;
            continue;
        }

        bool in_local = idx(0) >= min_id(0) && idx(0) <= max_id(0) && idx(1) >= min_id(1) &&
                        idx(1) <= max_id(1) && idx(2) >= min_id(2) && idx(2) <= max_id(2);
        if (!in_local) {
            md_.occupancy_buffer[idx_ctns] = mp_.clamp_min_log;
        }

        md_.occupancy_buffer[idx_ctns] = std::min(
            std::max(md_.occupancy_buffer[idx_ctns] + log_odds_update, mp_.clamp_min_log),
            mp_.clamp_max_log);
    }
}

void LocalOccupancyMap::clearAndInflateLocalMap() {
    const int vec_margin = 5;

    Eigen::Vector3i min_cut =
        md_.local_bound_min -
        Eigen::Vector3i(mp_.local_map_margin, mp_.local_map_margin, mp_.local_map_margin);
    Eigen::Vector3i max_cut =
        md_.local_bound_max +
        Eigen::Vector3i(mp_.local_map_margin, mp_.local_map_margin, mp_.local_map_margin);
    boundIndex(min_cut);
    boundIndex(max_cut);

    Eigen::Vector3i min_cut_m = min_cut - Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    Eigen::Vector3i max_cut_m = max_cut + Eigen::Vector3i(vec_margin, vec_margin, vec_margin);
    boundIndex(min_cut_m);
    boundIndex(max_cut_m);

    for (int x = min_cut_m(0); x <= max_cut_m(0); ++x)
        for (int y = min_cut_m(1); y <= max_cut_m(1); ++y) {
            for (int z = min_cut_m(2); z < min_cut(2); ++z) {
                md_.occupancy_buffer[toAddress(x, y, z)] = mp_.clamp_min_log - mp_.unknown_flag;
            }

            for (int z = max_cut(2) + 1; z <= max_cut_m(2); ++z) {
                md_.occupancy_buffer[toAddress(x, y, z)] = mp_.clamp_min_log - mp_.unknown_flag;
            }
        }

    for (int z = min_cut_m(2); z <= max_cut_m(2); ++z)
        for (int x = min_cut_m(0); x <= max_cut_m(0); ++x) {
            for (int y = min_cut_m(1); y < min_cut(1); ++y) {
                md_.occupancy_buffer[toAddress(x, y, z)] = mp_.clamp_min_log - mp_.unknown_flag;
            }

            for (int y = max_cut(1) + 1; y <= max_cut_m(1); ++y) {
                md_.occupancy_buffer[toAddress(x, y, z)] = mp_.clamp_min_log - mp_.unknown_flag;
            }
        }

    for (int y = min_cut_m(1); y <= max_cut_m(1); ++y)
        for (int z = min_cut_m(2); z <= max_cut_m(2); ++z) {
            for (int x = min_cut_m(0); x < min_cut(0); ++x) {
                md_.occupancy_buffer[toAddress(x, y, z)] = mp_.clamp_min_log - mp_.unknown_flag;
            }

            for (int x = max_cut(0) + 1; x <= max_cut_m(0); ++x) {
                md_.occupancy_buffer[toAddress(x, y, z)] = mp_.clamp_min_log - mp_.unknown_flag;
            }
        }

    const int inf_step = std::ceil(mp_.obstacles_inflation / mp_.resolution);
    std::vector<Eigen::Vector3i> inf_pts(std::pow(2 * inf_step + 1, 3));

    for (int x = md_.local_bound_min(0); x <= md_.local_bound_max(0); ++x)
        for (int y = md_.local_bound_min(1); y <= md_.local_bound_max(1); ++y)
            for (int z = md_.local_bound_min(2); z <= md_.local_bound_max(2); ++z) {
                md_.occupancy_buffer_inflate[toAddress(x, y, z)] = 0;
            }

    for (int x = md_.local_bound_min(0); x <= md_.local_bound_max(0); ++x)
        for (int y = md_.local_bound_min(1); y <= md_.local_bound_max(1); ++y)
            for (int z = md_.local_bound_min(2); z <= md_.local_bound_max(2); ++z) {
                if (md_.occupancy_buffer[toAddress(x, y, z)] <= mp_.min_occupancy_log) continue;
                inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
                for (const auto& inf_pt : inf_pts) {
                    if (!isInMap(inf_pt)) continue;
                    int idx_inf                           = toAddress(inf_pt);
                    md_.occupancy_buffer_inflate[idx_inf] = 1;
                }
            }
}

void LocalOccupancyMap::publishMap() {
    if (map_pub_.getNumSubscribers() <= 0) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3i min_id = md_.local_bound_min;
    Eigen::Vector3i max_id = md_.local_bound_max;

    const int lmm = mp_.local_map_margin / 2;
    min_id -= Eigen::Vector3i(lmm, lmm, lmm);
    max_id += Eigen::Vector3i(lmm, lmm, lmm);

    boundIndex(min_id);
    boundIndex(max_id);

    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
            for (int z = min_id(2); z <= max_id(2); ++z) {
                if (md_.occupancy_buffer[toAddress(x, y, z)] <= mp_.min_occupancy_log) continue;
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                if (pos(2) > mp_.visualization_truncate_height) continue;
                cloud.push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            }

    cloud.width           = cloud.points.size();
    cloud.height          = 1;
    cloud.is_dense        = true;
    cloud.header.frame_id = mp_.frame_id;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    map_pub_.publish(msg);
}

void LocalOccupancyMap::publishInflatedMap() {
    if (map_inf_pub_.getNumSubscribers() <= 0) return;

    pcl::PointCloud<pcl::PointXYZ> cloud;
    Eigen::Vector3i min_id =
        md_.local_bound_min -
        Eigen::Vector3i(mp_.local_map_margin, mp_.local_map_margin, mp_.local_map_margin);
    Eigen::Vector3i max_id =
        md_.local_bound_max +
        Eigen::Vector3i(mp_.local_map_margin, mp_.local_map_margin, mp_.local_map_margin);
    boundIndex(min_id);
    boundIndex(max_id);

    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
            for (int z = min_id(2); z <= max_id(2); ++z) {
                if (md_.occupancy_buffer_inflate[toAddress(x, y, z)] == 0) continue;
                Eigen::Vector3d pos;
                indexToPos(Eigen::Vector3i(x, y, z), pos);
                if (pos(2) > mp_.visualization_truncate_height) continue;
                cloud.push_back(pcl::PointXYZ(pos(0), pos(1), pos(2)));
            }

    cloud.width           = cloud.points.size();
    cloud.height          = 1;
    cloud.is_dense        = true;
    cloud.header.frame_id = mp_.frame_id;

    sensor_msgs::PointCloud2 msg;
    pcl::toROSMsg(cloud, msg);
    msg.header.stamp = ros::Time::now();
    map_inf_pub_.publish(msg);
}

bool LocalOccupancyMap::isInMap(const Eigen::Vector3d& pos) const {
    if (pos(0) < mp_.map_min_boundary(0) + 1e-4 || pos(1) < mp_.map_min_boundary(1) + 1e-4 ||
        pos(2) < mp_.map_min_boundary(2) + 1e-4) {
        return false;
    }
    if (pos(0) > mp_.map_max_boundary(0) - 1e-4 || pos(1) > mp_.map_max_boundary(1) - 1e-4 ||
        pos(2) > mp_.map_max_boundary(2) - 1e-4) {
        return false;
    }
    return true;
}

bool LocalOccupancyMap::isInMap(const Eigen::Vector3i& idx) const {
    return idx(0) >= 0 && idx(0) < mp_.map_voxel_num(0) && idx(1) >= 0 &&
           idx(1) < mp_.map_voxel_num(1) && idx(2) >= 0 && idx(2) < mp_.map_voxel_num(2);
}

void LocalOccupancyMap::posToIndex(const Eigen::Vector3d& pos, Eigen::Vector3i& id) const {
    for (int i = 0; i < 3; ++i) {
        id(i) = std::floor((pos(i) - mp_.map_origin(i)) * mp_.resolution_inv);
    }
}

void LocalOccupancyMap::indexToPos(const Eigen::Vector3i& id, Eigen::Vector3d& pos) const {
    for (int i = 0; i < 3; ++i) {
        pos(i) = (id(i) + 0.5) * mp_.resolution + mp_.map_origin(i);
    }
}

int LocalOccupancyMap::toAddress(const Eigen::Vector3i& id) const {
    return toAddress(id(0), id(1), id(2));
}

int LocalOccupancyMap::toAddress(int x, int y, int z) const {
    return x * mp_.map_voxel_num(1) * mp_.map_voxel_num(2) + y * mp_.map_voxel_num(2) + z;
}

void LocalOccupancyMap::boundIndex(Eigen::Vector3i& id) const {
    for (int i = 0; i < 3; ++i) {
        id(i) = std::max(0, std::min(id(i), mp_.map_voxel_num(i) - 1));
    }
}

Eigen::Vector3d LocalOccupancyMap::closestPointInMap(
    const Eigen::Vector3d& pt, const Eigen::Vector3d& camera_pt) const {
    Eigen::Vector3d diff   = pt - camera_pt;
    Eigen::Vector3d max_tc = mp_.map_max_boundary - camera_pt;
    Eigen::Vector3d min_tc = mp_.map_min_boundary - camera_pt;
    double min_t           = 1000000.0;

    for (int i = 0; i < 3; ++i) {
        if (std::fabs(diff(i)) <= 0.0) continue;
        const double t1 = max_tc(i) / diff(i);
        const double t2 = min_tc(i) / diff(i);
        if (t1 > 0.0 && t1 < min_t) min_t = t1;
        if (t2 > 0.0 && t2 < min_t) min_t = t2;
    }

    return camera_pt + (min_t - 1e-3) * diff;
}

void LocalOccupancyMap::inflatePoint(
    const Eigen::Vector3i& pt, int step, std::vector<Eigen::Vector3i>& pts) const {
    int num = 0;
    for (int x = -step; x <= step; ++x)
        for (int y = -step; y <= step; ++y)
            for (int z = -step; z <= step; ++z) pts[num++] = pt + Eigen::Vector3i(x, y, z);
}

double LocalOccupancyMap::logit(double p) const { return std::log(p / (1.0 - p)); }

void LocalOccupancyMap::loadCameraExtrinsic() {
    if (mp_.camera_extrinsic_config.empty()) {
        ROS_WARN("No camera_extrinsic_config set. Using built-in body_T_cam fallback.");
        return;
    }

    cv::FileStorage fs(mp_.camera_extrinsic_config, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error(
            "failed to open camera extrinsic config: " + mp_.camera_extrinsic_config);
    }

    cv::Mat cv_t;
    fs[mp_.camera_extrinsic_key] >> cv_t;
    if (cv_t.empty() || cv_t.rows != 4 || cv_t.cols != 4) {
        throw std::runtime_error(
            "invalid camera extrinsic key '" + mp_.camera_extrinsic_key + "' in " +
            mp_.camera_extrinsic_config);
    }

    cv_t.convertTo(cv_t, CV_64F);
    for (int r = 0; r < 4; ++r)
        for (int c = 0; c < 4; ++c) md_.cam2body(r, c) = cv_t.at<double>(r, c);

    ROS_INFO_STREAM(
        "Loaded " << mp_.camera_extrinsic_key << " from " << mp_.camera_extrinsic_config << ":\n"
                  << md_.cam2body);
}

void LocalOccupancyMap::resetBuffer(
    const Eigen::Vector3d& min_pos, const Eigen::Vector3d& max_pos) {
    Eigen::Vector3i min_id, max_id;
    posToIndex(min_pos, min_id);
    posToIndex(max_pos, max_id);

    boundIndex(min_id);
    boundIndex(max_id);

    for (int x = min_id(0); x <= max_id(0); ++x)
        for (int y = min_id(1); y <= max_id(1); ++y)
            for (int z = min_id(2); z <= max_id(2); ++z)
                md_.occupancy_buffer_inflate[toAddress(x, y, z)] = 0;
}

int LocalOccupancyMap::setCacheOccupancy(const Eigen::Vector3d& pos, int occ) {
    if (occ != 1 && occ != 0) return -10000;

    Eigen::Vector3i id;
    posToIndex(pos, id);
    if (!isInMap(id)) return -10000;

    int idx_ctns = toAddress(id);
    md_.count_hit_and_miss[idx_ctns] += 1;

    if (md_.count_hit_and_miss[idx_ctns] == 1) {
        md_.cache_voxel.push(id);
    }

    if (occ == 1) md_.count_hit[idx_ctns] += 1;

    return idx_ctns;
}

}  // namespace local_occupancy_map
