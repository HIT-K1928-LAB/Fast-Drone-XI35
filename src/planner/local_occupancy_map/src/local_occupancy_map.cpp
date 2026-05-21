#include <local_occupancy_map/local_occupancy_map.h>

#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/image_encodings.h>

#include <algorithm>
#include <cmath>

namespace local_occupancy_map {

LocalOccupancyMap::LocalOccupancyMap() = default;

void LocalOccupancyMap::init(ros::NodeHandle& nh) {
  node_ = nh;

  double map_origin_x = -10.0;
  double map_origin_y = -10.0;
  double map_origin_z = -0.01;
  double map_size_x = 20.0;
  double map_size_y = 20.0;
  double map_size_z = 4.0;

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
  node_.param("depth_filter_mindist", mp_.depth_filter_mindist, mp_.depth_filter_mindist);
  node_.param("depth_filter_maxdist", mp_.depth_filter_maxdist, mp_.depth_filter_maxdist);
  node_.param("depth_filter_margin", mp_.depth_filter_margin, mp_.depth_filter_margin);
  node_.param("min_ray_length", mp_.min_ray_length, mp_.min_ray_length);
  node_.param("max_ray_length", mp_.max_ray_length, mp_.max_ray_length);

  node_.param("obstacles_inflation", mp_.obstacles_inflation, mp_.obstacles_inflation);
  node_.param("local_map_margin", mp_.local_map_margin, mp_.local_map_margin);
  node_.param("visualization_truncate_height", mp_.visualization_truncate_height,
              mp_.visualization_truncate_height);
  node_.param("ground_height", mp_.ground_height, mp_.ground_height);
  node_.param("frame_id", mp_.frame_id, mp_.frame_id);

  node_.param("p_hit", mp_.p_hit, mp_.p_hit);
  node_.param("p_miss", mp_.p_miss, mp_.p_miss);
  node_.param("p_min", mp_.p_min, mp_.p_min);
  node_.param("p_max", mp_.p_max, mp_.p_max);
  node_.param("p_occ", mp_.p_occ, mp_.p_occ);

  double update_rate = 20.0;
  double publish_rate = 10.0;
  node_.param("update_rate", update_rate, update_rate);
  node_.param("publish_rate", publish_rate, publish_rate);

  mp_.resolution_inv = 1.0 / mp_.resolution;
  mp_.map_origin = Eigen::Vector3d(map_origin_x, map_origin_y, map_origin_z);
  mp_.map_size = Eigen::Vector3d(map_size_x, map_size_y, map_size_z);
  mp_.map_min_boundary = mp_.map_origin;
  mp_.map_max_boundary = mp_.map_origin + mp_.map_size;
  for (int i = 0; i < 3; ++i) {
    mp_.map_voxel_num(i) = std::ceil(mp_.map_size(i) / mp_.resolution);
  }

  mp_.prob_hit_log = logit(mp_.p_hit);
  mp_.prob_miss_log = logit(mp_.p_miss);
  mp_.clamp_min_log = logit(mp_.p_min);
  mp_.clamp_max_log = logit(mp_.p_max);
  mp_.min_occupancy_log = logit(mp_.p_occ);

  const int buffer_size = mp_.map_voxel_num(0) * mp_.map_voxel_num(1) * mp_.map_voxel_num(2);
  md_.occupancy_buffer.assign(buffer_size, mp_.clamp_min_log - mp_.unknown_flag);
  md_.occupancy_buffer_inflate.assign(buffer_size, 0);
  md_.proj_points.reserve(640 * 480 / std::max(1, mp_.skip_pixel * mp_.skip_pixel));

  md_.cam2body << 0.0, 0.0, 1.0, 0.0,
      -1.0, 0.0, 0.0, 0.0,
      0.0, -1.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 1.0;

  depth_sub_.reset(new message_filters::Subscriber<sensor_msgs::Image>(node_, "depth", 20));
  odom_sub_.reset(new message_filters::Subscriber<nav_msgs::Odometry>(node_, "odom", 50));
  sync_image_odom_.reset(new SyncImageOdom(SyncPolicyImageOdom(50), *depth_sub_, *odom_sub_));
  sync_image_odom_->registerCallback(
      boost::bind(&LocalOccupancyMap::depthOdomCallback, this, _1, _2));

  update_timer_ = node_.createTimer(ros::Duration(1.0 / update_rate),
                                   &LocalOccupancyMap::updateCallback, this);
  publish_timer_ = node_.createTimer(ros::Duration(1.0 / publish_rate),
                                    &LocalOccupancyMap::publishCallback, this);

  map_pub_ = node_.advertise<sensor_msgs::PointCloud2>("occupancy", 10);
  map_inf_pub_ = node_.advertise<sensor_msgs::PointCloud2>("occupancy_inflate", 10);

  ROS_INFO_STREAM("local_occupancy_map initialized. voxel_num="
                  << mp_.map_voxel_num.transpose() << ", resolution=" << mp_.resolution);
}

void LocalOccupancyMap::depthOdomCallback(const sensor_msgs::ImageConstPtr& img,
                                          const nav_msgs::OdometryConstPtr& odom) {
  Eigen::Quaterniond body_q(odom->pose.pose.orientation.w, odom->pose.pose.orientation.x,
                            odom->pose.pose.orientation.y, odom->pose.pose.orientation.z);
  Eigen::Matrix4d body2world = Eigen::Matrix4d::Identity();
  body2world.block<3, 3>(0, 0) = body_q.toRotationMatrix();
  body2world(0, 3) = odom->pose.pose.position.x;
  body2world(1, 3) = odom->pose.pose.position.y;
  body2world(2, 3) = odom->pose.pose.position.z;

  const Eigen::Matrix4d cam_t = body2world * md_.cam2body;
  md_.camera_pos = cam_t.block<3, 1>(0, 3);
  md_.camera_r = cam_t.block<3, 3>(0, 0);

  cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(img, img->encoding);
  if (img->encoding == sensor_msgs::image_encodings::TYPE_32FC1) {
    cv_ptr->image.convertTo(md_.depth_image, CV_16UC1, mp_.k_depth_scaling_factor);
  } else if (img->encoding == sensor_msgs::image_encodings::TYPE_16UC1 ||
             img->encoding == sensor_msgs::image_encodings::MONO16) {
    cv_ptr->image.copyTo(md_.depth_image);
  } else {
    ROS_WARN_THROTTLE(1.0, "Unsupported depth encoding: %s", img->encoding.c_str());
    return;
  }

  md_.occ_need_update = isInMap(md_.camera_pos);
  md_.has_first_depth = true;
}

void LocalOccupancyMap::updateCallback(const ros::TimerEvent&) {
  if (!md_.occ_need_update || md_.depth_image.empty()) return;

  projectDepthImage();
  raycastProcess();
  clearAndInflateLocalMap();

  md_.occ_need_update = false;
}

void LocalOccupancyMap::publishCallback(const ros::TimerEvent&) {
  publishInflatedMap();
  publishMap();
}

void LocalOccupancyMap::projectDepthImage() {
  md_.proj_points.clear();

  const int cols = md_.depth_image.cols;
  const int rows = md_.depth_image.rows;
  const int skip = std::max(1, mp_.skip_pixel);
  const int margin = mp_.use_depth_filter ? std::max(0, mp_.depth_filter_margin) : 0;

  for (int v = margin; v < rows - margin; v += skip) {
    const uint16_t* row_ptr = md_.depth_image.ptr<uint16_t>(v);
    for (int u = margin; u < cols - margin; u += skip) {
      double depth = row_ptr[u] / mp_.k_depth_scaling_factor;
      if (depth <= 1e-3) {
        if (!mp_.use_depth_filter) continue;
        depth = mp_.max_ray_length + 0.1;
      } else if (mp_.use_depth_filter && depth < mp_.depth_filter_mindist) {
        continue;
      } else if (mp_.use_depth_filter && depth > mp_.depth_filter_maxdist) {
        depth = mp_.max_ray_length + 0.1;
      }

      Eigen::Vector3d pt_camera;
      pt_camera(0) = (u - mp_.cx) * depth / mp_.fx;
      pt_camera(1) = (v - mp_.cy) * depth / mp_.fy;
      pt_camera(2) = depth;

      md_.proj_points.push_back(md_.camera_r * pt_camera + md_.camera_pos);
    }
  }
}

void LocalOccupancyMap::raycastProcess() {
  if (md_.proj_points.empty()) return;

  const double step = mp_.resolution * 0.5;
  for (Eigen::Vector3d pt_w : md_.proj_points) {
    bool endpoint_occupied = true;
    if (!isInMap(pt_w)) {
      pt_w = closestPointInMap(pt_w, md_.camera_pos);
      endpoint_occupied = false;
    }

    Eigen::Vector3d ray = pt_w - md_.camera_pos;
    double length = ray.norm();
    if (length < mp_.min_ray_length) continue;
    if (length > mp_.max_ray_length) {
      pt_w = md_.camera_pos + ray / length * mp_.max_ray_length;
      ray = pt_w - md_.camera_pos;
      length = mp_.max_ray_length;
      endpoint_occupied = false;
    }

    Eigen::Vector3i last_free_id(INT_MIN, INT_MIN, INT_MIN);
    const int steps = std::max(1, static_cast<int>(std::floor(length / step)));
    for (int i = 1; i < steps; ++i) {
      Eigen::Vector3d sample = md_.camera_pos + ray * (static_cast<double>(i) / steps);
      Eigen::Vector3i id;
      posToIndex(sample, id);
      if (!isInMap(id) || id == last_free_id) continue;
      setOccupancyLogOdds(id, mp_.prob_miss_log);
      last_free_id = id;
    }

    if (endpoint_occupied) {
      Eigen::Vector3i id;
      posToIndex(pt_w, id);
      if (isInMap(id)) setOccupancyLogOdds(id, mp_.prob_hit_log);
    }
  }

  // Keep the visible/local map window fixed around the vehicle, instead of shrinking
  // it to the current camera frustum. This matches the EGO-style local map behavior
  // better when the vehicle yaws in place.
  posToIndex(md_.camera_pos - mp_.local_update_range, md_.local_bound_min);
  posToIndex(md_.camera_pos + mp_.local_update_range, md_.local_bound_max);
  boundIndex(md_.local_bound_min);
  boundIndex(md_.local_bound_max);
}

void LocalOccupancyMap::clearAndInflateLocalMap() {
  Eigen::Vector3d local_min = md_.camera_pos - mp_.local_update_range;
  Eigen::Vector3d local_max = md_.camera_pos + mp_.local_update_range;
  Eigen::Vector3i keep_min, keep_max;
  posToIndex(local_min, keep_min);
  posToIndex(local_max, keep_max);
  boundIndex(keep_min);
  boundIndex(keep_max);

  for (int x = md_.local_bound_min(0); x <= md_.local_bound_max(0); ++x)
    for (int y = md_.local_bound_min(1); y <= md_.local_bound_max(1); ++y)
      for (int z = md_.local_bound_min(2); z <= md_.local_bound_max(2); ++z) {
        const bool in_local = x >= keep_min(0) && x <= keep_max(0) && y >= keep_min(1) &&
                              y <= keep_max(1) && z >= keep_min(2) && z <= keep_max(2);
        if (!in_local) {
          md_.occupancy_buffer[toAddress(x, y, z)] = mp_.clamp_min_log - mp_.unknown_flag;
        }
      }

  Eigen::Vector3i inf_min = md_.local_bound_min -
                            Eigen::Vector3i(mp_.local_map_margin, mp_.local_map_margin,
                                            mp_.local_map_margin);
  Eigen::Vector3i inf_max = md_.local_bound_max +
                            Eigen::Vector3i(mp_.local_map_margin, mp_.local_map_margin,
                                            mp_.local_map_margin);
  boundIndex(inf_min);
  boundIndex(inf_max);
  resetInflationBuffer(inf_min, inf_max);

  const int inf_step = std::ceil(mp_.obstacles_inflation / mp_.resolution);
  std::vector<Eigen::Vector3i> inf_pts;
  for (int x = md_.local_bound_min(0); x <= md_.local_bound_max(0); ++x)
    for (int y = md_.local_bound_min(1); y <= md_.local_bound_max(1); ++y)
      for (int z = md_.local_bound_min(2); z <= md_.local_bound_max(2); ++z) {
        if (md_.occupancy_buffer[toAddress(x, y, z)] <= mp_.min_occupancy_log) continue;
        inflatePoint(Eigen::Vector3i(x, y, z), inf_step, inf_pts);
        for (const auto& inf_pt : inf_pts) {
          if (isInMap(inf_pt)) md_.occupancy_buffer_inflate[toAddress(inf_pt)] = 1;
        }
      }
}

void LocalOccupancyMap::publishMap() {
  if (map_pub_.getNumSubscribers() <= 0) return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_id = md_.local_bound_min;
  Eigen::Vector3i max_id = md_.local_bound_max;
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

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = ros::Time::now();
  map_pub_.publish(msg);
}

void LocalOccupancyMap::publishInflatedMap() {
  if (map_inf_pub_.getNumSubscribers() <= 0) return;

  pcl::PointCloud<pcl::PointXYZ> cloud;
  Eigen::Vector3i min_id = md_.local_bound_min -
                           Eigen::Vector3i(mp_.local_map_margin, mp_.local_map_margin,
                                           mp_.local_map_margin);
  Eigen::Vector3i max_id = md_.local_bound_max +
                           Eigen::Vector3i(mp_.local_map_margin, mp_.local_map_margin,
                                           mp_.local_map_margin);
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

  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.header.frame_id = mp_.frame_id;

  sensor_msgs::PointCloud2 msg;
  pcl::toROSMsg(cloud, msg);
  msg.header.stamp = ros::Time::now();
  map_inf_pub_.publish(msg);
}

bool LocalOccupancyMap::isInMap(const Eigen::Vector3d& pos) const {
  return pos(0) >= mp_.map_min_boundary(0) && pos(1) >= mp_.map_min_boundary(1) &&
         pos(2) >= mp_.map_min_boundary(2) && pos(0) < mp_.map_max_boundary(0) &&
         pos(1) < mp_.map_max_boundary(1) && pos(2) < mp_.map_max_boundary(2);
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

Eigen::Vector3d LocalOccupancyMap::closestPointInMap(const Eigen::Vector3d& pt,
                                                     const Eigen::Vector3d& camera_pt) const {
  Eigen::Vector3d diff = pt - camera_pt;
  Eigen::Vector3d max_tc = mp_.map_max_boundary - camera_pt;
  Eigen::Vector3d min_tc = mp_.map_min_boundary - camera_pt;
  double min_t = 1.0;

  for (int i = 0; i < 3; ++i) {
    if (std::fabs(diff(i)) < 1e-6) continue;
    const double t1 = max_tc(i) / diff(i);
    const double t2 = min_tc(i) / diff(i);
    if (t1 > 0.0) min_t = std::min(min_t, t1);
    if (t2 > 0.0) min_t = std::min(min_t, t2);
  }

  return camera_pt + std::max(0.0, min_t - 1e-3) * diff;
}

void LocalOccupancyMap::resetInflationBuffer(const Eigen::Vector3i& min_id,
                                             const Eigen::Vector3i& max_id) {
  for (int x = min_id(0); x <= max_id(0); ++x)
    for (int y = min_id(1); y <= max_id(1); ++y)
      for (int z = min_id(2); z <= max_id(2); ++z)
        md_.occupancy_buffer_inflate[toAddress(x, y, z)] = 0;
}

void LocalOccupancyMap::inflatePoint(const Eigen::Vector3i& pt, int step,
                                     std::vector<Eigen::Vector3i>& pts) const {
  pts.clear();
  pts.reserve(std::pow(2 * step + 1, 3));
  for (int x = -step; x <= step; ++x)
    for (int y = -step; y <= step; ++y)
      for (int z = -step; z <= step; ++z)
        pts.emplace_back(pt + Eigen::Vector3i(x, y, z));
}

double LocalOccupancyMap::logit(double p) const {
  return std::log(p / (1.0 - p));
}

void LocalOccupancyMap::setOccupancyLogOdds(const Eigen::Vector3i& id, double update) {
  if (!isInMap(id)) return;
  const int addr = toAddress(id);
  md_.occupancy_buffer[addr] =
      std::min(std::max(md_.occupancy_buffer[addr] + update, mp_.clamp_min_log),
               mp_.clamp_max_log);
}

}  // namespace local_occupancy_map
