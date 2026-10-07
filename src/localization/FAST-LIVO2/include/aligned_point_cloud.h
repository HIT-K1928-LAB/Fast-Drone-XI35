#ifndef FAST_LIVO_ALIGNED_POINT_CLOUD_H
#define FAST_LIVO_ALIGNED_POINT_CLOUD_H

#include <Eigen/Core>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud2_iterator.h>

#include <string>

namespace fast_livo
{

// Keep every non-XYZ PointCloud2 field (including RGB/intensity) unchanged.
// The input points are already in FAST-LIVO's raw world frame, not the LiDAR
// body frame. Apply the same world alignment used by the FCU-body odometry.
inline bool alignWorldPointCloudInPlace(
    sensor_msgs::PointCloud2 &cloud,
    const Eigen::Matrix3d &rotation_aligned_world,
    const Eigen::Vector3d &initial_body_position_world)
{
  for (const char *name : {"x", "y", "z"})
  {
    bool found = false;
    for (const auto &field : cloud.fields)
    {
      if (field.name == name &&
          field.datatype == sensor_msgs::PointField::FLOAT32 && field.count == 1)
      {
        found = true;
        break;
      }
    }
    if (!found) return false;
  }

  sensor_msgs::PointCloud2Iterator<float> x(cloud, "x");
  sensor_msgs::PointCloud2Iterator<float> y(cloud, "y");
  sensor_msgs::PointCloud2Iterator<float> z(cloud, "z");
  for (; x != x.end(); ++x, ++y, ++z)
  {
    const Eigen::Vector3d raw(*x, *y, *z);
    if (!raw.allFinite()) continue;
    const Eigen::Vector3d aligned =
        rotation_aligned_world * (raw - initial_body_position_world);
    *x = static_cast<float>(aligned.x());
    *y = static_cast<float>(aligned.y());
    *z = static_cast<float>(aligned.z());
  }
  return true;
}

}  // namespace fast_livo

#endif
