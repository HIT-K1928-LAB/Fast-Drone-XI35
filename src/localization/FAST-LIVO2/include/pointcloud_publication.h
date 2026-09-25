#pragma once

#include <sensor_msgs/PointCloud2.h>

namespace fast_livo
{

inline bool hasPublishablePoints(const sensor_msgs::PointCloud2 &cloud)
{
  return cloud.width > 0 && !cloud.data.empty();
}

}  // namespace fast_livo
