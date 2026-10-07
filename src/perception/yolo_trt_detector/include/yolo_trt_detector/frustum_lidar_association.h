#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <queue>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>

#include <yolo_trt_detector/lidar_target_tracker.h>

namespace yolo_trt_detector {

struct ProjectedLidarPoint {
  cv::Point3d position;
  cv::Point2d pixel;
  double optical_depth = 0.0;
};

struct FrustumLidarCluster {
  LidarCluster cluster;
  cv::Point2d pixel_centroid;
  double optical_depth = 0.0;
  double normalized_center_distance = 0.0;
  double rank = 0.0;
};

struct FrustumLidarConfig {
  double cluster_radius = 0.14;
  int min_points = 3;
  int max_points = 800;
  double max_extent = 0.80;
  double max_diagonal = 1.00;
  double max_normalized_center_distance = 0.55;
  double minimum_rank_separation = 0.15;
  double center_rank_weight = 0.25;
};

namespace detail {

struct FrustumVoxelKey {
  int x = 0;
  int y = 0;
  int z = 0;
  bool operator==(const FrustumVoxelKey& other) const {
    return x == other.x && y == other.y && z == other.z;
  }
};

struct FrustumVoxelHash {
  size_t operator()(const FrustumVoxelKey& key) const {
    const uint64_t x = static_cast<uint32_t>(key.x);
    const uint64_t y = static_cast<uint32_t>(key.y);
    const uint64_t z = static_cast<uint32_t>(key.z);
    return static_cast<size_t>((x * 73856093ULL) ^ (y * 19349663ULL) ^
                               (z * 83492791ULL));
  }
};

inline FrustumVoxelKey frustumVoxel(const cv::Point3d& point, double size) {
  return {static_cast<int>(std::floor(point.x / size)),
          static_cast<int>(std::floor(point.y / size)),
          static_cast<int>(std::floor(point.z / size))};
}

}  // namespace detail

// Cluster only points whose projections lie inside the image detection. This
// prevents a wall or desk elsewhere in the cloud from becoming an identity
// candidate merely because its centroid happens to be near the tracked state.
inline std::vector<FrustumLidarCluster> clusterPointsInsideDetection(
    const std::vector<ProjectedLidarPoint>& projected_points,
    const cv::Rect& detection,
    const FrustumLidarConfig& input_config) {
  FrustumLidarConfig config = input_config;
  config.cluster_radius = std::max(0.03, config.cluster_radius);
  config.min_points = std::max(1, config.min_points);
  config.max_points = std::max(config.min_points, config.max_points);
  if (detection.width <= 1 || detection.height <= 1) return {};

  std::vector<ProjectedLidarPoint> points;
  points.reserve(projected_points.size());
  for (const auto& point : projected_points) {
    if (!std::isfinite(point.position.x) || !std::isfinite(point.position.y) ||
        !std::isfinite(point.position.z) || !std::isfinite(point.pixel.x) ||
        !std::isfinite(point.pixel.y) || !(point.optical_depth > 0.0)) continue;
    if (detection.contains(cv::Point(
            static_cast<int>(std::floor(point.pixel.x)),
            static_cast<int>(std::floor(point.pixel.y))))) {
      points.push_back(point);
    }
  }

  using Key = detail::FrustumVoxelKey;
  using Hash = detail::FrustumVoxelHash;
  std::unordered_map<Key, std::vector<size_t>, Hash> voxels;
  voxels.reserve(points.size());
  for (size_t index = 0; index < points.size(); ++index) {
    voxels[detail::frustumVoxel(points[index].position, config.cluster_radius)]
        .push_back(index);
  }

  std::vector<bool> visited(points.size(), false);
  std::vector<FrustumLidarCluster> result;
  const double radius2 = config.cluster_radius * config.cluster_radius;
  for (size_t seed = 0; seed < points.size(); ++seed) {
    if (visited[seed]) continue;
    std::queue<size_t> queue;
    std::vector<size_t> component;
    queue.push(seed);
    visited[seed] = true;
    while (!queue.empty()) {
      const size_t index = queue.front();
      queue.pop();
      component.push_back(index);
      const Key key = detail::frustumVoxel(
          points[index].position, config.cluster_radius);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            const auto found = voxels.find({key.x + dx, key.y + dy, key.z + dz});
            if (found == voxels.end()) continue;
            for (const size_t neighbor : found->second) {
              if (visited[neighbor]) continue;
              const cv::Point3d delta =
                  points[neighbor].position - points[index].position;
              if (delta.dot(delta) <= radius2) {
                visited[neighbor] = true;
                queue.push(neighbor);
              }
            }
          }
        }
      }
    }
    if (component.size() < static_cast<size_t>(config.min_points) ||
        component.size() > static_cast<size_t>(config.max_points)) continue;

    cv::Point3d sum(0.0, 0.0, 0.0);
    cv::Point3d sum_square(0.0, 0.0, 0.0);
    cv::Point3d minimum(1e9, 1e9, 1e9);
    cv::Point3d maximum(-1e9, -1e9, -1e9);
    cv::Point2d pixel_sum(0.0, 0.0);
    double depth_sum = 0.0;
    for (const size_t index : component) {
      const auto& point = points[index];
      sum += point.position;
      sum_square.x += point.position.x * point.position.x;
      sum_square.y += point.position.y * point.position.y;
      sum_square.z += point.position.z * point.position.z;
      minimum.x = std::min(minimum.x, point.position.x);
      minimum.y = std::min(minimum.y, point.position.y);
      minimum.z = std::min(minimum.z, point.position.z);
      maximum.x = std::max(maximum.x, point.position.x);
      maximum.y = std::max(maximum.y, point.position.y);
      maximum.z = std::max(maximum.z, point.position.z);
      pixel_sum += point.pixel;
      depth_sum += point.optical_depth;
    }
    const cv::Point3d extent = maximum - minimum;
    if (extent.x > config.max_extent || extent.y > config.max_extent ||
        extent.z > config.max_extent ||
        std::sqrt(extent.dot(extent)) > config.max_diagonal) continue;

    const double inverse_count = 1.0 / component.size();
    FrustumLidarCluster candidate;
    candidate.cluster.centroid = sum * inverse_count;
    candidate.cluster.minimum = minimum;
    candidate.cluster.maximum = maximum;
    candidate.cluster.point_count = static_cast<int>(component.size());
    candidate.cluster.position_variance = cv::Vec3d(
        std::max(1e-4, sum_square.x * inverse_count -
                           candidate.cluster.centroid.x * candidate.cluster.centroid.x),
        std::max(1e-4, sum_square.y * inverse_count -
                           candidate.cluster.centroid.y * candidate.cluster.centroid.y),
        std::max(1e-4, sum_square.z * inverse_count -
                           candidate.cluster.centroid.z * candidate.cluster.centroid.z));
    candidate.pixel_centroid = pixel_sum * inverse_count;
    candidate.optical_depth = depth_sum * inverse_count;
    const double nx = (candidate.pixel_centroid.x -
                       (detection.x + 0.5 * detection.width)) /
                      std::max(1, detection.width);
    const double ny = (candidate.pixel_centroid.y -
                       (detection.y + 0.5 * detection.height)) /
                      std::max(1, detection.height);
    candidate.normalized_center_distance = std::sqrt(nx * nx + ny * ny);
    candidate.rank = candidate.optical_depth +
        config.center_rank_weight * candidate.normalized_center_distance -
        0.01 * std::log1p(static_cast<double>(candidate.cluster.point_count));
    result.push_back(candidate);
  }
  return result;
}

// Prefer the nearest compact depth layer inside the visual detection. Refuse
// to guess when two layers have nearly equal rank.
inline int selectForegroundFrustumCluster(
    const std::vector<FrustumLidarCluster>& candidates,
    const cv::Rect& detection,
    const FrustumLidarConfig& config) {
  (void)detection;
  if (candidates.empty()) return -1;
  std::vector<size_t> order(candidates.size());
  for (size_t index = 0; index < order.size(); ++index) order[index] = index;
  std::sort(order.begin(), order.end(), [&](size_t lhs, size_t rhs) {
    return candidates[lhs].rank < candidates[rhs].rank;
  });
  const auto& best = candidates[order.front()];
  if (best.normalized_center_distance >
      config.max_normalized_center_distance) return -1;
  if (order.size() > 1 &&
      candidates[order[1]].rank - best.rank < config.minimum_rank_separation) {
    return -1;
  }
  return static_cast<int>(order.front());
}

}  // namespace yolo_trt_detector
