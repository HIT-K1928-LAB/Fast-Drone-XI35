#pragma once

#include <algorithm>
#include <cstddef>
#include <vector>

namespace yolo_trt_detector {

struct VisualLidarMatchCandidate {
  size_t detection_index = 0;
  size_t cluster_index = 0;
  double normalized_center_distance = 0.0;
  double rank = 0.0;
};

struct VisualLidarMatchConfig {
  double max_normalized_center_distance = 0.35;
  double minimum_rank_separation = 0.12;
  bool require_single_detection_on_acquire = true;
};

// Returns the index in candidates, or -1 when the image/LiDAR association is
// unsafe. Initial identity acquisition is deliberately stricter than an
// update: the camera must show one eligible object and the best projected
// cluster must be both centered and unambiguous.
inline int selectVisualLidarMatch(
    const std::vector<VisualLidarMatchCandidate>& candidates,
    size_t eligible_detection_count,
    bool identity_confirmed,
    const VisualLidarMatchConfig& config) {
  if (candidates.empty()) return -1;
  if (!identity_confirmed && config.require_single_detection_on_acquire &&
      eligible_detection_count != 1) return -1;

  std::vector<size_t> order(candidates.size());
  for (size_t index = 0; index < order.size(); ++index) order[index] = index;
  std::sort(order.begin(), order.end(), [&](size_t lhs, size_t rhs) {
    return candidates[lhs].rank < candidates[rhs].rank;
  });
  const VisualLidarMatchCandidate& best = candidates[order.front()];
  if (best.normalized_center_distance >
      config.max_normalized_center_distance) return -1;

  if (order.size() > 1) {
    const VisualLidarMatchCandidate& second = candidates[order[1]];
    if (second.cluster_index != best.cluster_index &&
        second.rank - best.rank < config.minimum_rank_separation) return -1;
  }
  return static_cast<int>(order.front());
}

}  // namespace yolo_trt_detector
