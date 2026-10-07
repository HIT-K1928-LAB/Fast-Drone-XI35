#include <yolo_trt_detector/lidar_target_tracker.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <unordered_set>

namespace yolo_trt_detector {
namespace {

double squaredDistance(const cv::Point3d& a, const cv::Point3d& b) {
  const double dx = a.x - b.x;
  const double dy = a.y - b.y;
  const double dz = a.z - b.z;
  return dx * dx + dy * dy + dz * dz;
}

cv::Mat measurementMatrix() {
  cv::Mat matrix = cv::Mat::zeros(3, 6, CV_64F);
  for (int axis = 0; axis < 3; ++axis) matrix.at<double>(axis, axis) = 1.0;
  return matrix;
}

}  // namespace

size_t LidarTargetTracker::VoxelHash::operator()(const VoxelKey& key) const {
  const uint64_t x = static_cast<uint32_t>(key.x);
  const uint64_t y = static_cast<uint32_t>(key.y);
  const uint64_t z = static_cast<uint32_t>(key.z);
  return static_cast<size_t>((x * 73856093ULL) ^ (y * 19349663ULL) ^
                             (z * 83492791ULL));
}

LidarTargetTracker::LidarTargetTracker(const LidarTrackerConfig& config) {
  configure(config);
}

void LidarTargetTracker::configure(const LidarTrackerConfig& config) {
  config_ = config;
  config_.background_voxel_size = std::max(0.05, config_.background_voxel_size);
  config_.cluster_voxel_size = std::max(0.05, config_.cluster_voxel_size);
  config_.background_confirm_frames = std::max(2, config_.background_confirm_frames);
  config_.cluster_min_points = std::max(1, config_.cluster_min_points);
  config_.cluster_max_points = std::max(config_.cluster_min_points, config_.cluster_max_points);
  config_.visual_confirm_hits = std::max(1, config_.visual_confirm_hits);
  config_.visual_confirm_window = std::max(
      config_.visual_confirm_hits, config_.visual_confirm_window);
  config_.visual_confirm_max_gap = std::max(0.01, config_.visual_confirm_max_gap);
  config_.visual_tentative_gate = std::max(0.05, config_.visual_tentative_gate);
  config_.visual_tentative_max_speed =
      std::max(0.10, config_.visual_tentative_max_speed);
  config_.visual_acquire_max_range = std::max(
      config_.min_range, std::min(config_.max_range, config_.visual_acquire_max_range));
  config_.visual_max_age = std::max(0.10, config_.visual_max_age);
  config_.max_target_speed = std::max(0.10, config_.max_target_speed);
  config_.max_target_acceleration = std::max(0.10, config_.max_target_acceleration);
  config_.measurement_position_slack = std::max(0.0, config_.measurement_position_slack);
  config_.measurement_max_update_distance =
      std::max(0.05, config_.measurement_max_update_distance);
  config_.measurement_velocity_slack = std::max(0.0, config_.measurement_velocity_slack);
  config_.track_gate_max_distance =
      std::max(0.05, config_.track_gate_max_distance);
  config_.association_min_score_separation =
      std::max(0.0, config_.association_min_score_separation);
  config_.target_physical_size = std::max(0.05, config_.target_physical_size);
  config_.point_density_at_1m = std::max(1.0, config_.point_density_at_1m);
  config_.point_count_min_ratio =
      std::max(0.0, config_.point_count_min_ratio);
  config_.point_count_max_ratio = std::max(
      config_.point_count_min_ratio, config_.point_count_max_ratio);
  reset();
}

void LidarTargetTracker::reset() {
  background_.clear();
  candidates_.clear();
  candidate_stamp_ = 0.0;
  frame_index_ = 0;
  initialized_ = false;
  identity_confirmed_ = false;
  visual_confirmations_.clear();
  visual_match_window_.clear();
  last_filter_stamp_ = 0.0;
  last_lidar_update_stamp_ = 0.0;
  last_visual_confirmation_stamp_ = 0.0;
  last_visual_score_ = 0.0;
  latest_sensor_position_ = cv::Point3d();
  last_measurement_position_ = cv::Point3d();
  last_measurement_velocity_ = cv::Point3d();
  last_measurement_stamp_ = 0.0;
  accepted_measurement_count_ = 0;
  probability_ = {{0.8, 0.2}};
  fused_state_ = cv::Mat::zeros(6, 1, CV_64F);
  fused_covariance_ = cv::Mat::eye(6, 6, CV_64F);
}

LidarTargetTracker::VoxelKey LidarTargetTracker::voxel(
    const cv::Point3d& point, double size) const {
  return {static_cast<int>(std::floor(point.x / size)),
          static_cast<int>(std::floor(point.y / size)),
          static_cast<int>(std::floor(point.z / size))};
}

bool LidarTargetTracker::clusterPointCountMatchesRange(
    const LidarCluster& cluster,
    const cv::Point3d& sensor_position) const {
  if (!config_.enforce_range_point_count) return true;
  const double range2 = std::max(
      config_.min_range * config_.min_range,
      squaredDistance(cluster.centroid, sensor_position));
  // For a target with fixed projected area and approximately uniform angular
  // sampling, the expected return count scales with area / range^2.  The wide
  // ratio envelope accounts for attitude, occlusion and reflectivity while
  // rejecting isolated noise and merged walls/people of the wrong scale.
  const double expected = config_.point_density_at_1m *
      config_.target_physical_size * config_.target_physical_size / range2;
  const int minimum = std::max(
      config_.cluster_min_points,
      static_cast<int>(std::floor(expected * config_.point_count_min_ratio)));
  const int maximum = std::max(
      minimum,
      static_cast<int>(std::ceil(expected * config_.point_count_max_ratio)));
  return cluster.point_count >= minimum && cluster.point_count <= maximum;
}

std::vector<LidarCluster> LidarTargetTracker::extractCandidates(
    const std::vector<LidarPoint>& points,
    const cv::Point3d& sensor_position,
    double stamp) {
  ++frame_index_;
  const bool protect_track = initialized_ && identity_confirmed_;
  const cv::Point3d protected_center(
      fused_state_.at<double>(0), fused_state_.at<double>(1),
      fused_state_.at<double>(2));
  const double protection2 = config_.track_protection_radius *
                             config_.track_protection_radius;

  std::unordered_map<VoxelKey, Accumulator, VoxelHash> all_occupied;
  std::unordered_map<VoxelKey, Accumulator, VoxelHash> foreground;
  all_occupied.reserve(points.size() / 2 + 1);
  foreground.reserve(points.size() / 4 + 1);

  for (const LidarPoint& input : points) {
    if (!std::isfinite(input.x) || !std::isfinite(input.y) ||
        !std::isfinite(input.z)) continue;
    const cv::Point3d point(input.x, input.y, input.z);
    const double range2 = squaredDistance(point, sensor_position);
    if (range2 < config_.self_exclusion_radius * config_.self_exclusion_radius ||
        range2 < config_.min_range * config_.min_range ||
        range2 > config_.max_range * config_.max_range) continue;

    const VoxelKey background_key = voxel(point, config_.background_voxel_size);
    Accumulator& all = all_occupied[background_key];
    all.sum += point;
    ++all.count;

    const auto background_it = background_.find(background_key);
    const bool is_static = background_it != background_.end() &&
        background_it->second.consecutive_hits >= config_.background_confirm_frames;
    const bool is_protected = protect_track &&
        squaredDistance(point, protected_center) <= protection2;
    // Before YOLO has named a target, compact clusters must remain available
    // even when they have been stationary long enough to enter the background
    // model. Otherwise a hovering target would become impossible to acquire.
    // Once identity is known, normal static-background rejection is enabled.
    if (identity_confirmed_ && is_static && !is_protected) continue;

    const VoxelKey cluster_key = voxel(point, config_.cluster_voxel_size);
    Accumulator& cell = foreground[cluster_key];
    cell.sum += point;
    cell.sum_square.x += point.x * point.x;
    cell.sum_square.y += point.y * point.y;
    cell.sum_square.z += point.z * point.z;
    cell.minimum.x = std::min(cell.minimum.x, point.x);
    cell.minimum.y = std::min(cell.minimum.y, point.y);
    cell.minimum.z = std::min(cell.minimum.z, point.z);
    cell.maximum.x = std::max(cell.maximum.x, point.x);
    cell.maximum.y = std::max(cell.maximum.y, point.y);
    cell.maximum.z = std::max(cell.maximum.z, point.z);
    ++cell.count;
  }

  updateBackground(all_occupied, protected_center, protect_track, stamp);

  std::vector<LidarCluster> clusters;
  std::unordered_set<VoxelKey, VoxelHash> visited;
  visited.reserve(foreground.size());
  for (const auto& seed : foreground) {
    if (visited.count(seed.first)) continue;
    std::queue<VoxelKey> queue;
    queue.push(seed.first);
    visited.insert(seed.first);
    Accumulator merged;
    while (!queue.empty()) {
      const VoxelKey key = queue.front();
      queue.pop();
      const auto it = foreground.find(key);
      if (it == foreground.end()) continue;
      const Accumulator& cell = it->second;
      merged.sum += cell.sum;
      merged.sum_square += cell.sum_square;
      merged.count += cell.count;
      merged.minimum.x = std::min(merged.minimum.x, cell.minimum.x);
      merged.minimum.y = std::min(merged.minimum.y, cell.minimum.y);
      merged.minimum.z = std::min(merged.minimum.z, cell.minimum.z);
      merged.maximum.x = std::max(merged.maximum.x, cell.maximum.x);
      merged.maximum.y = std::max(merged.maximum.y, cell.maximum.y);
      merged.maximum.z = std::max(merged.maximum.z, cell.maximum.z);
      for (int dx = -1; dx <= 1; ++dx) {
        for (int dy = -1; dy <= 1; ++dy) {
          for (int dz = -1; dz <= 1; ++dz) {
            if (dx == 0 && dy == 0 && dz == 0) continue;
            const VoxelKey neighbor{key.x + dx, key.y + dy, key.z + dz};
            if (foreground.count(neighbor) && !visited.count(neighbor)) {
              visited.insert(neighbor);
              queue.push(neighbor);
            }
          }
        }
      }
    }
    if (merged.count < config_.cluster_min_points ||
        merged.count > config_.cluster_max_points) continue;
    const cv::Point3d extent = merged.maximum - merged.minimum;
    if (extent.x > config_.cluster_max_extent ||
        extent.y > config_.cluster_max_extent ||
        extent.z > config_.cluster_max_extent ||
        std::sqrt(extent.dot(extent)) > config_.cluster_max_diagonal) continue;
    LidarCluster cluster;
    cluster.point_count = merged.count;
    cluster.centroid = merged.sum * (1.0 / merged.count);
    cluster.minimum = merged.minimum;
    cluster.maximum = merged.maximum;
    const double floor_variance = config_.lidar_measurement_std *
                                  config_.lidar_measurement_std;
    cluster.position_variance = cv::Vec3d(
        std::max(floor_variance,
                 merged.sum_square.x / merged.count - cluster.centroid.x * cluster.centroid.x),
        std::max(floor_variance,
                 merged.sum_square.y / merged.count - cluster.centroid.y * cluster.centroid.y),
        std::max(floor_variance,
                 merged.sum_square.z / merged.count - cluster.centroid.z * cluster.centroid.z));
    if (!clusterPointCountMatchesRange(cluster, sensor_position)) continue;
    clusters.push_back(cluster);
  }
  return clusters;
}

void LidarTargetTracker::updateBackground(
    const std::unordered_map<VoxelKey, Accumulator, VoxelHash>& occupied,
    const cv::Point3d& protected_center,
    bool has_protected_center,
    double stamp) {
  const double protection2 = config_.track_protection_radius *
                             config_.track_protection_radius;
  for (const auto& item : occupied) {
    const Accumulator& points = item.second;
    const cv::Point3d center = points.sum * (1.0 / std::max(1, points.count));
    if (has_protected_center &&
        squaredDistance(center, protected_center) <= protection2) continue;
    BackgroundCell& cell = background_[item.first];
    cell.consecutive_hits = cell.last_frame + 1 == frame_index_
        ? std::min(config_.background_confirm_frames + 10,
                   cell.consecutive_hits + 1)
        : 1;
    cell.last_frame = frame_index_;
    cell.last_seen = stamp;
  }
  if (frame_index_ % 50 == 0) {
    for (auto it = background_.begin(); it != background_.end();) {
      if (stamp - it->second.last_seen > config_.background_forget_seconds) {
        it = background_.erase(it);
      } else {
        ++it;
      }
    }
  }
}

void LidarTargetTracker::initializeModels(
    const LidarCluster& cluster, double stamp) {
  const cv::Mat h = measurementMatrix();
  for (int model = 0; model < 2; ++model) {
    models_[model].init(6, 3, 0, CV_64F);
    models_[model].measurementMatrix = h.clone();
    models_[model].statePost = cv::Mat::zeros(6, 1, CV_64F);
    models_[model].statePost.at<double>(0) = cluster.centroid.x;
    models_[model].statePost.at<double>(1) = cluster.centroid.y;
    models_[model].statePost.at<double>(2) = cluster.centroid.z;
    models_[model].errorCovPost = cv::Mat::zeros(6, 6, CV_64F);
    for (int axis = 0; axis < 3; ++axis) {
      models_[model].errorCovPost.at<double>(axis, axis) =
          config_.initial_position_std * config_.initial_position_std;
      models_[model].errorCovPost.at<double>(axis + 3, axis + 3) =
          config_.initial_velocity_std * config_.initial_velocity_std;
    }
  }
  probability_ = {{0.8, 0.2}};
  initialized_ = true;
  last_filter_stamp_ = stamp;
  last_lidar_update_stamp_ = stamp;
  fuseModels();
}

void LidarTargetTracker::immPredict(double stamp) {
  double dt = stamp - last_filter_stamp_;
  if (!std::isfinite(dt) || dt <= 0.0) dt = 1e-3;
  dt = std::min(0.5, dt);
  static const double transition[2][2] = {{0.97, 0.03}, {0.08, 0.92}};
  std::array<double, 2> normalization{{0.0, 0.0}};
  for (int destination = 0; destination < 2; ++destination) {
    for (int source = 0; source < 2; ++source) {
      normalization[destination] += probability_[source] *
                                    transition[source][destination];
    }
  }
  std::array<cv::Mat, 2> mixed_state;
  std::array<cv::Mat, 2> mixed_covariance;
  for (int destination = 0; destination < 2; ++destination) {
    mixed_state[destination] = cv::Mat::zeros(6, 1, CV_64F);
    for (int source = 0; source < 2; ++source) {
      const double weight = probability_[source] * transition[source][destination] /
          std::max(1e-12, normalization[destination]);
      mixed_state[destination] += models_[source].statePost * weight;
    }
    mixed_covariance[destination] = cv::Mat::zeros(6, 6, CV_64F);
    for (int source = 0; source < 2; ++source) {
      const double weight = probability_[source] * transition[source][destination] /
          std::max(1e-12, normalization[destination]);
      const cv::Mat difference = models_[source].statePost - mixed_state[destination];
      mixed_covariance[destination] += weight *
          (models_[source].errorCovPost + difference * difference.t());
    }
  }
  probability_ = normalization;
  for (int model = 0; model < 2; ++model) {
    cv::KalmanFilter& filter = models_[model];
    filter.statePost = mixed_state[model];
    filter.errorCovPost = mixed_covariance[model];
    filter.transitionMatrix = cv::Mat::eye(6, 6, CV_64F);
    for (int axis = 0; axis < 3; ++axis) {
      filter.transitionMatrix.at<double>(axis, axis + 3) = dt;
    }
    filter.processNoiseCov = cv::Mat::zeros(6, 6, CV_64F);
    const double accel_std = model == 0 ? config_.smooth_model_accel_std
                                        : config_.maneuver_model_accel_std;
    const double q = accel_std * accel_std;
    for (int axis = 0; axis < 3; ++axis) {
      filter.processNoiseCov.at<double>(axis, axis) = 0.25 * dt * dt * dt * dt * q;
      filter.processNoiseCov.at<double>(axis, axis + 3) = 0.5 * dt * dt * dt * q;
      filter.processNoiseCov.at<double>(axis + 3, axis) = 0.5 * dt * dt * dt * q;
      filter.processNoiseCov.at<double>(axis + 3, axis + 3) = dt * dt * q;
    }
    filter.predict();
  }
  last_filter_stamp_ = stamp;
  // Keep statePre/errorCovPre intact for the subsequent Kalman correction,
  // while exposing the mixed predicted estimate for association.
  fused_state_ = cv::Mat::zeros(6, 1, CV_64F);
  for (int model = 0; model < 2; ++model) {
    fused_state_ += probability_[model] * models_[model].statePre;
  }
  fused_covariance_ = cv::Mat::zeros(6, 6, CV_64F);
  for (int model = 0; model < 2; ++model) {
    const cv::Mat difference = models_[model].statePre - fused_state_;
    fused_covariance_ += probability_[model] *
        (models_[model].errorCovPre + difference * difference.t());
  }
}

double LidarTargetTracker::gaussianLikelihood(
    const cv::Mat& innovation, const cv::Mat& covariance) {
  cv::Mat inverse;
  const double determinant = cv::determinant(covariance);
  if (!(determinant > 1e-18) ||
      !cv::invert(covariance, inverse, cv::DECOMP_SVD)) return 1e-12;
  const cv::Mat mahalanobis = innovation.t() * inverse * innovation;
  const double exponent = -0.5 * mahalanobis.at<double>(0, 0);
  constexpr double kTwoPi = 6.28318530717958647692;
  return std::max(1e-12, std::exp(std::max(-80.0, exponent)) /
      std::sqrt(kTwoPi * kTwoPi * kTwoPi * determinant));
}

void LidarTargetTracker::immCorrect(const LidarCluster& cluster) {
  const cv::Mat measurement = (cv::Mat_<double>(3, 1) <<
      cluster.centroid.x, cluster.centroid.y, cluster.centroid.z);
  std::array<double, 2> likelihood{{0.0, 0.0}};
  for (int model = 0; model < 2; ++model) {
    cv::KalmanFilter& filter = models_[model];
    filter.measurementNoiseCov = cv::Mat::zeros(3, 3, CV_64F);
    for (int axis = 0; axis < 3; ++axis) {
      filter.measurementNoiseCov.at<double>(axis, axis) =
          cluster.position_variance[axis];
    }
    const cv::Mat innovation = measurement -
        filter.measurementMatrix * filter.statePre;
    const cv::Mat innovation_covariance =
        filter.measurementMatrix * filter.errorCovPre *
        filter.measurementMatrix.t() + filter.measurementNoiseCov;
    likelihood[model] = gaussianLikelihood(innovation, innovation_covariance);
    filter.correct(measurement);
  }
  double total = probability_[0] * likelihood[0] +
                 probability_[1] * likelihood[1];
  if (!(total > 1e-18)) total = 1e-18;
  for (int model = 0; model < 2; ++model) {
    probability_[model] = probability_[model] * likelihood[model] / total;
  }
  fuseModels();
}

void LidarTargetTracker::fuseModels() {
  fused_state_ = cv::Mat::zeros(6, 1, CV_64F);
  for (int model = 0; model < 2; ++model) {
    const cv::Mat& state = models_[model].statePost.empty()
        ? models_[model].statePre : models_[model].statePost;
    if (!state.empty()) fused_state_ += probability_[model] * state;
  }
  fused_covariance_ = cv::Mat::zeros(6, 6, CV_64F);
  for (int model = 0; model < 2; ++model) {
    const cv::Mat& state = models_[model].statePost.empty()
        ? models_[model].statePre : models_[model].statePost;
    const cv::Mat& covariance = models_[model].errorCovPost.empty()
        ? models_[model].errorCovPre : models_[model].errorCovPost;
    if (state.empty() || covariance.empty()) continue;
    const cv::Mat difference = state - fused_state_;
    fused_covariance_ += probability_[model] *
        (covariance + difference * difference.t());
  }
}

int LidarTargetTracker::associateCandidate(
    const CandidateFilter& candidate_filter) const {
  if (!initialized_ || candidates_.empty()) return -1;
  const cv::Point3d predicted(
      fused_state_.at<double>(0), fused_state_.at<double>(1),
      fused_state_.at<double>(2));
  const double adaptive_gate = config_.track_gate_distance +
      2.0 * std::sqrt(std::max({fused_covariance_.at<double>(0, 0),
                                fused_covariance_.at<double>(1, 1),
                                fused_covariance_.at<double>(2, 2), 0.0}));
  int best = -1;
  double best_score = std::numeric_limits<double>::infinity();
  double second_best_score = std::numeric_limits<double>::infinity();
  for (size_t i = 0; i < candidates_.size(); ++i) {
    if (candidate_filter && !candidate_filter(candidates_[i])) continue;
    if (!candidateMotionIsPlausible(candidates_[i])) continue;
    const double distance = std::sqrt(squaredDistance(
        predicted, candidates_[i].centroid));
    if (distance > std::min(adaptive_gate, config_.track_gate_max_distance)) continue;
    const double sparse_penalty = 0.03 /
        std::sqrt(static_cast<double>(std::max(1, candidates_[i].point_count)));
    const double score = distance + sparse_penalty;
    if (score < best_score) {
      second_best_score = best_score;
      best_score = score;
      best = static_cast<int>(i);
    } else if (score < second_best_score) {
      second_best_score = score;
    }
  }
  if (best >= 0 && std::isfinite(second_best_score) &&
      second_best_score - best_score < config_.association_min_score_separation) {
    return -1;
  }
  return best;
}

bool LidarTargetTracker::candidateMotionIsPlausible(
    const LidarCluster& candidate) const {
  if (accepted_measurement_count_ <= 0 ||
      !(candidate_stamp_ > last_measurement_stamp_)) return true;
  const double dt = candidate_stamp_ - last_measurement_stamp_;
  const cv::Point3d delta = candidate.centroid - last_measurement_position_;
  const double displacement = std::sqrt(delta.dot(delta));
  if (displacement > config_.measurement_max_update_distance) return false;
  if (displacement > config_.measurement_position_slack +
          config_.max_target_speed * dt) return false;

  const cv::Point3d measured_velocity = delta * (1.0 / dt);
  if (accepted_measurement_count_ >= 2) {
    const cv::Point3d velocity_delta =
        measured_velocity - last_measurement_velocity_;
    const double velocity_change = std::sqrt(velocity_delta.dot(velocity_delta));
    if (velocity_change > config_.measurement_velocity_slack +
            config_.max_target_acceleration * dt) return false;
  }
  return true;
}

void LidarTargetTracker::clearIdentity() {
  identity_confirmed_ = false;
  initialized_ = false;
  clearTentativeIdentity();
  last_measurement_stamp_ = 0.0;
  accepted_measurement_count_ = 0;
  last_measurement_velocity_ = cv::Point3d();
}

void LidarTargetTracker::clearTentativeIdentity() {
  visual_confirmations_.clear();
  visual_match_window_.clear();
}

void LidarTargetTracker::trimVisualWindow() {
  while (visual_match_window_.size() >
         static_cast<size_t>(config_.visual_confirm_window)) {
    if (visual_match_window_.front() && !visual_confirmations_.empty()) {
      visual_confirmations_.pop_front();
    }
    visual_match_window_.pop_front();
  }
}

LidarTrackEstimate LidarTargetTracker::processCloud(
    const std::vector<LidarPoint>& points,
    const cv::Point3d& sensor_position,
    double stamp,
    const CandidateFilter& candidate_filter) {
  latest_sensor_position_ = sensor_position;
  if (initialized_) immPredict(stamp);
  candidates_ = extractCandidates(points, sensor_position, stamp);
  candidate_stamp_ = stamp;
  if (initialized_ && identity_confirmed_) {
    const int associated = associateCandidate(candidate_filter);
    if (associated >= 0) {
      const cv::Point3d previous_position = last_measurement_position_;
      const double previous_stamp = last_measurement_stamp_;
      immCorrect(candidates_[associated]);
      last_lidar_update_stamp_ = stamp;
      last_measurement_position_ = candidates_[associated].centroid;
      if (accepted_measurement_count_ > 0 && stamp > previous_stamp) {
        last_measurement_velocity_ =
            (last_measurement_position_ - previous_position) /
            (stamp - previous_stamp);
      }
      last_measurement_stamp_ = stamp;
      ++accepted_measurement_count_;
    } else {
      for (auto& model : models_) {
        model.statePost = model.statePre.clone();
        model.errorCovPost = model.errorCovPre.clone();
      }
      fuseModels();
    }
  }
  LidarTrackEstimate result = estimate(stamp);
  if (identity_confirmed_ && !result.valid) {
    // A covariance/age loss invalidates identity.  Keep the learned static
    // background, but require YOLO to select a cluster again; never let an
    // unbounded prediction gate silently switch to another object.
    clearIdentity();
  }
  return result;
}

bool LidarTargetTracker::confirmIdentity(
    const LidarCluster& cluster, double visual_score, double stamp) {
  if (!std::isfinite(stamp) || !std::isfinite(visual_score)) return false;
  if (!clusterPointCountMatchesRange(cluster, latest_sensor_position_)) {
    return false;
  }
  if (identity_confirmed_ && initialized_) {
    const cv::Point3d current(
        fused_state_.at<double>(0), fused_state_.at<double>(1),
        fused_state_.at<double>(2));
    if (std::sqrt(squaredDistance(current, cluster.centroid)) >
        config_.visual_confirmation_gate) return false;
    // YOLO is an identity observation, not a second position sensor.  Do not
    // correct the filter here: doing so would count the same LiDAR cluster
    // twice and could also apply an image timestamp older than the cloud.
    last_visual_confirmation_stamp_ = stamp;
    last_visual_score_ = visual_score;
    return true;
  }

  const double acquisition_range = std::sqrt(squaredDistance(
      cluster.centroid, latest_sensor_position_));
  if (acquisition_range > config_.visual_acquire_max_range) {
    noteVisualFrameWithoutIdentity(stamp);
    return false;
  }

  if (!visual_confirmations_.empty()) {
    const VisualConfirmation& previous = visual_confirmations_.back();
    const double gap = stamp - previous.stamp;
    const double movement = std::sqrt(squaredDistance(
        cluster.centroid, previous.centroid));
    if (!(gap > 0.0) || gap > config_.visual_confirm_max_gap ||
        movement > config_.visual_tentative_gate +
            config_.visual_tentative_max_speed * gap) {
      clearTentativeIdentity();
    }
  }
  visual_confirmations_.push_back({cluster.centroid, stamp});
  visual_match_window_.push_back(true);
  trimVisualWindow();
  const int hit_count = static_cast<int>(std::count(
      visual_match_window_.begin(), visual_match_window_.end(), true));
  if (hit_count <
      static_cast<size_t>(config_.visual_confirm_hits)) return false;

  initializeModels(cluster, stamp);
  identity_confirmed_ = true;
  clearTentativeIdentity();
  last_visual_confirmation_stamp_ = stamp;
  last_visual_score_ = visual_score;
  last_measurement_position_ = cluster.centroid;
  last_measurement_velocity_ = cv::Point3d();
  last_measurement_stamp_ = stamp;
  accepted_measurement_count_ = 1;
  return true;
}

void LidarTargetTracker::noteVisualFrameWithoutIdentity(double stamp) {
  if (identity_confirmed_ || !std::isfinite(stamp)) return;
  if (!visual_confirmations_.empty() &&
      stamp - visual_confirmations_.back().stamp >
          config_.visual_confirm_max_gap) {
    clearTentativeIdentity();
  }
  visual_match_window_.push_back(false);
  trimVisualWindow();
  if (std::find(visual_match_window_.begin(), visual_match_window_.end(), true) ==
      visual_match_window_.end()) {
    visual_confirmations_.clear();
  }
}

LidarTrackEstimate LidarTargetTracker::estimate(double stamp) const {
  LidarTrackEstimate result;
  result.stamp = stamp;
  result.identity_confirmed = identity_confirmed_;
  result.model_probability = probability_;
  if (!initialized_ || !identity_confirmed_ || fused_state_.empty()) {
    if (!visual_confirmations_.empty()) {
      result.state = LidarTrackState::kTentative;
    }
    return result;
  }
  result.position = cv::Point3d(
      fused_state_.at<double>(0), fused_state_.at<double>(1),
      fused_state_.at<double>(2));
  result.velocity = cv::Point3d(
      fused_state_.at<double>(3), fused_state_.at<double>(4),
      fused_state_.at<double>(5));
  for (int axis = 0; axis < 3; ++axis) {
    result.position_variance[axis] =
        std::max(0.0, fused_covariance_.at<double>(axis, axis));
    result.velocity_variance[axis] =
        std::max(0.0, fused_covariance_.at<double>(axis + 3, axis + 3));
  }
  result.last_lidar_update_age = std::max(0.0, stamp - last_lidar_update_stamp_);
  result.last_visual_confirmation_age =
      std::max(0.0, stamp - last_visual_confirmation_stamp_);
  const double largest_position_variance = std::max(
      result.position_variance[0],
      std::max(result.position_variance[1], result.position_variance[2]));
  const double speed = std::sqrt(
      result.velocity.x * result.velocity.x +
      result.velocity.y * result.velocity.y +
      result.velocity.z * result.velocity.z);
  const double sensor_range = std::sqrt(squaredDistance(
      result.position, latest_sensor_position_));
  const bool visual_confirmation_expired =
      config_.require_recent_visual_confirmation &&
      result.last_visual_confirmation_age > config_.visual_max_age;
  if (result.last_lidar_update_age > config_.lidar_lost_time ||
      visual_confirmation_expired ||
      largest_position_variance > config_.covariance_lost_threshold ||
      speed > config_.max_target_speed || sensor_range > config_.max_range) {
    result.state = LidarTrackState::kLost;
    result.valid = false;
  } else if (result.last_lidar_update_age > 0.15) {
    result.state = LidarTrackState::kCoasting;
    result.valid = true;
  } else {
    result.state = LidarTrackState::kConfirmed;
    result.valid = true;
  }
  return result;
}

}  // namespace yolo_trt_detector
