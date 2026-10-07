#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <unordered_map>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/video/tracking.hpp>

namespace yolo_trt_detector {

struct LidarPoint {
  double x = 0.0;
  double y = 0.0;
  double z = 0.0;
  float intensity = 0.0f;
};

struct LidarCluster {
  cv::Point3d centroid;
  cv::Point3d minimum;
  cv::Point3d maximum;
  cv::Vec3d position_variance{0.01, 0.01, 0.01};
  int point_count = 0;
};

enum class LidarTrackState { kLost, kTentative, kConfirmed, kCoasting };

struct LidarTrackEstimate {
  LidarTrackState state = LidarTrackState::kLost;
  cv::Point3d position;
  cv::Point3d velocity;
  cv::Vec3d position_variance{1.0, 1.0, 1.0};
  cv::Vec3d velocity_variance{1.0, 1.0, 1.0};
  std::array<double, 2> model_probability{{0.8, 0.2}};
  double stamp = 0.0;
  double last_lidar_update_age = 0.0;
  double last_visual_confirmation_age = 0.0;
  bool identity_confirmed = false;
  bool valid = false;
};

struct LidarTrackerConfig {
  double min_range = 0.45;
  double max_range = 18.0;
  double self_exclusion_radius = 0.65;
  double background_voxel_size = 0.18;
  int background_confirm_frames = 8;
  double background_forget_seconds = 3.0;
  double cluster_voxel_size = 0.22;
  int cluster_min_points = 2;
  int cluster_max_points = 1200;
  double cluster_max_extent = 1.20;
  double cluster_max_diagonal = 1.60;
  double track_gate_distance = 1.00;
  double track_gate_max_distance = 0.50;
  double association_min_score_separation = 0.12;
  double track_protection_radius = 0.75;
  double lidar_lost_time = 2.0;
  double covariance_lost_threshold = 2.25;
  double visual_confirmation_gate = 1.25;
  int visual_confirm_hits = 5;
  int visual_confirm_window = 7;
  double visual_confirm_max_gap = 0.25;
  double visual_tentative_gate = 0.60;
  double visual_tentative_max_speed = 2.0;
  double visual_acquire_max_range = 8.0;
  double visual_max_age = 1.0;
  bool require_recent_visual_confirmation = true;
  double max_target_speed = 4.0;
  double max_target_acceleration = 8.0;
  double measurement_position_slack = 0.25;
  double measurement_max_update_distance = 0.45;
  double measurement_velocity_slack = 0.8;
  double lidar_measurement_std = 0.10;
  double smooth_model_accel_std = 0.7;
  double maneuver_model_accel_std = 3.0;
  double initial_position_std = 0.20;
  double initial_velocity_std = 1.0;
  bool enforce_range_point_count = false;
  double target_physical_size = 0.55;
  double point_density_at_1m = 300.0;
  double point_count_min_ratio = 0.15;
  double point_count_max_ratio = 5.0;
};

// Static-world background subtraction + voxel connected components + a
// two-model interacting multiple-model (IMM) position tracker.  YOLO does not
// supply range to this class: it only confirms which LiDAR cluster has the
// requested identity.  After that confirmation, LiDAR updates continue even
// when the image detector is absent.
class LidarTargetTracker {
 public:
  using CandidateFilter = std::function<bool(const LidarCluster&)>;
  explicit LidarTargetTracker(const LidarTrackerConfig& config = {});

  void configure(const LidarTrackerConfig& config);
  void reset();

  LidarTrackEstimate processCloud(
      const std::vector<LidarPoint>& points,
      const cv::Point3d& sensor_position,
      double stamp,
      const CandidateFilter& candidate_filter = CandidateFilter());

  bool confirmIdentity(
      const LidarCluster& cluster, double visual_score, double stamp);
  void noteVisualFrameWithoutIdentity(double stamp);

  const std::vector<LidarCluster>& candidates() const { return candidates_; }
  double candidateStamp() const { return candidate_stamp_; }
  LidarTrackEstimate estimate(double stamp) const;

 private:
  struct VoxelKey {
    int x = 0;
    int y = 0;
    int z = 0;
    bool operator==(const VoxelKey& other) const {
      return x == other.x && y == other.y && z == other.z;
    }
  };
  struct VoxelHash {
    size_t operator()(const VoxelKey& key) const;
  };
  struct BackgroundCell {
    int consecutive_hits = 0;
    uint64_t last_frame = 0;
    double last_seen = 0.0;
  };
  struct Accumulator {
    cv::Point3d sum{0.0, 0.0, 0.0};
    cv::Point3d minimum{1e9, 1e9, 1e9};
    cv::Point3d maximum{-1e9, -1e9, -1e9};
    cv::Point3d sum_square{0.0, 0.0, 0.0};
    int count = 0;
  };

  VoxelKey voxel(const cv::Point3d& point, double size) const;
  std::vector<LidarCluster> extractCandidates(
      const std::vector<LidarPoint>& points,
      const cv::Point3d& sensor_position,
      double stamp);
  bool clusterPointCountMatchesRange(
      const LidarCluster& cluster,
      const cv::Point3d& sensor_position) const;
  void updateBackground(
      const std::unordered_map<VoxelKey, Accumulator, VoxelHash>& occupied,
      const cv::Point3d& protected_center,
      bool has_protected_center,
      double stamp);

  void initializeModels(const LidarCluster& cluster, double stamp);
  void immPredict(double stamp);
  void immCorrect(const LidarCluster& cluster);
  void fuseModels();
  int associateCandidate(const CandidateFilter& candidate_filter) const;
  bool candidateMotionIsPlausible(const LidarCluster& candidate) const;
  void clearIdentity();
  void clearTentativeIdentity();
  void trimVisualWindow();
  static double gaussianLikelihood(
      const cv::Mat& innovation, const cv::Mat& covariance);

  struct VisualConfirmation {
    cv::Point3d centroid;
    double stamp = 0.0;
  };

  LidarTrackerConfig config_;
  std::unordered_map<VoxelKey, BackgroundCell, VoxelHash> background_;
  uint64_t frame_index_ = 0;
  std::vector<LidarCluster> candidates_;
  double candidate_stamp_ = 0.0;

  std::array<cv::KalmanFilter, 2> models_;
  std::array<double, 2> probability_{{0.8, 0.2}};
  cv::Mat fused_state_;
  cv::Mat fused_covariance_;
  bool initialized_ = false;
  bool identity_confirmed_ = false;
  std::deque<VisualConfirmation> visual_confirmations_;
  std::deque<bool> visual_match_window_;
  double last_filter_stamp_ = 0.0;
  double last_lidar_update_stamp_ = 0.0;
  double last_visual_confirmation_stamp_ = 0.0;
  double last_visual_score_ = 0.0;
  cv::Point3d latest_sensor_position_;
  cv::Point3d last_measurement_position_;
  cv::Point3d last_measurement_velocity_;
  double last_measurement_stamp_ = 0.0;
  int accepted_measurement_count_ = 0;
};

}  // namespace yolo_trt_detector
