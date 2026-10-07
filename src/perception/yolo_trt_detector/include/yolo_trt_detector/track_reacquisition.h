#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <deque>
#include <stdexcept>

namespace yolo_trt_detector {

enum class ReacquisitionDecision {
  kRejected,
  kTentative,
  kConfirmedNew,
  kReacquired,
};

struct TrackReacquisitionConfig {
  double init_confidence = 0.45;
  double update_confidence = 0.25;
  int confirm_hits = 3;
  int confirm_window = 5;
  double init_gate_distance = 1.2;
  double identity_memory_seconds = 2.0;
  double reacquire_distance = 1.5;
  double max_measurement_jump = 1.0;
};

class TrackReacquisitionPolicy {
 public:
  using Position = std::array<double, 3>;

  explicit TrackReacquisitionPolicy(
      const TrackReacquisitionConfig& config = TrackReacquisitionConfig()) {
    configure(config);
  }

  void configure(const TrackReacquisitionConfig& config) {
    if (!(config.init_confidence >= 0.0) ||
        !(config.update_confidence >= 0.0) || config.confirm_hits <= 0 ||
        config.confirm_window < config.confirm_hits ||
        !(config.init_gate_distance > 0.0) ||
        !(config.identity_memory_seconds >= 0.0) ||
        !(config.reacquire_distance > 0.0) ||
        !(config.max_measurement_jump > 0.0)) {
      throw std::invalid_argument("invalid track reacquisition configuration");
    }
    config_ = config;
    clearTentative();
  }

  void rememberConfirmed(const Position& position, const Position& velocity,
                         double stamp_seconds) {
    if (!finite(position) || !finite(velocity) || !std::isfinite(stamp_seconds)) {
      return;
    }
    retained_position_ = position;
    retained_velocity_ = velocity;
    retained_stamp_seconds_ = stamp_seconds;
    retained_valid_ = true;
    lost_stamp_seconds_ = stamp_seconds;
    clearTentative();
  }

  void markLost(double stamp_seconds) {
    if (retained_valid_ && std::isfinite(stamp_seconds)) {
      lost_stamp_seconds_ = std::max(stamp_seconds, retained_stamp_seconds_);
    }
    clearTentative();
  }

  bool hasRetainedIdentity(double stamp_seconds) const {
    if (!retained_valid_ || !std::isfinite(stamp_seconds)) {
      return false;
    }
    const double age = stamp_seconds - lost_stamp_seconds_;
    return age >= 0.0 && age <= config_.identity_memory_seconds;
  }

  double reacquisitionDistance(const Position& candidate,
                               double stamp_seconds) const {
    if (!hasRetainedIdentity(stamp_seconds) || !finite(candidate)) {
      return INFINITY;
    }
    return distance(candidate, predictedRetainedPosition(stamp_seconds));
  }

  ReacquisitionDecision observeCandidate(const Position& candidate,
                                         double confidence,
                                         double stamp_seconds) {
    if (!finite(candidate) || !std::isfinite(confidence) ||
        !std::isfinite(stamp_seconds)) {
      recordHit(false);
      return ReacquisitionDecision::kRejected;
    }

    if (hasRetainedIdentity(stamp_seconds)) {
      if (confidence < config_.update_confidence) {
        return ReacquisitionDecision::kRejected;
      }
      const double innovation = reacquisitionDistance(candidate, stamp_seconds);
      if (innovation > config_.reacquire_distance ||
          innovation > config_.max_measurement_jump) {
        return ReacquisitionDecision::kRejected;
      }
      retained_position_ = candidate;
      retained_stamp_seconds_ = stamp_seconds;
      retained_valid_ = true;
      clearTentative();
      return ReacquisitionDecision::kReacquired;
    }

    if (retained_valid_ && stamp_seconds - lost_stamp_seconds_ >
                               config_.identity_memory_seconds) {
      retained_valid_ = false;
      clearTentative();
    }

    if (confidence < config_.init_confidence) {
      recordHit(false);
      return ReacquisitionDecision::kRejected;
    }

    if (tentative_anchor_valid_ &&
        distance(candidate, tentative_anchor_) > config_.init_gate_distance) {
      clearTentative();
    }
    tentative_anchor_ = candidate;
    tentative_anchor_valid_ = true;
    recordHit(true);
    if (recentHits() >= config_.confirm_hits) {
      rememberConfirmed(candidate, Position{{0.0, 0.0, 0.0}}, stamp_seconds);
      return ReacquisitionDecision::kConfirmedNew;
    }
    return ReacquisitionDecision::kTentative;
  }

  void observeMiss() { recordHit(false); }

  void clearTentative() {
    confirmation_hits_.clear();
    tentative_anchor_valid_ = false;
  }

  const Position& retainedVelocity() const { return retained_velocity_; }

 private:
  static bool finite(const Position& value) {
    return std::isfinite(value[0]) && std::isfinite(value[1]) &&
           std::isfinite(value[2]);
  }

  static double distance(const Position& lhs, const Position& rhs) {
    const double dx = lhs[0] - rhs[0];
    const double dy = lhs[1] - rhs[1];
    const double dz = lhs[2] - rhs[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
  }

  Position predictedRetainedPosition(double stamp_seconds) const {
    const double dt = std::max(0.0, stamp_seconds - retained_stamp_seconds_);
    return Position{{retained_position_[0] + retained_velocity_[0] * dt,
                     retained_position_[1] + retained_velocity_[1] * dt,
                     retained_position_[2] + retained_velocity_[2] * dt}};
  }

  void recordHit(bool hit) {
    confirmation_hits_.push_back(hit);
    while (static_cast<int>(confirmation_hits_.size()) > config_.confirm_window) {
      confirmation_hits_.pop_front();
    }
  }

  int recentHits() const {
    return static_cast<int>(
        std::count(confirmation_hits_.begin(), confirmation_hits_.end(), true));
  }

  TrackReacquisitionConfig config_;
  bool retained_valid_ = false;
  Position retained_position_{{0.0, 0.0, 0.0}};
  Position retained_velocity_{{0.0, 0.0, 0.0}};
  double retained_stamp_seconds_ = 0.0;
  double lost_stamp_seconds_ = 0.0;
  bool tentative_anchor_valid_ = false;
  Position tentative_anchor_{{0.0, 0.0, 0.0}};
  std::deque<bool> confirmation_hits_;
};

}  // namespace yolo_trt_detector
