#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <string>

namespace yopo_minco_planner {
// No ROS, GPU or commands. All elapsed-time inputs use a monotonic clock.
class RecoveryGate {
 public:
  enum class Phase { TRACK, WAIT_HOVER, REPLAN, READY, STOPPED };
  double release_wait=1.0, stable_hold=1.0, speed_limit=0.10, position_band=0.10;
  double timeout=30.0, good_span=0.4, max_frame_gap=0.25, min_progress=0.05;
  int required_frames=5, max_attempts=3, max_no_progress=2;
  Phase phase=Phase::STOPPED;
  int attempts=0, good_frames=0;
  unsigned long revision=0;
  std::string failure;

  void start(double distance) {
    phase=Phase::TRACK; attempts=0; no_progress_=0; best_distance_=distance;
    first_failure_=-1; failure.clear(); reset();
  }
  bool recovering() const {
    return phase==Phase::WAIT_HOVER || phase==Phase::REPLAN || phase==Phase::READY;
  }
  void stop(const std::string& why) { phase=Phase::STOPPED; failure=why; reset(); }
  bool begin(double now, double distance) {
    if (phase!=Phase::TRACK) return false;
    if (attempts>=max_attempts) { stop("recovery attempt limit"); return false; }
    if (attempts>0) {
      no_progress_ = best_distance_-distance>=min_progress ? 0 : no_progress_+1;
      if (no_progress_>=max_no_progress) { stop("recovery made no sustained progress"); return false; }
    }
    best_distance_=std::min(best_distance_,distance);
    if (first_failure_<0) first_failure_=now;
    ++attempts; released_=now; phase=Phase::WAIT_HOVER; reset(); return true;
  }
  void update(double now, const std::array<double,3>& p, double speed) {
    if (!recovering()) return;
    if (now-first_failure_>=timeout) { stop("recovery total timeout"); return; }
    if (now-released_<release_wait || !std::isfinite(speed) || speed>speed_limit) {
      phase=Phase::WAIT_HOVER; reset(); return;
    }
    double d2=0; for (int i=0;i<3;++i) d2+=(p[i]-anchor_[i])*(p[i]-anchor_[i]);
    if (stable_since_<0 || d2>position_band*position_band) {
      phase=Phase::WAIT_HOVER; reset(); anchor_=p; stable_since_=now; return;
    }
    if (phase==Phase::WAIT_HOVER && now-stable_since_>=stable_hold) phase=Phase::REPLAN;
  }
  // Distinct source frames, bounded wall-time gap, and minimum observation span.
  void candidate(double now, double stamp, bool valid) {
    if (phase!=Phase::REPLAN && phase!=Phase::READY) return;
    if (!valid) { clearCandidates(); return; }
    if (stamp<=last_stamp_) return;
    if (last_good_>=0 && now-last_good_>max_frame_gap) clearCandidates();
    last_stamp_=stamp; last_good_=now;
    if (good_frames++==0) good_since_=now;
    if (good_frames>=required_frames && now-good_since_>=good_span) phase=Phase::READY;
  }
  bool freshReady(double now) const {
    return phase==Phase::READY && last_good_>=0 && now-last_good_<=max_frame_gap;
  }
  void resumed() { phase=Phase::TRACK; reset(); }
 private:
  int no_progress_=0;
  double best_distance_=0, first_failure_=-1, released_=0, stable_since_=-1;
  double good_since_=-1, last_good_=-1, last_stamp_=-1;
  std::array<double,3> anchor_{{0,0,0}};
  void clearCandidates() {
    good_frames=0; good_since_=-1; last_good_=-1;
    if (phase==Phase::READY) phase=Phase::REPLAN;
    ++revision;
  }
  void reset() { stable_since_=-1; last_stamp_=-1; clearCandidates(); }
};
} // namespace yopo_minco_planner
