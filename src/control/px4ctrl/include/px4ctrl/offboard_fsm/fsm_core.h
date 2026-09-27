#pragma once

#include <cstdint>
#include <string>

#include <px4ctrl/offboard_fsm/config.h>
#include <px4ctrl/offboard_fsm/types.h>

namespace px4ctrl
{

Events rcTakeoffLandEvent(FsmState state, bool extended_state_fresh,
                          uint8_t landed_state);

class FsmCore
{
public:
  explicit FsmCore(const Config& config);

  FsmOutput step(const InputSnapshot& input, double dt);
  FsmState state() const { return state_; }
  const std::string& transitionReason() const { return transition_reason_; }
  const Reference& reference() const { return reference_; }
  bool plannerAlignmentValid() const { return planner_alignment_valid_; }

private:
  void transition(FsmState next, const std::string& reason,
                  const InputSnapshot& input);
  void captureHold(const InputSnapshot& input);
  void capturePlannerAlignment(const InputSnapshot& input);
  bool activeState() const;
  bool healthyForControl(const InputSnapshot& input) const;
  bool streamReady(const InputSnapshot& input) const;
  bool plannerCommandValid(const InputSnapshot& input,
                           bool activation_check) const;
  Reference transformedPlannerReference(const InputSnapshot& input) const;
  void updateVerticalProfile(double target_z, double max_velocity,
                             double max_acceleration, double max_jerk,
                             double dt);
  void updateLandingProfile(const InputSnapshot& input, double dt);
  void updateBrakeProfile(double dt);
  void notePublishing(const InputSnapshot& input, bool publishing);

  const Config config_;
  FsmState state_{FsmState::WAIT_FCU};
  double state_entered_at_{0.0};
  std::string transition_reason_{"node initialization"};
  Reference reference_;

  geometry_msgs::Point hold_position_;
  double hold_yaw_{0.0};
  geometry_msgs::Point takeoff_origin_;
  bool have_takeoff_origin_{false};
  geometry_msgs::Point safety_origin_;
  bool have_safety_origin_{false};
  double requested_takeoff_height_{0.0};

  double vertical_velocity_{0.0};
  double vertical_acceleration_{0.0};
  double landing_floor_limit_{0.0};
  geometry_msgs::Vector3 brake_velocity_;
  double stable_since_{0.0};
  double touchdown_since_{0.0};
  double landing_control_elapsed_{0.0};

  double yaw_offset_{0.0};
  geometry_msgs::Vector3 translation_;
  bool planner_alignment_valid_{false};
  double planner_accept_after_{0.0};
  uint64_t last_planner_sequence_{0};
  int planner_activation_count_{0};
  bool pending_takeoff_{false};

  double stream_started_at_{0.0};
  double last_publish_at_{0.0};
};

}  // namespace px4ctrl
