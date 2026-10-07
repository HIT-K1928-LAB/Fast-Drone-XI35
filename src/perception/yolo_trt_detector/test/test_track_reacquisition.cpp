#include <gtest/gtest.h>

#include "yolo_trt_detector/track_reacquisition.h"

namespace {

using yolo_trt_detector::ReacquisitionDecision;
using yolo_trt_detector::TrackReacquisitionPolicy;
using yolo_trt_detector::TrackReacquisitionConfig;
using Position = TrackReacquisitionPolicy::Position;

TrackReacquisitionPolicy makePolicy() {
  TrackReacquisitionConfig config;
  config.init_confidence = 0.45;
  config.confirm_hits = 3;
  config.confirm_window = 5;
  config.init_gate_distance = 1.2;
  config.identity_memory_seconds = 2.0;
  config.reacquire_distance = 1.5;
  config.max_measurement_jump = 1.0;
  return TrackReacquisitionPolicy(config);
}

TEST(TrackReacquisition, ThreeHitsInFiveConfirmNewIdentity) {
  auto policy = makePolicy();
  EXPECT_EQ(ReacquisitionDecision::kTentative,
            policy.observeCandidate(Position{{1.0, 0.0, 0.0}}, 0.46, 0.0));
  EXPECT_EQ(ReacquisitionDecision::kRejected,
            policy.observeCandidate(Position{{1.0, 0.0, 0.0}}, 0.30, 0.1));
  EXPECT_EQ(ReacquisitionDecision::kTentative,
            policy.observeCandidate(Position{{1.1, 0.0, 0.0}}, 0.50, 0.2));
  EXPECT_EQ(ReacquisitionDecision::kConfirmedNew,
            policy.observeCandidate(Position{{1.2, 0.0, 0.0}}, 0.60, 0.3));
}

TEST(TrackReacquisition, NearbyCandidateReacquiresRetainedIdentity) {
  auto policy = makePolicy();
  policy.rememberConfirmed(Position{{2.0, 1.0, 0.5}}, Position{{0.2, 0.0, 0.0}}, 1.0);
  policy.markLost(1.1);
  EXPECT_EQ(ReacquisitionDecision::kReacquired,
            policy.observeCandidate(Position{{2.35, 1.0, 0.5}}, 0.30, 1.5));
}

TEST(TrackReacquisition, LargeJumpIsRejectedWhileIdentityMemoryIsActive) {
  auto policy = makePolicy();
  policy.rememberConfirmed(Position{{0.0, 0.0, 0.0}}, Position{{0.0, 0.0, 0.0}}, 2.0);
  policy.markLost(2.1);
  EXPECT_EQ(ReacquisitionDecision::kRejected,
            policy.observeCandidate(Position{{1.1, 0.0, 0.0}}, 0.95, 2.2));
  EXPECT_TRUE(policy.hasRetainedIdentity(2.2));
}

TEST(TrackReacquisition, NewIdentityRequiresConfirmationAfterMemoryExpiry) {
  auto policy = makePolicy();
  policy.rememberConfirmed(Position{{0.0, 0.0, 0.0}}, Position{{0.0, 0.0, 0.0}}, 3.0);
  policy.markLost(3.1);
  EXPECT_EQ(ReacquisitionDecision::kRejected,
            policy.observeCandidate(Position{{4.0, 0.0, 0.0}}, 0.95, 4.0));
  EXPECT_EQ(ReacquisitionDecision::kTentative,
            policy.observeCandidate(Position{{4.0, 0.0, 0.0}}, 0.50, 5.2));
  EXPECT_EQ(ReacquisitionDecision::kTentative,
            policy.observeCandidate(Position{{4.1, 0.0, 0.0}}, 0.50, 5.3));
  EXPECT_EQ(ReacquisitionDecision::kConfirmedNew,
            policy.observeCandidate(Position{{4.2, 0.0, 0.0}}, 0.50, 5.4));
}

}  // namespace

int main(int argc, char** argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
