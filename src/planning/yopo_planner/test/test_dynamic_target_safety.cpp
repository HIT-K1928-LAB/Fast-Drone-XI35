#include <gtest/gtest.h>

#include <array>
#include <limits>

#include "yopo_planner/dynamic_target_safety.h"

namespace {

yopo_planner::DynamicTargetSample safeSample() {
    yopo_planner::DynamicTargetSample sample;
    sample.position = {{5.0, 0.0, 1.5}};
    sample.velocity = {{1.0, 0.0, 0.0}};
    sample.position_variance = {{0.05, 0.05, 0.05}};
    sample.vehicle_position = {{0.0, 0.0, 1.5}};
    sample.tracker_status = "CONFIRMED";
    return sample;
}

}  // namespace

TEST(DynamicTargetSafety, AcceptsPhysicallyPlausibleConfirmedTrack) {
    const yopo_planner::DynamicTargetSafetyConfig config;
    EXPECT_EQ(yopo_planner::validateDynamicTarget(safeSample(), config),
              yopo_planner::DynamicTargetRejectReason::kNone);
}

TEST(DynamicTargetSafety, RejectsTentativeIdentity) {
    auto sample = safeSample();
    sample.tracker_status = "TENTATIVE";
    EXPECT_EQ(yopo_planner::validateDynamicTarget(sample, {}),
              yopo_planner::DynamicTargetRejectReason::kTrackerState);
}

TEST(DynamicTargetSafety, RejectsImplausibleTargetSpeed) {
    auto sample = safeSample();
    sample.velocity = {{4.1, 0.0, 0.0}};
    yopo_planner::DynamicTargetSafetyConfig config;
    config.max_speed = 4.0;
    EXPECT_EQ(yopo_planner::validateDynamicTarget(sample, config),
              yopo_planner::DynamicTargetRejectReason::kSpeed);
}

TEST(DynamicTargetSafety, RejectsTargetOutsideTrackingRange) {
    auto sample = safeSample();
    sample.position = {{18.1, 0.0, 1.5}};
    yopo_planner::DynamicTargetSafetyConfig config;
    config.max_range = 18.0;
    EXPECT_EQ(yopo_planner::validateDynamicTarget(sample, config),
              yopo_planner::DynamicTargetRejectReason::kRange);
}

TEST(DynamicTargetSafety, RejectsLargeOrNonFiniteCovariance) {
    auto sample = safeSample();
    sample.position_variance = {{0.05, 0.51, 0.05}};
    yopo_planner::DynamicTargetSafetyConfig config;
    config.max_position_variance = 0.50;
    EXPECT_EQ(yopo_planner::validateDynamicTarget(sample, config),
              yopo_planner::DynamicTargetRejectReason::kCovariance);

    sample = safeSample();
    sample.position_variance[0] = std::numeric_limits<double>::quiet_NaN();
    EXPECT_EQ(yopo_planner::validateDynamicTarget(sample, config),
              yopo_planner::DynamicTargetRejectReason::kNonFinite);
}

int main(int argc, char** argv) {
    testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
