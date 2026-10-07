#pragma once

#include <array>
#include <cmath>
#include <string>

namespace yopo_planner {

enum class DynamicTargetRejectReason {
    kNone,
    kTrackerState,
    kNonFinite,
    kSpeed,
    kRange,
    kCovariance,
};

struct DynamicTargetSafetyConfig {
    double max_speed = 4.0;
    double max_range = 18.0;
    double max_position_variance = 0.50;
};

struct DynamicTargetSample {
    std::array<double, 3> position{{0.0, 0.0, 0.0}};
    std::array<double, 3> velocity{{0.0, 0.0, 0.0}};
    std::array<double, 3> position_variance{{0.0, 0.0, 0.0}};
    std::array<double, 3> vehicle_position{{0.0, 0.0, 0.0}};
    std::string tracker_status;
};

inline DynamicTargetRejectReason validateDynamicTarget(
    const DynamicTargetSample& sample,
    const DynamicTargetSafetyConfig& config) {
    if (sample.tracker_status != "CONFIRMED" &&
        sample.tracker_status != "COASTING") {
        return DynamicTargetRejectReason::kTrackerState;
    }
    double speed2 = 0.0;
    double range2 = 0.0;
    for (int axis = 0; axis < 3; ++axis) {
        if (!std::isfinite(sample.position[axis]) ||
            !std::isfinite(sample.velocity[axis]) ||
            !std::isfinite(sample.position_variance[axis]) ||
            !std::isfinite(sample.vehicle_position[axis])) {
            return DynamicTargetRejectReason::kNonFinite;
        }
        speed2 += sample.velocity[axis] * sample.velocity[axis];
        const double offset = sample.position[axis] - sample.vehicle_position[axis];
        range2 += offset * offset;
        if (sample.position_variance[axis] < 0.0 ||
            sample.position_variance[axis] > config.max_position_variance) {
            return DynamicTargetRejectReason::kCovariance;
        }
    }
    if (std::sqrt(speed2) > config.max_speed) {
        return DynamicTargetRejectReason::kSpeed;
    }
    if (std::sqrt(range2) > config.max_range) {
        return DynamicTargetRejectReason::kRange;
    }
    return DynamicTargetRejectReason::kNone;
}

inline const char* dynamicTargetRejectReasonName(
    DynamicTargetRejectReason reason) {
    switch (reason) {
        case DynamicTargetRejectReason::kNone: return "none";
        case DynamicTargetRejectReason::kTrackerState: return "tracker state";
        case DynamicTargetRejectReason::kNonFinite: return "non-finite value";
        case DynamicTargetRejectReason::kSpeed: return "target speed";
        case DynamicTargetRejectReason::kRange: return "target range";
        case DynamicTargetRejectReason::kCovariance: return "target covariance";
    }
    return "unknown";
}

}  // namespace yopo_planner
