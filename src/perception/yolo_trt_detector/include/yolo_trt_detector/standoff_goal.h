#pragma once

#include <cmath>

#include <tf2/LinearMath/Vector3.h>

namespace yolo_trt_detector {

// Return the point on the vehicle-to-target segment that remains
// standoff_distance metres from the target.  When the target is already
// closer than that distance, holding the current vehicle position is safer
// than extending the segment behind the vehicle.
inline bool calculateStandoffGoal(
    const tf2::Vector3& vehicle_position, const tf2::Vector3& target_position,
    double standoff_distance, tf2::Vector3& goal_position) {
    if (!std::isfinite(vehicle_position.x()) ||
        !std::isfinite(vehicle_position.y()) ||
        !std::isfinite(vehicle_position.z()) ||
        !std::isfinite(target_position.x()) ||
        !std::isfinite(target_position.y()) ||
        !std::isfinite(target_position.z()) ||
        !std::isfinite(standoff_distance) || standoff_distance < 0.0) {
        return false;
    }

    const tf2::Vector3 vehicle_to_target = target_position - vehicle_position;
    const double target_range = vehicle_to_target.length();
    if (target_range <= standoff_distance || target_range < 1e-6) {
        goal_position = vehicle_position;
        return true;
    }

    goal_position =
        target_position - vehicle_to_target * (standoff_distance / target_range);
    return true;
}

}  // namespace yolo_trt_detector
