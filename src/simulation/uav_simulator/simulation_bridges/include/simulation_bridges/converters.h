#pragma once

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <string>

namespace simulation_bridges {

template <typename Stamp>
auto stampToNanoseconds(const Stamp &stamp) -> decltype(stamp.toNSec()) {
    return stamp.toNSec();
}

inline std::uint64_t stampToNanoseconds(std::uint64_t stamp) { return stamp; }

template <typename ModelStates, typename Stamp, typename Odometry>
bool modelStateToOdometry(
    const ModelStates &states, const std::string &model_name, const Stamp &stamp,
    const std::string &frame_id, const std::string &child_frame_id, Odometry *odom) {
    if (odom == nullptr) return false;

    const auto model = std::find(states.name.begin(), states.name.end(), model_name);
    if (model == states.name.end()) return false;

    const std::size_t index = static_cast<std::size_t>(std::distance(states.name.begin(), model));
    if (index >= states.pose.size() || index >= states.twist.size()) return false;

    odom->header.stamp = stamp;
    odom->header.frame_id = frame_id;
    odom->child_frame_id = child_frame_id;
    odom->pose.pose = states.pose[index];
    odom->twist.twist = states.twist[index];
    return true;
}

template <typename PointCloud, typename CustomMsg>
void pointCloudToLivoxCustom(
    const PointCloud &cloud, std::uint32_t scan_period_ns, std::uint32_t line_count,
    CustomMsg *custom) {
    if (custom == nullptr) return;

    custom->header = cloud.header;
    custom->timebase = stampToNanoseconds(cloud.header.stamp);
    custom->lidar_id = 0;
    custom->point_num = static_cast<std::uint32_t>(cloud.points.size());
    custom->points.resize(cloud.points.size());

    const std::size_t last_index = cloud.points.empty() ? 0 : cloud.points.size() - 1;
    const std::uint32_t valid_line_count = std::max<std::uint32_t>(1U, line_count);

    for (std::size_t i = 0; i < cloud.points.size(); ++i) {
        auto &target = custom->points[i];
        const auto &source = cloud.points[i];
        target.x = source.x;
        target.y = source.y;
        target.z = source.z;
        target.reflectivity = 0;
        target.tag = 0x10;
        target.line = static_cast<std::uint8_t>(i % valid_line_count);
        target.offset_time = last_index == 0
                                 ? 0U
                                 : static_cast<std::uint32_t>(
                                       (static_cast<std::uint64_t>(scan_period_ns) * i) /
                                       last_index);
    }
}

}  // namespace simulation_bridges
