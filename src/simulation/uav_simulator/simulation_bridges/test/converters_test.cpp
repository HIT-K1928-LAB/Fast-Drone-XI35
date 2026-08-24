#include "simulation_bridges/converters.h"

#include <cassert>
#include <cstdint>
#include <string>
#include <vector>

struct Header {
    std::uint64_t stamp{};
    std::string frame_id;
};

struct Point {
    float x{};
    float y{};
    float z{};
};

struct Pose {
    int value{};
};

struct Twist {
    int value{};
};

struct ModelStates {
    std::vector<std::string> name;
    std::vector<Pose> pose;
    std::vector<Twist> twist;
};

struct Odometry {
    Header header;
    std::string child_frame_id;
    struct {
        Pose pose;
    } pose;
    struct {
        Twist twist;
    } twist;
};

struct PointCloud {
    Header header;
    std::vector<Point> points;
};

struct CustomPoint {
    std::uint32_t offset_time{};
    float x{};
    float y{};
    float z{};
    std::uint8_t reflectivity{};
    std::uint8_t tag{};
    std::uint8_t line{};
};

struct CustomMsg {
    Header header;
    std::uint64_t timebase{};
    std::uint32_t point_num{};
    std::uint8_t lidar_id{};
    std::vector<CustomPoint> points;
};

int main() {
    ModelStates states{{"ground_plane", "iris_0"}, {{1}, {42}}, {{2}, {84}}};
    Odometry odom;
    const bool found = simulation_bridges::modelStateToOdometry(
        states, "iris_0", std::uint64_t{123}, "world", "base_link", &odom);
    assert(found);
    assert(odom.header.stamp == 123);
    assert(odom.header.frame_id == "world");
    assert(odom.child_frame_id == "base_link");
    assert(odom.pose.pose.value == 42);
    assert(odom.twist.twist.value == 84);

    Odometry missing_odom;
    assert(!simulation_bridges::modelStateToOdometry(
        states, "missing", std::uint64_t{456}, "world", "base_link", &missing_odom));

    PointCloud cloud;
    cloud.header.stamp = 1000;
    cloud.header.frame_id = "livox";
    cloud.points = {{1.0F, 2.0F, 3.0F}, {4.0F, 5.0F, 6.0F}, {7.0F, 8.0F, 9.0F}};
    CustomMsg custom;
    simulation_bridges::pointCloudToLivoxCustom(cloud, 100000000U, 4U, &custom);
    assert(custom.header.stamp == 1000);
    assert(custom.header.frame_id == "livox");
    assert(custom.timebase == 1000);
    assert(custom.point_num == 3);
    assert(custom.points.size() == 3);
    assert(custom.points[0].x == 1.0F);
    assert(custom.points[1].y == 5.0F);
    assert(custom.points[2].z == 9.0F);
    assert(custom.points[0].offset_time == 0U);
    assert(custom.points[1].offset_time == 50000000U);
    assert(custom.points[2].offset_time == 100000000U);
    assert(custom.points[0].tag == 0x10U);
    assert(custom.points[1].line == 1U);
    assert(custom.points[2].line == 2U);

    return 0;
}
