#include "simulation_bridges/velodyne_pointcloud_converter.h"

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace {

template <typename T>
void writeValue(std::vector<std::uint8_t> *data, std::size_t offset, T value) {
    std::memcpy(data->data() + offset, &value, sizeof(T));
}

template <typename T>
T readValue(const std::vector<std::uint8_t> &data, std::size_t offset) {
    T value{};
    std::memcpy(&value, data.data() + offset, sizeof(T));
    return value;
}

sensor_msgs::PointField field(
    const std::string &name, std::uint32_t offset, std::uint8_t datatype) {
    sensor_msgs::PointField result;
    result.name = name;
    result.offset = offset;
    result.datatype = datatype;
    result.count = 1;
    return result;
}

}  // namespace

int main() {
    sensor_msgs::PointCloud2 input;
    input.header.frame_id = "laser_3d";
    input.header.stamp.fromSec(42.5);
    input.height = 1;
    input.width = 3;
    input.fields = {
        field("x", 0, sensor_msgs::PointField::FLOAT32),
        field("ring", 4, sensor_msgs::PointField::UINT16),
    };
    input.is_bigendian = false;
    input.point_step = 8;
    input.row_step = input.point_step * input.width;
    input.is_dense = true;
    input.data.resize(input.row_step);

    for (std::size_t index = 0; index < input.width; ++index) {
        const std::size_t offset = index * input.point_step;
        writeValue<float>(&input.data, offset, static_cast<float>(index + 1));
        writeValue<std::uint16_t>(
            &input.data, offset + 4, static_cast<std::uint16_t>(index));
    }

    sensor_msgs::PointCloud2 output;
    std::string error;
    assert(simulation_bridges::addVelodyneTimeField(input, 0.1, &output, &error));
    assert(error.empty());
    assert(output.header.frame_id == input.header.frame_id);
    assert(output.header.stamp == input.header.stamp);
    assert(output.width == 3);
    assert(output.height == 1);
    assert(output.fields.size() == 3);
    assert(output.fields.back().name == "time");
    assert(output.fields.back().offset == 8);
    assert(output.fields.back().datatype == sensor_msgs::PointField::FLOAT32);
    assert(output.point_step == 12);
    assert(output.row_step == 36);

    const float expected_times[] = {0.0F, 50000.0F, 100000.0F};
    for (std::size_t index = 0; index < output.width; ++index) {
        const std::size_t offset = index * output.point_step;
        assert(readValue<float>(output.data, offset) == static_cast<float>(index + 1));
        assert(readValue<std::uint16_t>(output.data, offset + 4) == index);
        assert(std::fabs(readValue<float>(output.data, offset + 8) - expected_times[index]) < 0.1F);
    }

    return 0;
}
