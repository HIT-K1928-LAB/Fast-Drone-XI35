#pragma once

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointField.h>

#include <cmath>
#include <cstdint>
#include <cstring>
#include <string>

namespace simulation_bridges {

inline bool addVelodyneTimeField(
    const sensor_msgs::PointCloud2 &input, double scan_period,
    sensor_msgs::PointCloud2 *output, std::string *error = nullptr) {
    const auto fail = [error](const std::string &message) {
        if (error != nullptr) *error = message;
        return false;
    };

    if (output == nullptr) return fail("output message is null");
    if (!std::isfinite(scan_period) || scan_period <= 0.0) {
        return fail("scan_period must be positive and finite");
    }
    if (input.is_bigendian) return fail("big-endian point clouds are not supported");
    if (input.point_step == 0) return fail("point_step must be non-zero");

    const std::size_t point_count =
        static_cast<std::size_t>(input.width) * input.height;
    const std::size_t required_input_size =
        input.height == 0 ? 0 :
        static_cast<std::size_t>(input.row_step) * (input.height - 1) +
            static_cast<std::size_t>(input.point_step) * input.width;
    if (input.data.size() < required_input_size) {
        return fail("point cloud data is shorter than its dimensions");
    }

    for (const auto &field : input.fields) {
        if (field.name == "time") {
            *output = input;
            if (error != nullptr) error->clear();
            return true;
        }
    }

    *output = input;
    sensor_msgs::PointField time_field;
    time_field.name = "time";
    time_field.offset = input.point_step;
    time_field.datatype = sensor_msgs::PointField::FLOAT32;
    time_field.count = 1;
    output->fields.push_back(time_field);
    output->point_step = input.point_step + sizeof(float);
    output->row_step = output->point_step * output->width;
    output->data.assign(
        static_cast<std::size_t>(output->row_step) * output->height, 0U);

    const float scan_period_microseconds =
        static_cast<float>(scan_period * 1.0e6);
    const std::size_t last_index = point_count == 0 ? 0 : point_count - 1;
    for (std::size_t row = 0; row < input.height; ++row) {
        for (std::size_t column = 0; column < input.width; ++column) {
            const std::size_t index = row * input.width + column;
            const std::size_t input_offset =
                row * input.row_step + column * input.point_step;
            const std::size_t output_offset =
                row * output->row_step + column * output->point_step;
            std::memcpy(
                output->data.data() + output_offset,
                input.data.data() + input_offset,
                input.point_step);

            const float point_time = last_index == 0
                                         ? 0.0F
                                         : scan_period_microseconds *
                                               static_cast<float>(index) /
                                               static_cast<float>(last_index);
            std::memcpy(
                output->data.data() + output_offset + time_field.offset,
                &point_time, sizeof(point_time));
        }
    }

    if (error != nullptr) error->clear();
    return true;
}

}  // namespace simulation_bridges
