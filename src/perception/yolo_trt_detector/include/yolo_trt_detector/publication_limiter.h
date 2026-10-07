#pragma once

#include <cmath>
#include <stdexcept>

namespace yolo_trt_detector {

class PublicationLimiter {
 public:
  explicit PublicationLimiter(double rate_hz = 10.0) { configure(rate_hz); }

  void configure(double rate_hz) {
    if (!std::isfinite(rate_hz) || rate_hz <= 0.0) {
      throw std::invalid_argument("publication rate must be finite and positive");
    }
    period_seconds_ = 1.0 / rate_hz;
    reset();
  }

  bool shouldPublish(double stamp_seconds) {
    if (!std::isfinite(stamp_seconds)) {
      return false;
    }
    if (!has_last_stamp_ || stamp_seconds < last_stamp_seconds_ ||
        stamp_seconds - last_stamp_seconds_ + 1e-9 >= period_seconds_) {
      last_stamp_seconds_ = stamp_seconds;
      has_last_stamp_ = true;
      return true;
    }
    return false;
  }

  void reset() {
    has_last_stamp_ = false;
    last_stamp_seconds_ = 0.0;
  }

 private:
  double period_seconds_ = 0.1;
  bool has_last_stamp_ = false;
  double last_stamp_seconds_ = 0.0;
};

}  // namespace yolo_trt_detector
