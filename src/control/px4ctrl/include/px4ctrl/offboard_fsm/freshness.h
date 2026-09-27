#pragma once

#include <cmath>

namespace px4ctrl {

// Callback receipt times and control-cycle times use the same monotonic clock.
// With a multi-threaded spinner, a callback can update its receipt time after a
// control cycle sampled `now` but before that cycle evaluates freshness.  Such
// a small negative age means "newer than this snapshot", not stale data.
inline bool receiptFresh(const double receipt_time, const double timeout,
                         const double now) {
  if (!std::isfinite(receipt_time) || !std::isfinite(timeout) ||
      !std::isfinite(now) || receipt_time <= 0.0 || timeout < 0.0) {
    return false;
  }
  return now - receipt_time <= timeout;
}

} // namespace px4ctrl
