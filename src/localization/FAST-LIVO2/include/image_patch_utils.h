#ifndef FAST_LIVO_IMAGE_PATCH_UTILS_H_
#define FAST_LIVO_IMAGE_PATCH_UTILS_H_

#include <Eigen/Core>
#include <opencv2/core.hpp>

#include <cmath>
#include <cstdint>
#include <limits>

namespace fast_livo
{

// Computes the aligned integer origin used by the raw patch reads and checks
// their complete access extent. Returning the origin is intentional: callers
// must use these exact integers instead of repeating a potentially different
// float/double rounding operation after the bounds check.
inline bool getPatchAccessOrigin(const cv::Mat &image,
                                 const Eigen::Vector2d &pixel,
                                 int patch_size,
                                 int patch_size_half,
                                 int scale,
                                 int &aligned_u,
                                 int &aligned_v)
{
  if (image.empty() || image.type() != CV_8UC1 || patch_size <= 0 ||
      patch_size_half < 0 || patch_size_half > patch_size ||
      scale <= 0 || !pixel.allFinite())
  {
    return false;
  }

  const double aligned_u_double = std::floor(pixel.x() / scale) * scale;
  const double aligned_v_double = std::floor(pixel.y() / scale) * scale;
  if (aligned_u_double < std::numeric_limits<int>::min() ||
      aligned_u_double > std::numeric_limits<int>::max() ||
      aligned_v_double < std::numeric_limits<int>::min() ||
      aligned_v_double > std::numeric_limits<int>::max())
  {
    return false;
  }

  const int candidate_u = static_cast<int>(aligned_u_double);
  const int candidate_v = static_cast<int>(aligned_v_double);
  const int64_t lower_extent = (static_cast<int64_t>(patch_size_half) + 1) * scale;
  const int64_t upper_extent =
      (static_cast<int64_t>(patch_size) - patch_size_half + 1) * scale;
  const int64_t min_u = static_cast<int64_t>(candidate_u) - lower_extent;
  const int64_t min_v = static_cast<int64_t>(candidate_v) - lower_extent;
  const int64_t max_u = static_cast<int64_t>(candidate_u) + upper_extent;
  const int64_t max_v = static_cast<int64_t>(candidate_v) + upper_extent;

  if (min_u < 0 || min_v < 0 || max_u >= image.cols || max_v >= image.rows)
    return false;

  aligned_u = candidate_u;
  aligned_v = candidate_v;
  return true;
}

inline bool isPatchAccessInFrame(const cv::Mat &image,
                                 const Eigen::Vector2d &pixel,
                                 int patch_size,
                                 int patch_size_half,
                                 int scale)
{
  int aligned_u = 0;
  int aligned_v = 0;
  return getPatchAccessOrigin(
      image, pixel, patch_size, patch_size_half, scale, aligned_u, aligned_v);
}

}  // namespace fast_livo

#endif  // FAST_LIVO_IMAGE_PATCH_UTILS_H_
