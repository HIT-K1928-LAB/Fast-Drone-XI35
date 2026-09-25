#ifndef FAST_LIVO_OUTPUT_PATH_H
#define FAST_LIVO_OUTPUT_PATH_H

#include <filesystem>

namespace fast_livo
{

inline std::filesystem::path resolvePcdOutputDirectory(
    const char *session_log_root,
    const std::filesystem::path &package_root)
{
  if (session_log_root != nullptr && session_log_root[0] != '\0')
  {
    return std::filesystem::path(session_log_root) / "fastlivo2" / "pcd";
  }
  return package_root / "Log" / "pcd";
}

}  // namespace fast_livo

#endif
