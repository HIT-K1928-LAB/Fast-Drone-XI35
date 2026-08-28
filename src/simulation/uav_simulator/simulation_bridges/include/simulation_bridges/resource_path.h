#pragma once

#include <string>

namespace simulation_bridges {

template <typename PackageResolver>
std::string resolvePackageUri(const std::string &uri, PackageResolver resolver) {
    const std::string prefix = "package://";
    if (uri.compare(0, prefix.size(), prefix) != 0) return uri;

    const std::string package_path = uri.substr(prefix.size());
    const std::size_t separator = package_path.find('/');
    if (separator == std::string::npos || separator == 0 || separator + 1 >= package_path.size()) {
        return {};
    }

    const std::string package_root = resolver(package_path.substr(0, separator));
    if (package_root.empty()) return {};

    return package_root + "/" + package_path.substr(separator + 1);
}

}  // namespace simulation_bridges
