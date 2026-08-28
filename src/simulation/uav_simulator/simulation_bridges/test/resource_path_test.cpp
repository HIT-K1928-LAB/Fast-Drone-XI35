#include "simulation_bridges/resource_path.h"

#include <cassert>
#include <string>

int main() {
    const auto resolver = [](const std::string &package_name) {
        return package_name == "simulation_bridges" ? std::string("/workspace/simulation_bridges")
                                                     : std::string();
    };

    assert(simulation_bridges::resolvePackageUri(
               "package://simulation_bridges/scan_mode/mid360.csv", resolver) ==
           "/workspace/simulation_bridges/scan_mode/mid360.csv");
    assert(simulation_bridges::resolvePackageUri("/tmp/mid360.csv", resolver) ==
           "/tmp/mid360.csv");
    assert(simulation_bridges::resolvePackageUri("package://missing/file.csv", resolver).empty());
    assert(simulation_bridges::resolvePackageUri("package://simulation_bridges", resolver).empty());
    return 0;
}
