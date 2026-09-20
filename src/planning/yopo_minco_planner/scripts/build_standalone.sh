#!/usr/bin/env bash
# Build only this package against the already-working flight workspace.
set -eo pipefail
package_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
algorithm_root=$(cd "${package_dir}/../../.." && pwd)
build_dir=${YOPO_MINCO_BUILD_DIR:-${algorithm_root}/build_yopo_minco}
source "${FLIGHT_SETUP:-/root/Fast-Drone-XI35/devel/setup.bash}"
# The board's custom OpenCV 4.5.4 lacks photo. This node does not use cv_bridge;
# its verified standalone build uses the complete system OpenCV installation.
opencv_dir=${YOPO_MINCO_OPENCV_DIR:-/usr/lib/aarch64-linux-gnu/cmake/opencv4}
cmake -S "$package_dir" -B "$build_dir" \
  -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DOpenCV_DIR="$opencv_dir"
cmake --build "$build_dir" -j2
printf '\nSource this overlay after the flight workspace:\nsource %s/devel/setup.bash\n' "$build_dir"
