#!/usr/bin/env bash

ROS_VERSION=${ROS_VERSION:-noetic}
OPENCV_VERSION=${OPENCV_VERSION:-4.5.4}
FDRONE_WS=${FDRONE_WS:-/root/Fast-Drone-XI35}
LOG_DIR=${FDRONE_WS}/log

append_bashrc_line() {
    local line="$1"
    if ! grep -Fxq "$line" /root/.bashrc; then
        echo "$line" >> /root/.bashrc
    fi
}

# modify mavros
PX4_LAUNCH=/opt/ros/${ROS_VERSION}/share/mavros/launch/px4.launch
if [ -f "${PX4_LAUNCH}" ]; then
    sed -i 's|/dev/ttyACM0:57600|/dev/ttyTHS0:921600|1' "${PX4_LAUNCH}"
fi

# LCM Config
if ifconfig wlan1 >/dev/null 2>&1; then
    ifconfig wlan1 multicast || true
    route add -net 224.0.0.0 netmask 240.0.0.0 dev wlan1 2>/dev/null || true
fi

# start service
service ssh start
/daemon/nvargus-daemon &

mkdir -p "${LOG_DIR}"

# add ROS environment to interactive shells
append_bashrc_line "export ROS_LOG_DIR=${LOG_DIR}"
append_bashrc_line "source /opt/ros/${ROS_VERSION}/setup.bash"
append_bashrc_line "source /root/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash"

if ! grep -q "Fast-Drone-XI35/devel/setup.bash" /root/.bashrc; then
    cat >> /root/.bashrc <<'EOF'
if [ -f /root/Fast-Drone-XI35/devel/setup.bash ]; then
    source /root/Fast-Drone-XI35/devel/setup.bash
fi
EOF
fi

# Sourcing ROS environment
if [ -f /opt/ros/${ROS_VERSION}/setup.bash ]; then
    source /opt/ros/${ROS_VERSION}/setup.bash
fi
if [ -f /root/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash ]; then
    source /root/cv_bridge${OPENCV_VERSION}_ws/devel/setup.bash
fi

cd "${FDRONE_WS}" 2>/dev/null || cd /root

if [ "$#" -gt 0 ]; then
    exec "$@"
fi

exec /bin/bash
