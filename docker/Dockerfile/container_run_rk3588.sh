#!/usr/bin/env bash
set -euo pipefail

CONTAINER_NAME=${CONTAINER_NAME:-fd_runtime_rk3588}
IMAGE_NAME=${IMAGE_NAME:-fastdronexi35:rk3588}
SHM_SIZE=${SHM_SIZE:-4g}
MAVROS_FCU_URL=${MAVROS_FCU_URL:-/dev/ttyS7:921600}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=${PROJECT_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}
DISPLAY_VALUE=${DISPLAY:-:0}
XAUTH_FILE=${XAUTHORITY:-${HOME}/.Xauthority}

if docker ps -a --format '{{.Names}}' | grep -qw "${CONTAINER_NAME}"; then
    echo "Container '${CONTAINER_NAME}' already exists."
    echo "Use: docker start ${CONTAINER_NAME} && docker exec -it ${CONTAINER_NAME} bash"
    exit 0
fi

if [ ! -e /dev/dri/renderD129 ]; then
    echo "RK3588 NPU device /dev/dri/renderD129 is missing."
    echo "Check: readlink -f /sys/class/drm/renderD129/device/driver"
    exit 1
fi

NPU_DRIVER=$(readlink -f /sys/class/drm/renderD129/device/driver 2>/dev/null || true)
if [[ "${NPU_DRIVER}" != */RKNPU ]]; then
    echo "Warning: renderD129 is not bound to the expected RKNPU driver: ${NPU_DRIVER:-unknown}"
fi

mkdir -p "${PROJECT_DIR}/log"

if command -v xhost >/dev/null 2>&1; then
    DISPLAY="${DISPLAY_VALUE}" xhost +local:root >/dev/null 2>&1 || true
fi

DOCKER_ARGS=(
    -itd
    --privileged
    --network host
    --shm-size="${SHM_SIZE}"
    -e DISPLAY="${DISPLAY_VALUE}"
    -e QT_X11_NO_MITSHM=1
    -e ROS_LOG_DIR=/root/Fast-Drone-XI35/log
    -e RKNN_TARGET_SOC=RK3588
    -e MAVROS_FCU_URL="${MAVROS_FCU_URL}"
    -v /dev:/dev
    -v "${PROJECT_DIR}:/root/Fast-Drone-XI35"
    -v "${PROJECT_DIR}/docker/container_init.sh:/container_init.sh:ro"
    --name "${CONTAINER_NAME}"
)

if [ -d /tmp/.X11-unix ]; then
    DOCKER_ARGS+=(-v /tmp/.X11-unix:/tmp/.X11-unix:rw)
fi

if [ -f "${XAUTH_FILE}" ]; then
    DOCKER_ARGS+=(
        -e XAUTHORITY=/root/.Xauthority
        -v "${XAUTH_FILE}:/root/.Xauthority:ro"
    )
fi

if [ -f /etc/localtime ]; then
    DOCKER_ARGS+=(-v /etc/localtime:/etc/localtime:ro)
fi

if [ -f /etc/timezone ]; then
    DOCKER_ARGS+=(-v /etc/timezone:/etc/timezone:ro)
fi

if [ -d /run/udev ]; then
    DOCKER_ARGS+=(-v /run/udev:/run/udev:ro)
fi

docker run "${DOCKER_ARGS[@]}" "${IMAGE_NAME}" /bin/bash

echo "Container '${CONTAINER_NAME}' started."
echo "NPU device: /dev/dri/renderD129 (${NPU_DRIVER})"
echo "MAVROS FCU: ${MAVROS_FCU_URL}"
echo "Enter: docker exec -it ${CONTAINER_NAME} bash"
