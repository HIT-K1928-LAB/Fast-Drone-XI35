#!/usr/bin/env bash
set -e

CONTAINER_NAME=${CONTAINER_NAME:-fd_runtime}
IMAGE_NAME=${IMAGE_NAME:-fastdronexi35:orin}
SHM_SIZE=${SHM_SIZE:-16g}
SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
PROJECT_DIR=${PROJECT_DIR:-$(cd "${SCRIPT_DIR}/../.." && pwd)}
DATA_DIR=${DATA_DIR:-${HOME}/Docker_Data}

# Always reserve X11 support.
# If DISPLAY is empty now, use :0 by default so the container can use X11 later.
DISPLAY_VALUE=${DISPLAY:-:0}
XAUTH_FILE=${XAUTHORITY:-${HOME}/.Xauthority}

# check if the container already exists
if docker ps -a --format '{{.Names}}' | grep -qw "${CONTAINER_NAME}"; then
    echo "Container '${CONTAINER_NAME}' already exists."
    echo "Use: docker start ${CONTAINER_NAME} && docker exec -it ${CONTAINER_NAME} bash"
    exit 0
fi

mkdir -p "${DATA_DIR}"
mkdir -p "${PROJECT_DIR}/log"

# Make sure the X11 socket directory exists.
# This is useful when the machine is currently headless but may attach display later.
if [ ! -d /tmp/.X11-unix ]; then
    mkdir -p /tmp/.X11-unix
    chmod 1777 /tmp/.X11-unix || true
fi

# Try to allow local root user in container to access host X server.
# It is okay if this fails when no display server is currently running.
if command -v xhost >/dev/null 2>&1; then
    DISPLAY="${DISPLAY_VALUE}" xhost +local:root >/dev/null 2>&1 || true
fi

DOCKER_ARGS=(
    -itd
    --privileged=true
    --network host
    --shm-size="${SHM_SIZE}"

    -e ACCEPT_EULA=Y
    -e PRIVACY_CONSENT=Y

    # X11 / Qt GUI
    -e DISPLAY="${DISPLAY_VALUE}"
    -e QT_X11_NO_MITSHM=1

    # NVIDIA / Jetson
    -e NVIDIA_VISIBLE_DEVICES=all
    -e NVIDIA_DRIVER_CAPABILITIES=all

    # ROS
    -e ROS_LOG_DIR=/root/Fast-Drone-XI35/log

    # X11 socket, always mounted
    -v /tmp/.X11-unix:/tmp/.X11-unix:rw

    # Jetson Argus camera support
    --mount type=bind,source=/usr/sbin/nvargus-daemon,target=/daemon/nvargus-daemon,readonly

    # Data / devices / project
    -v "${DATA_DIR}:/root/data"
    -v /dev:/dev
    -v "${PROJECT_DIR}:/root/Fast-Drone-XI35"

    --runtime=nvidia
    --gpus all
    --name "${CONTAINER_NAME}"
)

if [ -f /etc/localtime ]; then
    DOCKER_ARGS+=(-v /etc/localtime:/etc/localtime:ro)
fi

if [ -f /etc/timezone ]; then
    DOCKER_ARGS+=(-v /etc/timezone:/etc/timezone:ro)
fi

if [ -d /run/udev ]; then
    DOCKER_ARGS+=(-v /run/udev:/run/udev:ro)
fi

if [ -d /tmp/argus_socket ]; then
    DOCKER_ARGS+=(-v /tmp/argus_socket:/tmp/argus_socket)
fi

# Mount Xauthority only if it already exists.
# If it does not exist, X11 can still work through xhost +local:root.
if [ -f "${XAUTH_FILE}" ]; then
    DOCKER_ARGS+=(
        -e XAUTHORITY=/root/.Xauthority
        -v "${XAUTH_FILE}:/root/.Xauthority:ro"
    )
fi

docker run "${DOCKER_ARGS[@]}" "${IMAGE_NAME}" /bin/bash

echo "Container '${CONTAINER_NAME}' started."
echo "Mounted project: ${PROJECT_DIR} -> /root/Fast-Drone-XI35"
echo "X11 DISPLAY inside container: ${DISPLAY_VALUE}"
echo "Enter with: docker exec -it ${CONTAINER_NAME} bash"