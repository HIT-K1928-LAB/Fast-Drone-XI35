#!/usr/bin/env bash
set -e

fd_image_name="fastdronexi35:orin"
container_name="fd_runtime"
script_dir=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
container_id=$(docker ps -aq --filter "name=^/${container_name}$")

read -p "This will remove container ${container_name} and image ${fd_image_name}, then rebuild the Jetson image. Continue? (y/n): " user_input
if [ "${user_input}" != "y" ] && [ "${user_input}" != "Y" ]; then
    echo "Operation terminated."
    exit 1
fi

if [ -n "${container_id}" ]; then
    echo "Stopping and removing container ${container_name}..."
    docker stop "${container_id}" >/dev/null 2>&1 || true
    docker rm "${container_id}"
    echo "Container ${container_name} has been removed."
fi

if docker image inspect "${fd_image_name}" >/dev/null 2>&1; then
    docker rmi "${fd_image_name}"
fi

make -C "${script_dir}" jetson_rebuild
