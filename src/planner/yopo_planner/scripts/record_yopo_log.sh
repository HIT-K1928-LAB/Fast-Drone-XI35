#!/usr/bin/env bash
set -e

log_dir="${1:-$HOME/yopo_logs}"
shift || true
bag_prefix="${1:-yopo_log}"
shift || true

mkdir -p "$log_dir"
stamp="$(date +%Y%m%d_%H%M%S)"

exec rosbag record -O "$log_dir/${bag_prefix}_${stamp}.bag" "$@"
