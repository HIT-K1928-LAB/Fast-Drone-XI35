#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE=$(cd "$SCRIPT_DIR/.." && pwd)
LOG_DIR="$WORKSPACE/log"
TIME_DIR_REGEX='^[0-9]{4}-[0-9]{2}-[0-9]{2}_[0-9]{2}-[0-9]{2}-[0-9]{2}$'

usage() {
    echo "Usage:"
    echo "  $0 delete"
    echo "  $0 convert"
}

delete_logs() {
    mkdir -p "$LOG_DIR"
    find "$LOG_DIR" -mindepth 1 -maxdepth 1 -exec rm -rf -- {} +
    echo "Deleted all logs under: $LOG_DIR"
}

latest_time_dir_name() {
    local run_dir="$1"
    local time_dirs=()

    mapfile -t time_dirs < <(
        find "$run_dir" -mindepth 1 -maxdepth 1 -type d -printf "%f\n" |
            grep -E "$TIME_DIR_REGEX" |
            sort
    )

    if [ "${#time_dirs[@]}" -eq 0 ]; then
        return 1
    fi

    echo "${time_dirs[$((${#time_dirs[@]} - 1))]}"
}

convert_ros_log_dirs() {
    local converted=0
    local skipped=0
    local latest_target=""

    mkdir -p "$LOG_DIR"

    if [ -L "$LOG_DIR/latest" ]; then
        latest_target=$(readlink -f "$LOG_DIR/latest" || true)
    fi

    for run_dir in "$LOG_DIR"/*; do
        [ -d "$run_dir" ] || continue
        [ -L "$run_dir" ] && continue

        local run_name
        run_name=$(basename "$run_dir")

        if [[ "$run_name" =~ $TIME_DIR_REGEX ]]; then
            skipped=$((skipped + 1))
            continue
        fi

        local time_name
        if ! time_name=$(latest_time_dir_name "$run_dir"); then
            echo "Skip $run_name: no timestamp log directory found."
            skipped=$((skipped + 1))
            continue
        fi

        local target_dir="$LOG_DIR/$time_name"
        if [ -e "$target_dir" ]; then
            echo "Skip $run_name: target already exists: $time_name"
            skipped=$((skipped + 1))
            continue
        fi

        mv "$run_dir" "$target_dir"
        echo "Renamed $run_name -> $time_name"
        converted=$((converted + 1))

        if [ -n "$latest_target" ] && [ "$latest_target" = "$run_dir" ]; then
            ln -sfn "$target_dir" "$LOG_DIR/latest"
            latest_target="$target_dir"
        fi
    done

    echo "Converted: $converted, skipped: $skipped"
}

case "${1:-}" in
    delete)
        delete_logs
        ;;
    convert)
        convert_ros_log_dirs
        ;;
    -h | --help | help | "")
        usage
        ;;
    *)
        echo "Unknown command: $1"
        usage
        exit 1
        ;;
esac
