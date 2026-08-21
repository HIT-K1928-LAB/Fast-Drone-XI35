#!/usr/bin/env bash

set -uo pipefail
export LC_ALL=C

INTERVAL_SECONDS="${1:-5}"
CONTAINER_NAME="${CONTAINER_NAME:-fd_runtime_rk3588}"

if ! awk -v value="$INTERVAL_SECONDS" 'BEGIN { exit !(value > 0) }'; then
    echo "Usage: $0 [interval_seconds] [output.csv]" >&2
    exit 2
fi

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
WORKSPACE=$(cd "$SCRIPT_DIR/.." && pwd)
RUN_ID=$(date +%Y%m%d_%H%M%S)
DEFAULT_OUTPUT="$WORKSPACE/src/localization/FAST-LIVO2/Log/monitor/system_${RUN_ID}.csv"
OUTPUT_CSV="${2:-$DEFAULT_OUTPUT}"
mkdir -p "$(dirname "$OUTPUT_CSV")"

read_cpu_counters() {
    local label user nice system idle iowait irq softirq steal rest
    read -r label user nice system idle iowait irq softirq steal rest < /proc/stat
    CPU_TOTAL=$((user + nice + system + idle + iowait + irq + softirq + steal))
    CPU_IDLE=$((idle + iowait))
    CPU_IOWAIT=$iowait
}

read_file_counter() {
    local path="$1"
    if [[ -r "$path" ]]; then
        cat "$path" 2>/dev/null || echo 0
    else
        echo 0
    fi
}

read_net_counter() {
    local interface="$1"
    local counter="$2"
    read_file_counter "/sys/class/net/$interface/statistics/$counter"
}

read_psi_avg10() {
    local path="$1"
    local value
    value=$(awk '
        /^some / {
            for (i = 1; i <= NF; ++i) {
                if ($i ~ /^avg10=/) {
                    split($i, value, "=")
                    print value[2]
                    found = 1
                    exit
                }
            }
        }
        END { if (!found) print 0 }
    ' "$path" 2>/dev/null || true)
    echo "${value:-0}"
}

refresh_container_cgroup() {
    local container_id
    container_id=$(docker inspect -f '{{.Id}}' "$CONTAINER_NAME" 2>/dev/null || true)
    CONTAINER_MEMORY_FILE="/sys/fs/cgroup/memory/docker/$container_id/memory.usage_in_bytes"
    CONTAINER_CPU_FILE="/sys/fs/cgroup/cpu,cpuacct/docker/$container_id/cpuacct.usage"
}

WIFI_INTERFACE=$(ip route show default 2>/dev/null | awk 'NR == 1 { print $5 }')
WIFI_INTERFACE="${WIFI_INTERFACE:-rename5}"
LIDAR_INTERFACE="${LIDAR_INTERFACE:-eth0}"
DISK_DEVICE="${DISK_DEVICE:-mmcblk0}"

refresh_container_cgroup
read_cpu_counters
previous_cpu_total=$CPU_TOTAL
previous_cpu_idle=$CPU_IDLE
previous_cpu_iowait=$CPU_IOWAIT
previous_container_cpu=$(read_file_counter "$CONTAINER_CPU_FILE")
previous_disk_sectors=$(awk -v device="$DISK_DEVICE" '$3 == device { print $10; found = 1 } END { if (!found) print 0 }' /proc/diskstats)
previous_lidar_rx=$(read_net_counter "$LIDAR_INTERFACE" rx_bytes)
previous_wifi_tx=$(read_net_counter "$WIFI_INTERFACE" tx_bytes)
previous_epoch=$(date +%s)
start_epoch=$previous_epoch

echo "timestamp,elapsed_s,load1,cpu_pct,iowait_pct,mem_used_mib,mem_available_mib,swap_used_mib,container_mem_mib,container_cpu_pct,fastlivo_rss_mib,fastlivo_cpu_pct,ros_rss_mib,temp_soc_c,temp_max_c,disk_used_pct,disk_write_mib_s,lidar_rx_mib_s,wifi_tx_mib_s,psi_mem_avg10,psi_io_avg10,oom_kill,major_faults" > "$OUTPUT_CSV"

stop_monitor() {
    echo
    echo "Monitor stopped. CSV: $OUTPUT_CSV"
    exit 0
}
trap stop_monitor INT TERM HUP

echo "Monitoring RK3588 every ${INTERVAL_SECONDS}s"
echo "Container: $CONTAINER_NAME"
echo "CSV: $OUTPUT_CSV"

while true; do
    sleep "$INTERVAL_SECONDS"

    now_epoch=$(date +%s)
    delta_seconds=$((now_epoch - previous_epoch))
    ((delta_seconds > 0)) || delta_seconds=1
    timestamp=$(date --iso-8601=seconds)
    elapsed_seconds=$((now_epoch - start_epoch))

    read_cpu_counters
    delta_cpu_total=$((CPU_TOTAL - previous_cpu_total))
    delta_cpu_idle=$((CPU_IDLE - previous_cpu_idle))
    delta_cpu_iowait=$((CPU_IOWAIT - previous_cpu_iowait))
    cpu_pct=$(awk -v total="$delta_cpu_total" -v idle="$delta_cpu_idle" 'BEGIN { if (total > 0) printf "%.1f", 100 * (total - idle) / total; else print "0.0" }')
    iowait_pct=$(awk -v total="$delta_cpu_total" -v wait="$delta_cpu_iowait" 'BEGIN { if (total > 0) printf "%.1f", 100 * wait / total; else print "0.0" }')
    load1=$(awk '{ print $1 }' /proc/loadavg)

    read -r mem_total_kib mem_available_kib swap_total_kib swap_free_kib < <(
        awk '
            /^MemTotal:/ { total = $2 }
            /^MemAvailable:/ { available = $2 }
            /^SwapTotal:/ { swap_total = $2 }
            /^SwapFree:/ { swap_free = $2 }
            END { print total + 0, available + 0, swap_total + 0, swap_free + 0 }
        ' /proc/meminfo
    )
    mem_used_mib=$(awk -v total="$mem_total_kib" -v available="$mem_available_kib" 'BEGIN { printf "%.1f", (total - available) / 1024 }')
    mem_available_mib=$(awk -v value="$mem_available_kib" 'BEGIN { printf "%.1f", value / 1024 }')
    swap_used_mib=$(awk -v total="$swap_total_kib" -v free="$swap_free_kib" 'BEGIN { printf "%.1f", (total - free) / 1024 }')

    if [[ ! -r "$CONTAINER_MEMORY_FILE" || ! -r "$CONTAINER_CPU_FILE" ]]; then
        refresh_container_cgroup
    fi
    container_memory=$(read_file_counter "$CONTAINER_MEMORY_FILE")
    container_cpu=$(read_file_counter "$CONTAINER_CPU_FILE")
    container_mem_mib=$(awk -v value="$container_memory" 'BEGIN { printf "%.1f", value / 1048576 }')
    container_cpu_pct=$(awk -v current="$container_cpu" -v previous="$previous_container_cpu" -v seconds="$delta_seconds" 'BEGIN { if (current >= previous && seconds > 0) printf "%.1f", (current - previous) / seconds / 10000000; else print "0.0" }')

    read -r fastlivo_rss_kib fastlivo_cpu_pct ros_rss_kib < <(
        ps -eo rss=,pcpu=,args= | awk '
            /fastlivo_mapping/ && $0 !~ /awk/ { fast_rss += $1; fast_cpu += $2 }
            /fastlivo_mapping|livox_ros_driver2_node|hikrobot_mvs_node.py|rosmaster|roscore/ && $0 !~ /awk/ { ros_rss += $1 }
            END { printf "%d %.1f %d\n", fast_rss + 0, fast_cpu + 0, ros_rss + 0 }
        '
    )
    fastlivo_rss_mib=$(awk -v value="$fastlivo_rss_kib" 'BEGIN { printf "%.1f", value / 1024 }')
    ros_rss_mib=$(awk -v value="$ros_rss_kib" 'BEGIN { printf "%.1f", value / 1024 }')

    temp_soc_millic=0
    temp_max_millic=0
    for zone in /sys/class/thermal/thermal_zone*; do
        [[ -r "$zone/temp" ]] || continue
        zone_temp=$(cat "$zone/temp" 2>/dev/null || echo 0)
        zone_type=$(cat "$zone/type" 2>/dev/null || true)
        ((zone_temp > temp_max_millic)) && temp_max_millic=$zone_temp
        [[ "$zone_type" == "soc-thermal" ]] && temp_soc_millic=$zone_temp
    done
    temp_soc_c=$(awk -v value="$temp_soc_millic" 'BEGIN { printf "%.1f", value / 1000 }')
    temp_max_c=$(awk -v value="$temp_max_millic" 'BEGIN { printf "%.1f", value / 1000 }')

    disk_used_pct=$(df -P /userdata 2>/dev/null | awk 'NR == 2 { gsub(/%/, "", $5); print $5 + 0 }')
    disk_used_pct="${disk_used_pct:-0}"
    disk_sectors=$(awk -v device="$DISK_DEVICE" '$3 == device { print $10; found = 1 } END { if (!found) print 0 }' /proc/diskstats)
    lidar_rx=$(read_net_counter "$LIDAR_INTERFACE" rx_bytes)
    wifi_tx=$(read_net_counter "$WIFI_INTERFACE" tx_bytes)
    disk_write_mib_s=$(awk -v current="$disk_sectors" -v previous="$previous_disk_sectors" -v seconds="$delta_seconds" 'BEGIN { printf "%.2f", (current - previous) * 512 / seconds / 1048576 }')
    lidar_rx_mib_s=$(awk -v current="$lidar_rx" -v previous="$previous_lidar_rx" -v seconds="$delta_seconds" 'BEGIN { printf "%.2f", (current - previous) / seconds / 1048576 }')
    wifi_tx_mib_s=$(awk -v current="$wifi_tx" -v previous="$previous_wifi_tx" -v seconds="$delta_seconds" 'BEGIN { printf "%.2f", (current - previous) / seconds / 1048576 }')

    psi_mem_avg10=$(read_psi_avg10 /proc/pressure/memory)
    psi_io_avg10=$(read_psi_avg10 /proc/pressure/io)
    oom_kill=$(awk '$1 == "oom_kill" { print $2; found = 1 } END { if (!found) print 0 }' /proc/vmstat)
    major_faults=$(awk '$1 == "pgmajfault" { print $2; found = 1 } END { if (!found) print 0 }' /proc/vmstat)

    printf '%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s,%s\n' \
        "$timestamp" "$elapsed_seconds" "$load1" "$cpu_pct" "$iowait_pct" \
        "$mem_used_mib" "$mem_available_mib" "$swap_used_mib" \
        "$container_mem_mib" "$container_cpu_pct" "$fastlivo_rss_mib" "$fastlivo_cpu_pct" "$ros_rss_mib" \
        "$temp_soc_c" "$temp_max_c" "$disk_used_pct" "$disk_write_mib_s" \
        "$lidar_rx_mib_s" "$wifi_tx_mib_s" "$psi_mem_avg10" "$psi_io_avg10" "$oom_kill" "$major_faults" \
        >> "$OUTPUT_CSV"

    printf '[%s] load=%s cpu=%s%% mem=%sMiB container=%sMiB fastlivo=%sMiB temp=%sC disk_write=%sMiB/s oom=%s\n' \
        "$timestamp" "$load1" "$cpu_pct" "$mem_used_mib" "$container_mem_mib" \
        "$fastlivo_rss_mib" "$temp_max_c" "$disk_write_mib_s" "$oom_kill"

    previous_cpu_total=$CPU_TOTAL
    previous_cpu_idle=$CPU_IDLE
    previous_cpu_iowait=$CPU_IOWAIT
    previous_container_cpu=$container_cpu
    previous_disk_sectors=$disk_sectors
    previous_lidar_rx=$lidar_rx
    previous_wifi_tx=$wifi_tx
    previous_epoch=$now_epoch
done
