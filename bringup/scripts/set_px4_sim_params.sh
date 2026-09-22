#!/usr/bin/env bash

set -eu

NS="${1:-iris_0}"
PARAM_ID="${2:-COM_RCL_EXCEPT}"
PARAM_INT="${3:-4}"
TIMEOUT_S="${4:-90}"

echo "[set_px4_sim_params] Waiting for /${NS}/mavros/param/set ..."

for i in $(seq 1 "${TIMEOUT_S}"); do
    if rosservice call "/${NS}/mavros/param/set" \
        "{param_id: \"${PARAM_ID}\", value: {integer: ${PARAM_INT}, real: 0.0}}" \
        >/dev/null 2>&1; then
        echo "[set_px4_sim_params] OK: /${NS} ${PARAM_ID}=${PARAM_INT}"
        exit 0
    fi
    sleep 1
done

echo "[set_px4_sim_params] ERROR: MAVROS not ready within ${TIMEOUT_S}s, ${PARAM_ID} NOT set" >&2
exit 1
