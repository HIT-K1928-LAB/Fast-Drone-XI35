#!/usr/bin/env bash
set -euo pipefail

TRTEXEC_BIN="${TRTEXEC_BIN:-/usr/src/tensorrt/bin/trtexec}"
ONNX_MODEL="${1:-models/best_rgbd.onnx}"
ENGINE_MODEL="${2:-models/best_rgbd.engine}"

if [[ ! -f "${ONNX_MODEL}" ]]; then
  echo "ONNX file not found: ${ONNX_MODEL}" >&2
  exit 1
fi
if [[ ! -x "${TRTEXEC_BIN}" ]]; then
  echo "trtexec not executable: ${TRTEXEC_BIN}" >&2
  exit 1
fi

TRT_HELP="$("${TRTEXEC_BIN}" --help 2>&1 || true)"
if grep -q -- '--skipInference' <<< "${TRT_HELP}"; then
  BUILD_FLAG=--skipInference
else
  BUILD_FLAG=--buildOnly
fi

"${TRTEXEC_BIN}" \
  --onnx="${ONNX_MODEL}" \
  --saveEngine="${ENGINE_MODEL}" \
  --fp16 \
  --inputIOFormats=fp32:chw \
  --outputIOFormats=fp32:chw \
  --avgTiming=1 \
  "${BUILD_FLAG}"
