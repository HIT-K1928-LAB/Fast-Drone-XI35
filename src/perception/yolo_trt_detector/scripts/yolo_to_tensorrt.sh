#!/usr/bin/env bash
set -euo pipefail

TRTEXEC_BIN="${TRTEXEC_BIN:-/usr/src/tensorrt/bin/trtexec}"
ONNX_MODEL="${1:-models/best_yolo11s_p2.onnx}"
ENGINE_MODEL="${2:-models/best_yolo11s_p2.engine}"

"${TRTEXEC_BIN}" \
  --onnx="${ONNX_MODEL}" \
  --saveEngine="${ENGINE_MODEL}" \
  --fp16 \
  --avgTiming=1 \
  --buildOnly
