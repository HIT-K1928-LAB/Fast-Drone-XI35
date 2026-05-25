#!/usr/bin/env bash
set -euo pipefail

TRTEXEC_BIN="${TRTEXEC_BIN:-/usr/src/tensorrt/bin/trtexec}"
ONNX_MODEL="${1:-best_yolo11s_p2.pt.onnx}"
ENGINE_MODEL="${2:-best_yolo11s_p2.pt.engine}"

"${TRTEXEC_BIN}" \
  --onnx="${ONNX_MODEL}" \
  --saveEngine="${ENGINE_MODEL}" \
  --fp16 \
  --avgTiming=1 \
  --buildOnly
