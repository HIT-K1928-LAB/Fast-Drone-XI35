#!/usr/bin/env python3
"""Build a static batch-1 TensorRT 8.x engine on the target Jetson. No ROS commands."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import platform
import tempfile


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--onnx", required=True, type=Path)
    p.add_argument("--engine", required=True, type=Path)
    p.add_argument("--precision", choices=("fp32", "fp16"), default="fp32")
    p.add_argument("--workspace-mib", type=int, default=1024)
    p.add_argument("--parse-only", action="store_true", help="Validate ONNX interface without engine build")
    p.add_argument("--force", action="store_true")
    args = p.parse_args()
    source, dest = args.onnx.expanduser().resolve(), args.engine.expanduser().resolve()
    if source == dest:
        raise ValueError("ONNX and engine paths must differ")
    if args.workspace_mib < 1:
        raise ValueError("--workspace-mib must be positive")
    meta_path = Path(str(dest) + ".json")
    for target in (dest, meta_path):
        if target.exists() and not args.force and not args.parse_only:
            raise FileExistsError(str(target) + " exists; use a new path or --force")
    import tensorrt as trt
    if trt.__version__.split('.')[0] != '8':
        raise RuntimeError("This script targets TensorRT 8.x; installed: " + trt.__version__)
    logger = trt.Logger(trt.Logger.INFO)
    trt.init_libnvinfer_plugins(logger, "")
    builder = trt.Builder(logger)
    network = builder.create_network(1 << int(trt.NetworkDefinitionCreationFlag.EXPLICIT_BATCH))
    parser = trt.OnnxParser(network, logger)
    data = source.read_bytes()
    if not parser.parse(data):
        raise RuntimeError("ONNX parse failed:\n" + "\n".join(str(parser.get_error(i))
                                                             for i in range(parser.num_errors)))
    expected = {"depth": (1, 1, 96, 160), "obs": (1, 9, 3, 5),
                "endstate": (1, 14, 3, 5), "score": (1, 3, 5), "radius": (1, 20, 3, 5)}
    tensors = ([network.get_input(i) for i in range(network.num_inputs)] +
               [network.get_output(i) for i in range(network.num_outputs)])
    if ({network.get_input(i).name for i in range(network.num_inputs)} != {"depth", "obs"}
            or {network.get_output(i).name for i in range(network.num_outputs)} != {"endstate", "score", "radius"}):
        raise ValueError("Wrong model interface; expected official MINCO with three outputs")
    for tensor in tensors:
        print(tensor.name, tuple(tensor.shape), tensor.dtype)
        if tuple(tensor.shape) != expected[tensor.name] or tensor.dtype != trt.float32:
            raise ValueError("Unexpected shape/dtype for " + tensor.name)
    if args.parse_only:
        print("Parser/interface checks passed. No engine built or numerical test run.")
        return
    config = builder.create_builder_config()
    config.set_memory_pool_limit(trt.MemoryPoolType.WORKSPACE, args.workspace_mib * 1024 * 1024)
    config.clear_flag(trt.BuilderFlag.TF32)  # reproducible FP32 baseline, not implicit TF32
    if args.precision == "fp16":
        if not builder.platform_has_fast_fp16:
            raise RuntimeError("Target does not report fast FP16 support")
        config.set_flag(trt.BuilderFlag.FP16)  # allows mixed precision; I/O remains FP32
    serialized = builder.build_serialized_network(network, config)
    if serialized is None:
        raise RuntimeError("TensorRT build failed; inspect preceding builder errors")
    blob = bytes(serialized)
    runtime = trt.Runtime(logger)
    engine = runtime.deserialize_cuda_engine(blob)
    if engine is None:
        raise RuntimeError("Built engine cannot be deserialized on target")
    bindings = []
    for i in range(engine.num_bindings):
        name = engine.get_binding_name(i)
        shape = tuple(engine.get_binding_shape(i))
        dtype = engine.get_binding_dtype(i)
        if name not in expected or shape != expected[name] or dtype != trt.float32:
            raise ValueError("Engine interface mismatch: " + name)
        bindings.append(dict(name=name, shape=shape, dtype=str(dtype),
                             is_input=engine.binding_is_input(i)))
    if len(bindings) != 5 or {b['name'] for b in bindings} != set(expected):
        raise ValueError("Engine binding count/names mismatch")
    dest.parent.mkdir(parents=True, exist_ok=True)
    fd, tmp = tempfile.mkstemp(dir=str(dest.parent), suffix=".engine")
    try:
        with os.fdopen(fd, "wb") as f:
            f.write(blob)
        os.replace(tmp, dest)
    finally:
        if os.path.exists(tmp):
            os.unlink(tmp)
    meta = dict(tensorrt=trt.__version__, architecture=platform.machine(),
                precision=args.precision, tf32=False, workspace_mib=args.workspace_mib,
                onnx_sha256=hashlib.sha256(data).hexdigest(),
                engine_sha256=hashlib.sha256(blob).hexdigest(), bindings=bindings,
                numerical_validation=False,
                note="Native TensorRT engine, not a torch2trt state_dict. Validate outputs before deployment.")
    meta_path.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
    print("Engine written:", dest)
    print("Build/deserialization passed; numerical accuracy and flight integration are NOT validated.")


if __name__ == "__main__":
    main()
