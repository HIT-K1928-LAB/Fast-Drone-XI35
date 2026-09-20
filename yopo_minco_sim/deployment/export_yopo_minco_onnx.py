#!/usr/bin/env python3
"""Export official YOPO-MINCO network weights, not a solved MINCO trajectory.

Exports raw forward(depth, obs): endstate, score, radius. Preprocessing,
decoding, corridor admission and MINCO solving remain outside the ONNX graph.
"""
import argparse
import hashlib
import inspect
import json
import os
from pathlib import Path
import sys
import tempfile


def sha256(path):
    h = hashlib.sha256()
    with open(path, "rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            h.update(block)
    return h.hexdigest()


def parse_args():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--yopo-root", required=True, type=Path,
                   help="Official YOPO-MINCO directory containing config/ and policy/")
    p.add_argument("--weight", required=True, type=Path)
    p.add_argument("--output", required=True, type=Path)
    p.add_argument("--opset", type=int, default=13)
    p.add_argument("--device", choices=("cpu", "cuda"), default="cpu")
    p.add_argument("--verify", action="store_true", help="Compare all outputs using ONNX Runtime CPU")
    p.add_argument("--cases", type=int, default=3, help="Synthetic validation input count")
    p.add_argument("--atol", type=float, default=1e-5)
    p.add_argument("--rtol", type=float, default=1e-4)
    p.add_argument("--dry-run", action="store_true", help="Load weights and check PyTorch outputs only")
    p.add_argument("--force", action="store_true")
    return p.parse_args()


def main():
    args = parse_args()
    if args.cases < 1 or args.atol < 0 or args.rtol < 0:
        raise ValueError("Invalid validation parameters")
    root, weight, output = (p.expanduser().resolve() for p in
                            (args.yopo_root, args.weight, args.output))
    if not (root / "policy/yopo_network.py").is_file() or not weight.is_file():
        raise FileNotFoundError("Check --yopo-root and --weight")
    sidecar = Path(str(output) + ".json")
    samples = Path(str(output) + ".validation.npz")
    for p in (output, sidecar, samples):
        if p.exists() and not args.force and not args.dry_run:
            raise FileExistsError(str(p) + " exists; choose another output or use --force")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    import numpy as np
    import torch
    sys.path.insert(0, str(root))
    from config.config import cfg
    cfg["train"] = False
    from policy.yopo_network import YopoNetwork
    if args.device == "cuda" and not torch.cuda.is_available():
        raise RuntimeError("CUDA requested but unavailable")
    if "weights_only" not in inspect.signature(torch.load).parameters:
        raise RuntimeError("Use the verified server PyTorch environment supporting weights_only=True")
    model = YopoNetwork()
    state = torch.load(str(weight), map_location="cpu", weights_only=True)
    model.load_state_dict(state, strict=True)
    model = model.to(args.device).eval()
    names = ("endstate", "score", "radius")
    v, h = int(cfg["vertical_num"]), int(cfg["horizon_num"])
    nr = int(cfg["radius_num"])
    shapes = {
        "depth": (1, 1, int(cfg["image_height"]), int(cfg["image_width"])),
        "obs": (1, 9, v, h), "endstate": (1, 14, v, h),
        "score": (1, v, h), "radius": (1, 2 * nr, v, h),
    }
    rng = np.random.RandomState(2026)
    cases = []
    with torch.inference_mode():
        for i in range(args.cases):
            # Synthetic already-preprocessed inputs; no raw meter depth or raw 9-vector.
            depth = rng.uniform(0.05, 1.0, shapes["depth"]).astype(np.float32)
            obs = rng.uniform(-0.5, 0.5, shapes["obs"]).astype(np.float32)
            if i == 0:
                depth.fill(1.0)
                obs.fill(0.0)
            inputs = (torch.from_numpy(depth).to(args.device), torch.from_numpy(obs).to(args.device))
            result = model(*inputs)
            if not isinstance(result, (tuple, list)) or len(result) != 3:
                raise ValueError("Expected three MINCO outputs; check source checkout")
            case = {"depth": depth, "obs": obs}
            for name, tensor in zip(names, result):
                a = tensor.detach().cpu().numpy()
                if a.shape != shapes[name] or not np.isfinite(a).all():
                    raise ValueError("Invalid output %s: %s" % (name, a.shape))
                case[name] = a
            cases.append(case)
        print(json.dumps({"shapes": shapes, "torch": torch.__version__}, indent=2))
        if args.dry_run:
            print("PyTorch checkpoint/forward checks passed. No ONNX file written.")
            return
        import onnx
        ort = None
        if args.verify:
            import onnxruntime as ort
        output.parent.mkdir(parents=True, exist_ok=True)
        fd, temp_name = tempfile.mkstemp(suffix=".onnx", dir=str(output.parent))
        os.close(fd)
        try:
            export_kw = dict(input_names=["depth", "obs"], output_names=list(names),
                             opset_version=args.opset, do_constant_folding=True)
            if "dynamo" in inspect.signature(torch.onnx.export).parameters:
                export_kw["dynamo"] = False  # legacy exporter, supported by server Torch 2.4
            first_inputs = tuple(torch.from_numpy(cases[0][n]).to(args.device) for n in ("depth", "obs"))
            torch.onnx.export(model, first_inputs, temp_name, **export_kw)
            graph = onnx.load(temp_name)
            onnx.checker.check_model(graph)
            for tensors, expected_names in ((graph.graph.input, ["depth", "obs"]),
                                             (graph.graph.output, list(names))):
                if [t.name for t in tensors] != expected_names:
                    raise ValueError("Unexpected ONNX input/output names")
                for t in tensors:
                    dims = tuple(d.dim_value for d in t.type.tensor_type.shape.dim)
                    if dims != shapes[t.name]:
                        raise ValueError("Unexpected ONNX shape for " + t.name)
            metrics = []
            if ort is not None:
                session = ort.InferenceSession(temp_name, providers=["CPUExecutionProvider"])
                for i, case in enumerate(cases):
                    results = session.run(list(names), {n: case[n] for n in ("depth", "obs")})
                    for name, actual in zip(names, results):
                        expected = case[name]
                        if not np.isfinite(actual).all():
                            raise ValueError("Non-finite ORT output")
                        np.testing.assert_allclose(actual, expected, rtol=args.rtol, atol=args.atol,
                                                   err_msg="case %d / %s" % (i, name))
                        metrics.append(dict(case=i, output=name,
                                            max_abs_error=float(np.max(np.abs(actual - expected)))))
                print("ONNX Runtime comparison passed for all three outputs.")
            meta = dict(format="yopo-minco-forward-v1", checkpoint_sha256=sha256(weight),
                        config_sha256=sha256(root / "config/traj_opt.yaml"),
                        onnx_sha256=sha256(temp_name), torch_version=torch.__version__,
                        onnx_version=onnx.__version__, opset=args.opset, shapes=shapes,
                        radius_num=nr, radius_warp_lambda=float(cfg["radius_warp_lambda"]),
                        ort_verified=args.verify, metrics=metrics,
                        note="Network only. Preserve matching preprocessing, decoder and MINCO solver.")
            np.savez_compressed(str(samples), **{
                "case%d_%s" % (i, name): value for i, case in enumerate(cases) for name, value in case.items()
            })
            sidecar.write_text(json.dumps(meta, indent=2) + "\n", encoding="utf-8")
            os.replace(temp_name, output)
        finally:
            if os.path.exists(temp_name):
                os.unlink(temp_name)
    print("Exported:", output)
    print("Metadata:", sidecar)
    print("Validation samples:", samples)
    if not args.verify:
        print("ONNX checker passed; numerical comparison was NOT run (use --verify).")


if __name__ == "__main__":
    main()
