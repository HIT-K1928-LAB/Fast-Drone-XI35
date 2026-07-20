#!/usr/bin/env python3
import argparse
import os
import sys

import torch


def parse_args():
    parser = argparse.ArgumentParser(description="Export YOPO PyTorch checkpoint to ONNX.")
    parser.add_argument("--yopo-root", default="/root/YOPO/YOPO", help="YOPO source directory")
    parser.add_argument("--weight", default="/root/YOPO/YOPO/saved/YOPO_1/epoch50.pth", help="checkpoint path")
    parser.add_argument(
        "--output",
        default=os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "model", "yopo_epoch50.onnx"),
        help="output ONNX path",
    )
    parser.add_argument("--opset", type=int, default=13, help="ONNX opset version")
    parser.add_argument("--check", action="store_true", help="run onnx.checker after export")
    return parser.parse_args()


def main():
    args = parse_args()
    sys.path.insert(0, args.yopo_root)

    from config.config import cfg
    from policy.yopo_network import YopoNetwork

    cfg["train"] = False

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    try:
        state_dict = torch.load(args.weight, map_location=device, weights_only=True)
    except TypeError:
        state_dict = torch.load(args.weight, map_location=device)
    model = YopoNetwork()
    model.load_state_dict(state_dict)
    model.to(device)
    model.eval()

    depth = torch.zeros((1, 1, 96, 160), dtype=torch.float32, device=device)
    obs = torch.zeros((1, 9, cfg["vertical_num"], cfg["horizon_num"]), dtype=torch.float32, device=device)

    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with torch.inference_mode():
        torch.onnx.export(
            model,
            (depth, obs),
            args.output,
            input_names=["depth", "obs"],
            output_names=["endstate", "score"],
            opset_version=args.opset,
            do_constant_folding=True,
        )

    print(f"Exported YOPO ONNX: {args.output}")

    if args.check:
        import onnx

        onnx_model = onnx.load(args.output)
        onnx.checker.check_model(onnx_model)
        for tensor in onnx_model.graph.input:
            dims = [dim.dim_value for dim in tensor.type.tensor_type.shape.dim]
            print(f"input {tensor.name}: {dims}")
        for tensor in onnx_model.graph.output:
            dims = [dim.dim_value for dim in tensor.type.tensor_type.shape.dim]
            print(f"output {tensor.name}: {dims}")


if __name__ == "__main__":
    main()
