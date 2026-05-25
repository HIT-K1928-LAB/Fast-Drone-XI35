#!/usr/bin/env python3
import argparse
from pathlib import Path

from ultralytics import YOLO


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Export an Ultralytics YOLO .pt model to ONNX.")
    parser.add_argument("--weights", default="code/best.pt", help="Path to the .pt weights file.")
    parser.add_argument("--imgsz", type=int, default=640, help="Square input image size used for export.")
    parser.add_argument("--opset", type=int, default=None, help="Optional ONNX opset version.")
    parser.add_argument("--dynamic", action="store_true", help="Export with dynamic input shapes.")
    parser.add_argument("--simplify", action="store_true", help="Simplify the ONNX graph during export.")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    weights_path = Path(args.weights).expanduser().resolve()
    if not weights_path.is_file():
        raise FileNotFoundError(f"weights file not found: {weights_path}")

    model = YOLO(str(weights_path))
    export_args = {
        "format": "onnx",
        "imgsz": args.imgsz,
        "dynamic": args.dynamic,
        "simplify": args.simplify,
    }
    if args.opset is not None:
        export_args["opset"] = args.opset

    output_path = model.export(**export_args)
    print(f"ONNX exported to: {output_path}")


if __name__ == "__main__":
    main()
