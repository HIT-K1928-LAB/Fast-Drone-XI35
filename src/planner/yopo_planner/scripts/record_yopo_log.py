#!/usr/bin/env python3

import os
import sys
from datetime import datetime


def main():
    args = [arg for arg in sys.argv[1:] if not arg.startswith("__")]
    log_dir = args[0] if len(args) >= 1 else os.path.join(os.path.expanduser("~"), "yopo_logs")
    bag_prefix = args[1] if len(args) >= 2 else "yopo_log"
    topics = args[2:]

    if not topics:
        print("record_yopo_log.py requires at least one topic", file=sys.stderr)
        return 1

    os.makedirs(log_dir, exist_ok=True)
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    bag_path = os.path.join(log_dir, f"{bag_prefix}_{stamp}.bag")

    os.execvp("rosbag", ["rosbag", "record", "-O", bag_path] + topics)
    return 1


if __name__ == "__main__":
    sys.exit(main())
