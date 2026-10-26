#!/usr/bin/env python3
"""Capture paired RealSense depth frames with the projector off and on (ROS Noetic)."""
import argparse
from datetime import datetime
from itertools import count
import json
import time
from pathlib import Path

import numpy as np
import rospy
from dynamic_reconfigure.client import Client
from sensor_msgs.msg import Image


def depth_array(msg):
    if msg.encoding in ('16UC1', 'mono16'):
        dtype = np.dtype('u2')
    elif msg.encoding == '32FC1':
        dtype = np.dtype('f4')
    else:
        raise RuntimeError('Unsupported depth encoding: ' + msg.encoding)
    dtype = dtype.newbyteorder('>' if msg.is_bigendian else '<')
    if msg.step < msg.width * dtype.itemsize or len(msg.data) < msg.step * msg.height:
        raise RuntimeError('Invalid depth image buffer or row stride')
    rows = np.ndarray((msg.height, msg.width), dtype=dtype,
                      buffer=msg.data, strides=(msg.step, dtype.itemsize))
    return np.array(rows, copy=True)


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--topic', default='/camera/depth/image_rect_raw')
    p.add_argument('--server', default='/camera/stereo_module')
    p.add_argument('--output', default='laser_depth_capture')
    p.add_argument('--frames', type=int, default=10)
    p.add_argument('--settle', type=float, default=3.0)
    p.add_argument('--laser-power', type=float, default=None,
                   help='Optional explicit power when enabling; otherwise preserve existing power')
    args = p.parse_args()
    if args.frames < 1 or args.settle < 0:
        p.error('frames must be >= 1 and settle must be >= 0')
    rospy.init_node('capture_laser_depth', anonymous=True)
    client = Client(args.server, timeout=10)
    original = client.get_configuration(timeout=10)
    if 'emitter_enabled' not in original:
        raise RuntimeError('Dynamic reconfigure server has no emitter_enabled parameter')
    print('Original emitter_enabled:', original['emitter_enabled'])
    print('Original laser_power:', original.get('laser_power', 'unavailable'))
    print('Keep aircraft stationary and propellers disabled. Press Enter to start; Ctrl-C cancels.')
    input()
    base = Path(args.output).expanduser().resolve()
    for number in count(1):
        root = base.parent / ('%s_%d' % (base.name, number))
        try:
            root.mkdir(parents=True, exist_ok=False)
            break
        except FileExistsError:
            continue
    print('New experiment directory:', root)
    manifest = {'created_at': datetime.now().astimezone().isoformat(),
                'output_dir': str(root), 'topic': args.topic, 'server': args.server, 'original_config':
                {k: original.get(k) for k in ('emitter_enabled', 'laser_power')}, 'conditions': {}}
    try:
        for label, enabled in (('laser_off', 0), ('laser_on', 1)):
            updates = {'emitter_enabled': enabled}
            if enabled and args.laser_power is not None:
                updates['laser_power'] = args.laser_power
            actual = client.update_configuration(updates)
            if int(actual['emitter_enabled']) != enabled:
                raise RuntimeError('Camera rejected emitter_enabled=%d' % enabled)
            print('%s: emitter=%s power=%s; settling %.1fs' %
                  (label, actual['emitter_enabled'], actual.get('laser_power'), args.settle))
            time.sleep(args.settle)
            folder = root / label
            folder.mkdir(exist_ok=True)
            records = []
            last_stamp = None
            for i in range(args.frames):
                deadline = time.monotonic() + 15
                while True:
                    msg = rospy.wait_for_message(args.topic, Image, timeout=max(0.1, deadline-time.monotonic()))
                    stamp = msg.header.stamp.to_nsec()
                    if stamp != last_stamp:
                        break
                    if time.monotonic() >= deadline:
                        raise RuntimeError('No new depth frame on ' + args.topic)
                last_stamp = stamp
                arr = depth_array(msg)
                name = 'depth_%03d' % i
                np.save(str(folder / (name + '.npy')), arr)
                records.append({'name': name, 'stamp_ns': stamp, 'encoding': msg.encoding,
                                'shape': list(arr.shape), 'dtype': str(arr.dtype)})
                print(label, i + 1, '/', args.frames, 'valid:', int(np.count_nonzero(np.isfinite(arr) & (arr > 0))))
            manifest['conditions'][label] = {'config': {k: actual.get(k) for k in
                ('emitter_enabled', 'laser_power')}, 'frames': records}
            (root / 'manifest.json').write_text(json.dumps(manifest, indent=2), encoding='utf-8')
    finally:
        restore = {'emitter_enabled': original['emitter_enabled']}
        if 'laser_power' in original:
            restore['laser_power'] = original['laser_power']
        try:
            client.update_configuration(restore)
            print('Camera settings restored:', restore)
        except Exception as exc:
            print('WARNING: camera settings could not be restored:', exc)
    print('Saved to', root)


if __name__ == '__main__':
    main()
