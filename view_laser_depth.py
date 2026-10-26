#!/usr/bin/env python3
"""Click either captured depth image to inspect both conditions at the same pixel."""
import argparse
import json
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np


def main():
    p = argparse.ArgumentParser()
    p.add_argument('capture_dir', type=Path)
    p.add_argument('--frame', type=int, default=0)
    args = p.parse_args()
    manifest = json.loads((args.capture_dir / 'manifest.json').read_text(encoding='utf-8'))
    arrays = []
    labels = ('laser_off', 'laser_on')
    for label in labels:
        rec = manifest['conditions'][label]['frames'][args.frame]
        arr = np.load(str(args.capture_dir / label / (rec['name'] + '.npy')))
        scale = 0.001 if rec['encoding'] in ('16UC1', 'mono16') else 1.0
        arrays.append(arr.astype(float) * scale)
    if arrays[0].shape != arrays[1].shape:
        raise ValueError('Depth frame shapes differ')
    valid = np.concatenate([a[np.isfinite(a) & (a > 0)] for a in arrays])
    vmax = np.percentile(valid, 99) if valid.size else 5.0
    fig, axes = plt.subplots(1, 2, sharex=True, sharey=True)
    for ax, label, arr in zip(axes, labels, arrays):
        ax.imshow(np.ma.masked_where(~np.isfinite(arr) | (arr <= 0), arr),
                  cmap='turbo', vmin=0, vmax=vmax, interpolation='nearest')
        ax.set_title(label)
        ax.set_xlabel('x (pixel)')
    axes[0].set_ylabel('y (pixel)')
    marks = []
    def clicked(event):
        if event.inaxes not in axes or event.xdata is None or event.ydata is None:
            return
        x, y = round(event.xdata), round(event.ydata)
        if not (0 <= y < arrays[0].shape[0] and 0 <= x < arrays[0].shape[1]):
            return
        for mark in marks:
            mark.remove()
        marks.clear()
        vals = []
        for ax, arr in zip(axes, arrays):
            marks.append(ax.plot(x, y, 'wx', markersize=10, markeredgewidth=2)[0])
            v = arr[y, x]
            vals.append('invalid' if not np.isfinite(v) or v <= 0 else '%.3f m' % v)
        fig.suptitle('pixel (%d, %d): OFF %s | ON %s' % (x, y, vals[0], vals[1]))
        print('(%d, %d): OFF %s | ON %s' % (x, y, vals[0], vals[1]))
        fig.canvas.draw_idle()
    fig.canvas.mpl_connect('button_press_event', clicked)
    fig.colorbar(axes[1].images[0], ax=axes, label='depth (m)')
    plt.show()


if __name__ == '__main__':
    main()
