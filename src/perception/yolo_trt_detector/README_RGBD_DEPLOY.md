# RGB-D YOLO detector on Orin NX

This is a copy of the drone's `yolo_trt_detector` ROS 1 package, adapted only for the
exported five-channel RGB-D YOLO model. The old model files are intentionally not
included. Keep the verified `models/best_rgbd.onnx` and Orin-built
`models/best_rgbd.engine` in the active drone package.

## Model contract

- Input `images`: float32, `[1, 5, 480, 640]`, channel order
  `[R, G, B, linear_depth, depth_validity]`.
- Output `output0`: float32, `[1, 5, 6300]`, without embedded NMS.
- The depth image must be a `16UC1`/`mono16` image already aligned to the color
  image, with the **same raw depth units as the training images**. Inspect a few
  actual depth values before relying on detections. This node rejects `32FC1`.
- Raw depth values `0` and `65535` are invalid. Valid depth is clipped to
  `[2000, 30000]`, linearly scaled to `[0, 255]`, truncated to `uint8`, then
  divided by 255. The mask is 0/1 after tensor normalization.
- Color and depth are approximately synchronized; the maximum timestamp gap
  defaults to 0.05 seconds. All five channels receive the same geometry and
  zero padding. For native 640x480 frames, no resize or padding is needed.

## On the drone

The existing container maps the host's
`/home/orin10/code/Fast-Drone-XI35` to `/root/Fast-Drone-XI35`. On the current
drone, the established ROS package location is
`src/perception/yolo_trt_detector`. Back up that old YOLO package, then merge
the new source, launch files and dependencies into that same package while
preserving its existing `models/best_rgbd.onnx` and `models/best_rgbd.engine`.
Do not replace other packages under `src/perception`. A temporary copy of the
new package also exists under `src/realflight_modules/yolo_trt_detector` on
the drone; add `CATKIN_IGNORE` to **that temporary copy only** after merging,
so ROS and catkin select the established `src/perception` location. Check with
`rospack profile` and `rospack find yolo_trt_detector` before building. Do not
delete the workspace's `build` or `devel` directories. The copied legacy
`scripts/export_yolo_to_onnx.py` is not needed here: the five-channel ONNX
model has already been exported.

If the engine needs rebuilding later, in the existing container:

```bash
cd /root/Fast-Drone-XI35/src/perception/yolo_trt_detector
sha256sum models/best_rgbd.onnx
bash scripts/yolo_to_tensorrt.sh
```

The script builds `models/best_rgbd.engine` on the Orin NX with the
installed `trtexec`. Verify that its output reports `images` and `output0`, and
keep the ONNX hash and build log. The node checks the actual engine tensor names,
data types, and shapes before accepting it. It supports TensorRT 8 and 10 APIs.
The launch files already use `models/best_rgbd.engine`. If it was built
successfully on this Orin, do not rebuild solely to rename it. Check that it
exists under the active `src/perception` package.

The drone's existing build space was created with `catkin build`, so keep using
that tool; do not run `catkin_make` in the same build space. Once the package
path has been verified, compile just the detector package:

```bash
cd /root/Fast-Drone-XI35
source devel/setup.bash
catkin build yolo_trt_detector --no-deps
source devel/setup.bash
```

Inspect the camera topics before launch:

```bash
rostopic list | grep -E 'camera|yolo_trt'
rostopic hz /camera/color/image_raw
rostopic hz /camera/aligned_depth_to_color/image_raw
```

The existing `rs_camera.launch` defaults to `align_depth:=false` and
`enable_sync:=false`. The detector requires an aligned depth stream. If the
camera is already running with alignment, use the test launch below. If no
camera driver is running, `yolo_rgbd_camera_test.launch` starts the existing
D455 driver with alignment and sync enabled alongside the detector. Start
only one D455 driver for the physical camera. Neither launch edits the
existing camera or flight scripts.

For a ground-only recognition test, with the existing ROS master and D455
driver already running, first ensure no old YOLO detector node is running
(`rosnode list | grep yolo_trt`). Stop only that old node if necessary:

```bash
roslaunch yolo_trt_detector yolo_rgbd_test.launch
```

When no D455 driver is running, use this alternative instead:

```bash
roslaunch yolo_trt_detector yolo_rgbd_camera_test.launch
```

The test launch publishes `/yolo_trt/annotated_image` and
`/yolo_trt/markers`, but does not advertise or publish the target point used
by the mission system. It does not start the flight controller. The regular
`yolo_trt_detector.launch` retains target-point publishing for later
integration after that output is checked separately.

The original flight bring-up script starts `rs_camera.launch` with its default
alignment disabled. Running the detector as part of that bring-up later
requires launching the existing camera driver with `align_depth:=true` and
`enable_sync:=true`; otherwise the aligned-depth topic is absent. This cannot
be fixed by changing the detector's topic name to the unaligned raw depth.

On the Windows computer, use Foxglove Desktop's native ROS 1 connection to
the drone's ROS master and view `/yolo_trt/annotated_image` in an Image panel.

Compare a few synchronized RGB/depth pairs against the PyTorch or ONNX result
from the original training repo before using detections for any flight task.

## LiDAR-primary target tracking

The `orin-lidar-01` flight profile enables a LiDAR-primary tracking path. Its
data flow is:

1. FAST-LIVO publishes one complete motion-compensated scan on
   `/cloud_undistorted_body_world`. The points use the same FCU-aligned local
   world convention as the flight odometry.
2. The detector builds a persistent voxel background, removes static cells,
   and groups the remaining points with 26-connected voxel clustering.
3. A YOLO box is projected through the calibrated camera transform and only
   selects/confirms a LiDAR cluster. D455 depth is not used as a target-position
   measurement in this mode.
4. A two-model IMM estimates position, velocity and covariance from later
   LiDAR clusters. Therefore `/yolo_trt/tracked_target` continues while YOLO
   is temporarily absent. If LiDAR itself is lost or covariance grows past its
   limit, identity is cleared and YOLO must confirm a cluster again.

FAST-LIVO controls the dense stream independently of its normal RViz/map
publication:

```yaml
publish:
  dense_undistorted_body_world_en: true
```

When this option is false—or when the topic has no subscriber—FAST-LIVO skips
the additional cloud transformation and copy. The existing
`/cloud_registered`, `/cloud_registered_body_world`, mapping, odometry and PCD
saving paths are unchanged.

Before a propeller-on test, verify the chain on the ground:

```bash
rostopic hz /cloud_undistorted_body_world
rostopic echo -n 1 /cloud_undistorted_body_world/header
rostopic echo /yolo_trt/target_status
rostopic echo /yolo_trt/tracked_target
```

Expected behavior is `LOST` before YOLO identifies a cluster, then
`CONFIRMED`. Covering the camera should not stop `tracked_target` while the
Mid-360 still observes the same object. Moving the object outside the LiDAR
gate for longer than `lidar_lost_time` should return the status to `LOST`.
