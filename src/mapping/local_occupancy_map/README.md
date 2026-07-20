# local_occupancy_map

Depth-based local 3D occupancy mapping for planner visualization and safety checks.

The implementation follows the EGO-Planner mapping flow:

```text
depth image + odometry
  -> project depth pixels to world points
  -> raycast free/occupied voxels
  -> update log-odds occupancy buffer
  -> inflate occupied voxels
  -> publish PointCloud2 for RViz
```

## Topics

Input:

```text
~depth  sensor_msgs/Image
~odom   nav_msgs/Odometry
```

Output:

```text
~occupancy          sensor_msgs/PointCloud2
~occupancy_inflate  sensor_msgs/PointCloud2
```

## Run

Simulation:

```bash
roslaunch local_occupancy_map local_occupancy_map.launch \
  depth_topic:=/pcl_render_node/depth \
  odom_topic:=/sim/odom \
  rviz:=true
```

YOPO simulation starts this node by default through:

```bash
roslaunch yopo_planner yopo_sim.launch
```

Disable it with:

```bash
roslaunch yopo_planner yopo_sim.launch local_occupancy_map:=false
```

For real flight, remap `depth_topic` and `odom_topic` to the camera depth image and
VIO/LIO odometry, then update camera intrinsics in `config/local_occupancy_map.yaml`.
