# Livox laser simulation plugin

The files in this directory are vendored from
[`Livox-SDK/livox_laser_simulation`](https://github.com/Livox-SDK/livox_laser_simulation)
at commit `1cce1073633a062b92e30243a4c2920e45551bb5`.

Local compatibility change:

- Gazebo 11 uses the unversioned `ignition/math.hh` include path
  instead of the upstream Gazebo 9 `ignition/math4/ignition/math.hh` path.
- Pure SDF keeps the scan CSV as a `package://` URI, so the plugin resolves
  that URI through `ros::package` before opening the file.

The upstream files remain covered by the accompanying MIT `LICENSE`.
