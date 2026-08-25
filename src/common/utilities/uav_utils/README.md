# uav_utils ROS Package

Provide some widely used utilities in uav development. **No library. Only headers.**

Dependency: Eigen

## rosout_file_logger

Record `/rosout_agg` messages into per-node files under the `nodes`
subdirectory in the current ROS run directory:

```bash
rosrun uav_utils rosout_file_logger
```

or:

```bash
roslaunch uav_utils rosout_file_logger.launch
```

Each ROS node gets its own file in that directory, for example:

```text
nodes/vins_fusion.log
nodes/px4ctrl.log
```

The line format is:

```text
[INFO] [2026-06-04 14:40:58.826229647] [/vins_fusion] [rosNodeTest.cpp:main:197] waiting for image and imu...
```

The logger records all `/rosout_agg` levels after it starts: `DEBUG`, `INFO`,
`WARN`, `ERROR`, and `FATAL`.

The callback only pushes messages into an in-memory queue. A timer writes queued
messages to disk in batches to avoid flushing on every ROS log message. Runtime
parameters:

```bash
rosrun uav_utils rosout_file_logger _flush_period:=0.5 _max_queue_size:=10000 _max_batch_size:=1000 _flush_streams:=true _include_node_name:=true _include_location:=true _subdirectory:=nodes
roslaunch uav_utils rosout_file_logger.launch flush_period:=0.5 max_queue_size:=10000 max_batch_size:=1000 flush_streams:=true include_node_name:=true include_location:=true subdirectory:=nodes
```

- `flush_period`: seconds between batch writes.
- `max_queue_size`: maximum buffered log lines; oldest lines are dropped when full.
- `max_batch_size`: maximum lines written per timer tick.
- `flush_streams`: whether to flush file streams after each batch.
- `include_node_name`: whether each line includes the ROS node name, such as `[/vins_fusion]`.
- `include_location`: whether each line includes source location, such as `[rosNodeTest.cpp:main:197]`.
- `subdirectory`: one safe directory name under the current ROS run directory; defaults to `nodes`.

This package is deliberately designed not to find Eigen automatically, because there are many cases that you would like to use your own Eigen instead of that in the system directory (**/usr/include/eigen3**).

### How to resolve "fatal error: Eigen/Dense: No such file or directory" problem

1. The most simple and naive solution:

	Modify `your-ros-package/CMakeList.txt` like this:
	
	```cmake

	include_directories(
		...
		...
		/usr/include/eigen3
		...
	)
	```

2. Use **FindEigen.cmake** provided by ROS
	
	For details, see [https://github.com/ros/cmake_modules/blob/0.3-devel/README.md](https://github.com/ros/cmake_modules/blob/0.3-devel/README.md)

3. Specify your own Eigen
	
	* For recent releases of Eigen, cmake support is integrated. You may modify your `CMakeLists.txt` like this:

	``` cmake
	find_package(Eigen3 REQUIRED NO_DEFAULT_PATH PATHS /opt/eigen3.3/lib/x86_64-linux-gnu/cmake)
	include_directories(
  		...
  		${EIGEN3_INCLUDE_DIR}
  		${catkin_INCLUDE_DIRS}
  		...
	)

	```

	* For other third-party **FindEigen.cmake**, you have to read the cmake file to find correct **library-name** (Eigen, EIGEN, Eigen3, EIGEN3, etc.) and **include-path** (*_INCLUDE_DIRECTORIES, *_INCLUDE_DIRECTORY, *_INCLUDE_DIR, *_INCLUDE_DIRECTORIES, etc.)
