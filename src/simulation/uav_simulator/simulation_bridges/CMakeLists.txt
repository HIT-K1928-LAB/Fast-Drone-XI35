cmake_minimum_required(VERSION 3.0.2)
project(simulation_bridges)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
  gazebo_msgs
  gazebo_ros
  livox_ros_driver2
  nav_msgs
  roscpp
  roslib
  sensor_msgs
  tf
)
find_package(gazebo REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES gazebo_ros_velodyne_gpu_laser livox_laser_simulation
  CATKIN_DEPENDS gazebo_msgs gazebo_ros livox_ros_driver2 nav_msgs roscpp roslib sensor_msgs tf
)

set(XTDRONE_VELODYNE_PLUGIN_DIR
  ${CMAKE_CURRENT_SOURCE_DIR}/../../xtdrone/sitl_config/gazebo_plugin/velodyne/velodyne_gazebo_plugins
)

include_directories(
  include
  third_party/livox_laser_simulation/include
  ${XTDRONE_VELODYNE_PLUGIN_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})

add_library(livox_laser_simulation SHARED
  third_party/livox_laser_simulation/src/livox_points_plugin.cpp
  third_party/livox_laser_simulation/src/livox_ode_multiray_shape.cpp
)
target_link_libraries(livox_laser_simulation
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  RayPlugin
)

# XTDrone keeps its Gazebo plugin tree behind CATKIN_IGNORE. Build only the
# GPU Velodyne target here so pc_sim can use the existing 3d_gpu_lidar model.
add_library(gazebo_ros_velodyne_gpu_laser SHARED
  ${XTDRONE_VELODYNE_PLUGIN_DIR}/src/GazeboRosVelodyneLaser.cpp
)
target_compile_definitions(gazebo_ros_velodyne_gpu_laser PRIVATE GAZEBO_GPU_RAY=1)
target_link_libraries(gazebo_ros_velodyne_gpu_laser
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  GpuRayPlugin
)

add_executable(gazebo_ground_truth_node src/gazebo_ground_truth_node.cpp)
add_dependencies(gazebo_ground_truth_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(gazebo_ground_truth_node ${catkin_LIBRARIES})

add_executable(livox_pointcloud_bridge_node src/livox_pointcloud_bridge_node.cpp)
add_dependencies(livox_pointcloud_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(livox_pointcloud_bridge_node ${catkin_LIBRARIES})

add_executable(velodyne_pointcloud_bridge_node
  src/velodyne_pointcloud_bridge_node.cpp
)
add_dependencies(velodyne_pointcloud_bridge_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(velodyne_pointcloud_bridge_node ${catkin_LIBRARIES})

if(CATKIN_ENABLE_TESTING)
  add_executable(simulation_bridges_converters_test test/converters_test.cpp)
  target_include_directories(simulation_bridges_converters_test PRIVATE include)
  add_test(NAME simulation_bridges_converters_test COMMAND simulation_bridges_converters_test)

  add_executable(simulation_bridges_resource_path_test test/resource_path_test.cpp)
  target_include_directories(simulation_bridges_resource_path_test PRIVATE include)
  add_test(NAME simulation_bridges_resource_path_test COMMAND simulation_bridges_resource_path_test)

  add_executable(simulation_bridges_velodyne_converter_test
    test/velodyne_pointcloud_converter_test.cpp
  )
  target_include_directories(simulation_bridges_velodyne_converter_test PRIVATE include)
  target_link_libraries(simulation_bridges_velodyne_converter_test ${catkin_LIBRARIES})
  add_test(
    NAME simulation_bridges_velodyne_converter_test
    COMMAND simulation_bridges_velodyne_converter_test
  )
endif()

install(TARGETS
  gazebo_ros_velodyne_gpu_laser
  gazebo_ground_truth_node
  livox_laser_simulation
  livox_pointcloud_bridge_node
  velodyne_pointcloud_bridge_node
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)

install(DIRECTORY scan_mode/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/scan_mode
)

install(DIRECTORY models/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/models
)

install(FILES
  third_party/livox_laser_simulation/LICENSE
  third_party/livox_laser_simulation/UPSTREAM.md
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/third_party/livox_laser_simulation
)
