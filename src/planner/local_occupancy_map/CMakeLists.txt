cmake_minimum_required(VERSION 3.0.2)
project(local_occupancy_map)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

set(OpenCV_DIR /usr/local/opencv-4.5.4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
  nav_msgs
  pcl_ros
  message_filters
  cv_bridge_454
  plan_env
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES local_occupancy_map
  CATKIN_DEPENDS roscpp sensor_msgs nav_msgs pcl_ros message_filters cv_bridge_454 plan_env
  DEPENDS OpenCV PCL
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_library(local_occupancy_map
  src/local_occupancy_map.cpp
)
target_link_libraries(local_occupancy_map
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
  ${PCL_LIBRARIES}
)

add_executable(local_occupancy_map_node
  src/local_occupancy_map_node.cpp
)
target_link_libraries(local_occupancy_map_node
  local_occupancy_map
  ${catkin_LIBRARIES}
)
