cmake_minimum_required(VERSION 3.0.2)
project(yolo_trt_detector)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  std_msgs
  tf2
  visualization_msgs
  image_transport
  cv_bridge_454
)

set(OpenCV_DIR /usr/local/opencv-4.5.4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 visualization_msgs image_transport
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  /usr/include/aarch64-linux-gnu
)

link_directories(
  /usr/lib/aarch64-linux-gnu
)

add_executable(yolo_trt_node
  src/yolo_trt_node.cpp
)

target_link_libraries(yolo_trt_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  nvinfer
)
