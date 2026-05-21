cmake_minimum_required(VERSION 3.0.2)
project(yopo_planner)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  message_generation
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  quadrotor_msgs
)

add_message_files(
  FILES
  YopoLog.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

set(OpenCV_DIR /usr/local/opencv-4.5.4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(CUDA REQUIRED)

set(HAVE_OPENCV_PHOTO OFF)
foreach(OPENCV_LIB ${OpenCV_LIBS})
  if(OPENCV_LIB MATCHES "opencv_photo")
    set(HAVE_OPENCV_PHOTO ON)
  endif()
endforeach()
if(HAVE_OPENCV_PHOTO)
  message(STATUS "OpenCV photo module: enabled")
else()
  message(STATUS "OpenCV photo module: not found, YOPO depth fill will use fallback")
endif()

find_path(TENSORRT_INCLUDE_DIR NvInfer.h
  HINTS
    /usr/local/include/tensorrt
    /usr/local/tensorrt/include
    /usr/include/x86_64-linux-gnu
    /usr/include
)
find_library(NVINFER_LIBRARY nvinfer
  HINTS /usr/local/tensorrt/lib /usr/lib/x86_64-linux-gnu /usr/lib
)
find_library(NVONNXPARSER_LIBRARY nvonnxparser
  HINTS /usr/local/tensorrt/lib /usr/lib/x86_64-linux-gnu /usr/lib
)

if(NOT TENSORRT_INCLUDE_DIR OR NOT NVINFER_LIBRARY OR NOT NVONNXPARSER_LIBRARY)
  message(FATAL_ERROR "TensorRT headers/libraries not found. Expected NvInfer.h, libnvinfer, and libnvonnxparser.")
endif()
message(STATUS "TensorRT include: ${TENSORRT_INCLUDE_DIR}")
message(STATUS "TensorRT nvinfer: ${NVINFER_LIBRARY}")
message(STATUS "TensorRT nvonnxparser: ${NVONNXPARSER_LIBRARY}")

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES yopo_planner_lib
  CATKIN_DEPENDS message_runtime roscpp std_msgs sensor_msgs nav_msgs geometry_msgs quadrotor_msgs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${CUDA_INCLUDE_DIRS}
  ${TENSORRT_INCLUDE_DIR}
)

add_library(yopo_planner_lib
  src/yopo_engine.cpp
  src/yopo_planner.cpp
)
add_dependencies(yopo_planner_lib ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(yopo_planner_lib
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  ${NVINFER_LIBRARY}
  ${NVONNXPARSER_LIBRARY}
)
if(HAVE_OPENCV_PHOTO)
  target_compile_definitions(yopo_planner_lib PUBLIC YOPO_HAVE_OPENCV_PHOTO=1)
endif()

add_executable(yopo_planner_node src/yopo_node.cpp)
target_link_libraries(yopo_planner_node
  yopo_planner_lib
  ${catkin_LIBRARIES}
)
add_dependencies(yopo_planner_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/export_yopo_onnx.py
  scripts/record_yopo_log.sh
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
