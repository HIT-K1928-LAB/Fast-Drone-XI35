cmake_minimum_required(VERSION 3.0.2)
project(sensor_simulator)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin and any catkin packages
find_package(catkin REQUIRED COMPONENTS
        roscpp
        nav_msgs
        std_msgs
        sensor_msgs
        cv_bridge
        pcl_ros
        pcl_conversions
)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
find_package(yaml-cpp REQUIRED)
message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

# CUDA
find_package(CUDA REQUIRED)
set(SENSOR_SIMULATOR_CUDA_ARCH "" CACHE STRING
    "CUDA architecture list for sensor_simulator, e.g. 86, 89, or 86;89. Empty means auto-detect on the build machine.")
if("${SENSOR_SIMULATOR_CUDA_ARCH}" STREQUAL "")
  cuda_select_nvcc_arch_flags(ARCH_FLAGS)
  if("${ARCH_FLAGS}" STREQUAL "")
    message(FATAL_ERROR
      "Failed to automatically detect CUDA architecture. "
      "Set SENSOR_SIMULATOR_CUDA_ARCH for the target machine, for example:\n"
      "  catkin_make -DSENSOR_SIMULATOR_CUDA_ARCH=86")
  endif()
else()
  set(ARCH_FLAGS "")
  foreach(ARCH ${SENSOR_SIMULATOR_CUDA_ARCH})
    list(APPEND ARCH_FLAGS -gencode arch=compute_${ARCH},code=sm_${ARCH})
  endforeach()
endif()
message(WARNING "CUDA Architecture: ${ARCH_FLAGS}")
set(CUDA_NVCC_FLAGS ${CUDA_NVCC_FLAGS} ${ARCH_FLAGS})

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_CUDA_FLAGS_RELEASE "-O3")

## Declare a catkin package
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES sensor_simulator
        CATKIN_DEPENDS roscpp nav_msgs std_msgs sensor_msgs cv_bridge pcl_ros pcl_conversions
        DEPENDS OpenCV)

## Specify additional locations of header files
include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}  # Eigen库路径
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(sensor_simulator src/test_simulator.cpp src/sensor_simulator.cpp src/maps.cpp src/perlinnoise.cpp)
target_link_libraries(sensor_simulator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} OpenMP::OpenMP_CXX yaml-cpp)
target_compile_definitions(sensor_simulator PRIVATE
        CONFIG_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml"
        PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")


cuda_add_library(raycast_cuda src/sensor_simulator.cu)
target_link_libraries(raycast_cuda ${CUDA_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})

add_executable(sensor_simulator_cuda src/test_simulator_cuda.cpp src/maps.cpp src/perlinnoise.cpp)
target_link_libraries(sensor_simulator_cuda ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} raycast_cuda yaml-cpp)
target_compile_definitions(sensor_simulator_cuda PRIVATE
        CONFIG_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml"
        PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

add_executable(dataset_generator src/dataset_generator.cpp src/maps.cpp src/perlinnoise.cpp)
target_link_libraries(dataset_generator ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES} raycast_cuda yaml-cpp)
target_compile_definitions(dataset_generator PRIVATE
        CONFIG_FILE_PATH="${CMAKE_CURRENT_SOURCE_DIR}/config/config.yaml"
        PACKAGE_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")
