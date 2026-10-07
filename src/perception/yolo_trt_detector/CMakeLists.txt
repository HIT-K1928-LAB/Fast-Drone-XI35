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
  message_filters
  cv_bridge_454
)

if(EXISTS "/usr/local/include/tensorrt")
  message(STATUS "tensorrt include directory: /usr/local/include/tensorrt")
  link_directories(/usr/local/tensorrt/lib)
  include_directories(/usr/local/include/tensorrt)
else()
  message(STATUS "external path for tensorrt not found")
endif()

set(OpenCV_DIR /usr/local/opencv-4.5.4/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
find_package(CUDA REQUIRED)

catkin_package(
  CATKIN_DEPENDS roscpp geometry_msgs nav_msgs sensor_msgs std_msgs tf2 visualization_msgs image_transport message_filters cv_bridge_454
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

add_library(yolo_lidar_target_tracker
  src/lidar_target_tracker.cpp
)

target_link_libraries(yolo_lidar_target_tracker
  ${OpenCV_LIBRARIES}
)

add_executable(yolo_trt_node
  src/yolo_trt_node.cpp
)

target_link_libraries(yolo_trt_node
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${CUDA_LIBRARIES}
  nvinfer
  yolo_lidar_target_tracker
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(yolo_image_orientation_test
    test/test_image_orientation.cpp
  )
  if(TARGET yolo_image_orientation_test)
    target_link_libraries(yolo_image_orientation_test
      ${OpenCV_LIBRARIES}
    )
  endif()

  catkin_add_gtest(yolo_standoff_goal_test
    test/test_standoff_goal.cpp
  )
  if(TARGET yolo_standoff_goal_test)
    target_link_libraries(yolo_standoff_goal_test
      ${catkin_LIBRARIES}
    )
  endif()

  catkin_add_gtest(yolo_track_reacquisition_test
    test/test_track_reacquisition.cpp
  )

  catkin_add_gtest(yolo_publication_limiter_test
    test/test_publication_limiter.cpp
  )

  catkin_add_gtest(yolo_lidar_target_tracker_test
    test/test_lidar_target_tracker.cpp
  )
  if(TARGET yolo_lidar_target_tracker_test)
    target_link_libraries(yolo_lidar_target_tracker_test
      yolo_lidar_target_tracker
      ${OpenCV_LIBRARIES}
    )
  endif()

  catkin_add_gtest(yolo_visual_lidar_match_test
    test/test_visual_lidar_match.cpp
  )

  catkin_add_gtest(yolo_frustum_lidar_association_test
    test/test_frustum_lidar_association.cpp
  )
  if(TARGET yolo_frustum_lidar_association_test)
    target_link_libraries(yolo_frustum_lidar_association_test
      ${OpenCV_LIBRARIES}
    )
  endif()
endif()
