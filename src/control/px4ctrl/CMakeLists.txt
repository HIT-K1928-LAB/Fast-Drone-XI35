cmake_minimum_required(VERSION 2.8.3)
project(px4ctrl)

set(CMAKE_VERBOSE_MAKEFILE "false")

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  dynamic_reconfigure
  quadrotor_msgs
  geometry_msgs
  sensor_msgs
  uav_utils
  mavros
)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake")
find_package(Eigen3 REQUIRED)

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  include/${PROJECT_NAME}
  include
)

add_executable(px4ctrl_node
  src/px4ctrl_node.cpp
  src/PX4CtrlFSM.cpp
  src/PX4CtrlParam.cpp
  src/controller.cpp
  src/input.cpp
)

add_dependencies(px4ctrl_node ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg)

target_link_libraries(px4ctrl_node
  ${catkin_LIBRARIES}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_gtest(px4ctrl_thrust_model_test
    test/thrust_model_config_test.cpp
    src/PX4CtrlFSM.cpp
    src/PX4CtrlParam.cpp
    src/controller.cpp
    src/input.cpp
  )
  if(TARGET px4ctrl_thrust_model_test)
    add_dependencies(px4ctrl_thrust_model_test ${catkin_EXPORTED_TARGETS})
    target_link_libraries(px4ctrl_thrust_model_test ${catkin_LIBRARIES})
  endif()
endif()

catkin_install_python(PROGRAMS
  scripts/pid_setpoint_excitation.py
  thrust_calibrate_scrips/thrust_calibrate.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
