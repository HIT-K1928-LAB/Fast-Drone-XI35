cmake_minimum_required(VERSION 3.0.2)
project(px4ctrl)

add_compile_options(-std=c++14 -Wall -Wextra -Wpedantic)

find_package(catkin REQUIRED COMPONENTS
  diagnostic_msgs
  geometry_msgs
  message_generation
  mavros_msgs
  nav_msgs
  quadrotor_msgs
  roscpp
  rospy
  std_msgs
  std_srvs
)

add_message_files(
  FILES
  FsmCommand.msg
  FsmStatus.msg
)

generate_messages(
  DEPENDENCIES
  geometry_msgs
  std_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
    diagnostic_msgs
    geometry_msgs
    message_runtime
    mavros_msgs
    nav_msgs
    quadrotor_msgs
    roscpp
    rospy
    std_msgs
    std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(px4_offboard_fsm_core
  src/config.cpp
  src/fsm_core.cpp
)
add_dependencies(px4_offboard_fsm_core
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(px4_offboard_fsm_core ${catkin_LIBRARIES})

add_executable(offboard_fsm_node src/offboard_fsm_node.cpp)
add_dependencies(offboard_fsm_node
  ${${PROJECT_NAME}_EXPORTED_TARGETS}
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(offboard_fsm_node
  px4_offboard_fsm_core
  ${catkin_LIBRARIES}
)

catkin_install_python(PROGRAMS
  scripts/adaptive_circle_trajectory.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(TARGETS px4_offboard_fsm_core offboard_fsm_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY launch config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if(CATKIN_ENABLE_TESTING)
  catkin_add_nosetests(test/test_adaptive_circle_trajectory.py)
  catkin_add_gtest(offboard_fsm_core_test test/offboard_fsm_core_test.cpp)
  if(TARGET offboard_fsm_core_test)
    add_dependencies(offboard_fsm_core_test
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
    )
    target_link_libraries(offboard_fsm_core_test
      px4_offboard_fsm_core
      ${catkin_LIBRARIES}
    )
  endif()
endif()
