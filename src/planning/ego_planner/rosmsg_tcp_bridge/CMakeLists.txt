cmake_minimum_required(VERSION 2.8.3)
project(rosmsg_tcp_bridge)

set(CMAKE_BUILD_TYPE "Release")
ADD_COMPILE_OPTIONS(-std=c++11 )
ADD_COMPILE_OPTIONS(-std=c++14 )
set(CMAKE_CXX_FLAGS_RELEASE "-O2 -Wall -g")

find_package(Eigen3 REQUIRED)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  nav_msgs
  geometry_msgs
  traj_utils
)

catkin_package(
 CATKIN_DEPENDS roscpp std_msgs nav_msgs geometry_msgs traj_utils
)

include_directories(
  SYSTEM
  ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
  ${EIGEN3_INCLUDE_DIR}
)

add_executable(bridge_node
  src/bridge_node.cpp 
  )

add_dependencies(bridge_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(bridge_node 
  ${catkin_LIBRARIES}
  )

