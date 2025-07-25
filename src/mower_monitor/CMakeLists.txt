cmake_minimum_required(VERSION 3.0.2)
project(mower_monitor)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  mower_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(mower_monitor_node src/mower_monitor_node.cpp)

add_dependencies(mower_monitor_node ${catkin_EXPORTED_TARGETS} ${mower_msgs_EXPORTED_TARGETS} )

target_link_libraries(mower_monitor_node
  ${catkin_LIBRARIES}
  ${mower_msgs_LIBRARIES}
)
