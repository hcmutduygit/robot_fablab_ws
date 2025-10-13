cmake_minimum_required(VERSION 3.0.2)
project(map_basefootprint_publisher)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  utils
  tf
  message_runtime
)

catkin_package(
  CATKIN_DEPENDS geometry_msgs roscpp tf utils message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_basefootprint_publisher_node 
              src/map_basefootprint_publisher.cpp)

add_dependencies(map_basefootprint_publisher_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(map_basefootprint_publisher_node
  ${catkin_LIBRARIES}
)
