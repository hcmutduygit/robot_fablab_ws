cmake_minimum_required(VERSION 3.0.2)
project(utils)


find_package(catkin REQUIRED COMPONENTS
    roscpp
    geometry_msgs
    std_msgs
    message_generation)

# Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  waypoint.msg
  cmd_vel.msg
  pose_robot.msg
)


generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
    # Or other packages containing msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES utils
   CATKIN_DEPENDS message_runtime std_msgs geometry_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)

