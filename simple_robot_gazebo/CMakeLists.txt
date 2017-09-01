cmake_minimum_required(VERSION 2.8.3)
project(simple_robot_gazebo)

## Add support for C++11, supported in ROS Kinetic and newer
add_definitions(-std=c++11)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roslaunch
  roscpp
  gazebo_ros
  gazebo_plugins
)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES diff_wheeled_robot_gazebo
#  CATKIN_DEPENDS gazebo_msgs gazebo_plugins gazebo_ros gazebo_ros_control simple_robot_description
  DEPENDS roscpp gazebo_ros
)

###########
## Build ##
###########

link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  include/${PROJECT_NAME}/
)

add_library(moveWheels_plugin plugins/move_wheels.cc)
target_link_libraries(moveWheels_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(encodersTicks_plugin plugins/encoders_ticks.cc)
target_link_libraries(encodersTicks_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

add_library(odom_plugin plugins/odom.cc)
target_link_libraries(odom_plugin ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})
