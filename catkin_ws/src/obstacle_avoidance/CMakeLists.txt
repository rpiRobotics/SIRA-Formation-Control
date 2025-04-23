cmake_minimum_required(VERSION 3.0.2)
project(obstacle_avoidance)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  message_generation
  obstacle_detector
)

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    std_msgs
    geometry_msgs
    )


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES obstacle_avoidance
#  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

## First Executable

add_executable(obstacle_avoid src/obstacle_avoid.cpp)
target_link_libraries(obstacle_avoid ${catkin_LIBRARIES})
add_dependencies(obstacle_avoid ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Second Executable

add_executable(check_vel src/check_vel.cpp)
target_link_libraries(check_vel ${catkin_LIBRARIES})
add_dependencies(check_vel ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Third Executable

add_executable(lidar_base_static_tf_pub src/lidar_base_static_tf_pub.cpp)
target_link_libraries(lidar_base_static_tf_pub ${catkin_LIBRARIES})
add_dependencies(lidar_base_static_tf_pub ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Fourth Executable

add_executable(potential src/potential.cpp)
target_link_libraries(potential ${catkin_LIBRARIES})
add_dependencies(potential ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Fifth Executable

add_executable(routing src/routing.cpp)
target_link_libraries(routing ${catkin_LIBRARIES})
add_dependencies(routing ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


## Sixth Executable

add_executable(simulating_route src/simulating_route.cpp)
target_link_libraries(simulating_route ${catkin_LIBRARIES})
add_dependencies(simulating_route ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})