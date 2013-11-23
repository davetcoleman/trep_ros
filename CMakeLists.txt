cmake_minimum_required(VERSION 2.8.3)
project(trep_ros)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kdl_parser
  moveit_core
  moveit_ros_planning
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS geometry_msgs moveit_core roscpp #kdl_parser
  DEPENDS Eigen
)

include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} src/trep_ros.cpp)
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(trep_ros_main src/trep_ros_main.cpp)
add_dependencies(trep_ros_main ${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})
target_link_libraries(trep_ros_main ${catkin_LIBRARIES})


