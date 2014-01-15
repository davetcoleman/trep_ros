cmake_minimum_required(VERSION 2.8.3)
project(trep_ros)

message(STATUS "dave was here")

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  kdl_parser
  moveit_core
  moveit_ros_planning
  roscpp
)

find_package(Boost REQUIRED COMPONENTS system)
find_package(orocos_kdl REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES trep_ros
  CATKIN_DEPENDS geometry_msgs moveit_core roscpp kdl_parser
  DEPENDS Eigen
)

include_directories(include ${orocos_kdl_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

link_directories(${orocos_kdl_LIBRARY_DIRS})

# output all variables
#get_cmake_property(_variableNames VARIABLES)
#foreach (_variableName ${_variableNames})
#    message(STATUS "${_variableName}=${${_variableName}}")
#endforeach()

add_library(trep_ros src/trep_ros.cpp)
add_dependencies(trep_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(trep_ros ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_executable(trep_ros_main src/trep_ros_main.cpp)
add_dependencies(trep_ros_main trep_ros ${catkin_EXPORTED_TARGETS})
target_link_libraries(trep_ros_main trep_ros ${orocos_kdl_LIBRARIES} ${catkin_LIBRARIES})


