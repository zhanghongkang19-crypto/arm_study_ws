# generated from rosidl_cmake/cmake/rosidl_cmake_aggregate_target-extras.cmake.in

# Create a convenience aggregate target robot_msgs::robot_msgs
# that links all generated interface targets, so downstream packages can use
# a single modern CMake target name instead of ${robot_msgs_TARGETS}.
if(robot_msgs_TARGETS AND NOT TARGET robot_msgs::robot_msgs)
  add_library(robot_msgs::robot_msgs INTERFACE IMPORTED)
  set_target_properties(robot_msgs::robot_msgs PROPERTIES
    INTERFACE_LINK_LIBRARIES "${robot_msgs_TARGETS}")
endif()
