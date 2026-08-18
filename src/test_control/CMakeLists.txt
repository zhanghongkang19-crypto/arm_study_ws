cmake_minimum_required(VERSION 3.28)

project(test_control)


find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

find_package(robot_msgs REQUIRED)
find_package(robot_motion REQUIRED)

add_executable(robot_monitor
  src/test_control.cpp
)


ament_target_dependencies(robot_monitor
  rclcpp
  std_msgs
  sensor_msgs
  robot_msgs
  robot_motion
)


install(
  TARGETS robot_monitor
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()