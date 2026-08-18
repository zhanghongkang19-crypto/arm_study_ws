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

add_executable(test_pseudo_inverse_ik_node
  src/test_pseudo_inverse_ik.cpp
)
# 直接在这里引入 robot_motion 即可，ament 会自动完成头文件路径添加和库链接
ament_target_dependencies(test_pseudo_inverse_ik_node
  rclcpp
  std_msgs
  sensor_msgs
  robot_msgs
  robot_motion
)

install(
  TARGETS 
    robot_monitor
    test_pseudo_inverse_ik_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()