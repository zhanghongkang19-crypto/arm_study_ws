cmake_minimum_required(VERSION 3.28)
project(test_control)

set(Boost_USE_DEBUG_RUNTIME OFF CACHE BOOL "Disable Boost Debug Runtime" FORCE)
set(Boost_USE_STATIC_LIBS OFF CACHE BOOL "Disable Boost Static" FORCE)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(robot_motion REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

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

add_executable(test_ik
  src/test_ik.cpp
)
# 直接在这里引入 robot_motion 即可，ament 会自动完成头文件路径添加和库链接
ament_target_dependencies(test_ik
  rclcpp
  std_msgs
  sensor_msgs
  robot_msgs
  robot_motion
)

add_executable(test_moveit_controller
  src/test_moveit_controller.cpp
)
ament_target_dependencies(test_moveit_controller
  rclcpp
  std_msgs
  sensor_msgs
  robot_msgs
  robot_motion
  control_msgs
  rclcpp_action
  moveit_ros_planning_interface
)

install(
  TARGETS 
    robot_monitor
    test_ik
    test_moveit_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()