cmake_minimum_required(VERSION 3.8)

project(rebot_description)

# =========================
# ROS 2 dependencies
# =========================
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)

find_package(Eigen3 REQUIRED)


# =========================
# C++ Interactive Marker
# =========================
add_executable(target_marker
  src/interactive_marker.cpp
)

ament_target_dependencies(target_marker
  rclcpp
  geometry_msgs
  visualization_msgs
  interactive_markers
)

target_link_libraries(target_marker
  Eigen3::Eigen
)


# =========================
# Install C++ executable
# =========================
install(
  TARGETS target_marker
  DESTINATION lib/${PROJECT_NAME}
)


# =========================
# Install Python node
# =========================
install(
  PROGRAMS
    script/arm_mujoco_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# =========================
# Install robot description
# =========================
install(
  DIRECTORY
    urdf
    launch
    script
  DESTINATION share/${PROJECT_NAME}
)


# =========================
# Install script directory
# =========================
# 如果 script 里面还有其他需要运行的 Python 文件，
# 可以保留这个；否则可以不写。
install(
  DIRECTORY script/
  DESTINATION share/${PROJECT_NAME}/script
)


ament_package()
