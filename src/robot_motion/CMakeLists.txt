cmake_minimum_required(VERSION 3.28)

project(robot_motion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)

# =========================
# Library
# =========================

add_library(robot_motion SHARED
  src/pseudo_inverse_ik.cpp
)

# 当前包编译 + 下游包使用的头文件路径
target_include_directories(robot_motion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

# =========================
# Dependencies
# =========================

ament_target_dependencies(robot_motion
  rclcpp
  std_msgs
  sensor_msgs
  robot_msgs
  ament_index_cpp
)

target_link_libraries(robot_motion
  pinocchio::pinocchio
)

# =========================
# Install library
# =========================

install(
  TARGETS robot_motion
  EXPORT robot_motionTargets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# =========================
# Install headers
# =========================

install(
  DIRECTORY include/
  DESTINATION include
)

# =========================
# Install CMake target export
# =========================

install(
  EXPORT robot_motionTargets
  FILE robot_motionTargets.cmake
  DESTINATION share/${PROJECT_NAME}/cmake
)

# =========================
# Export to downstream packages
# =========================

ament_export_include_directories(include)

ament_export_targets(
  robot_motionTargets
  HAS_LIBRARY_TARGET
)

ament_export_dependencies(
  rclcpp
  std_msgs
  sensor_msgs
  robot_msgs
  ament_index_cpp
  pinocchio
)

ament_package()