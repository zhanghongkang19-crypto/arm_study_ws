cmake_minimum_required(VERSION 3.28)

project(robot_motion)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robot_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(pinocchio REQUIRED)
find_package(qpOASES REQUIRED)

find_path(CASADI_INCLUDE_DIR
  NAMES casadi/casadi.hpp
  PATHS /usr/local/include
)

find_library(CASADI_LIBRARY
  NAMES casadi
  PATHS /usr/local/lib
)

if(NOT CASADI_INCLUDE_DIR)
  message(FATAL_ERROR "CasADi header not found")
endif()

if(NOT CASADI_LIBRARY)
  message(FATAL_ERROR "CasADi library not found")
endif()

message(STATUS "CasADi include: ${CASADI_INCLUDE_DIR}")
message(STATUS "CasADi library: ${CASADI_LIBRARY}")



# =========================
# Library
# =========================

add_library(robot_motion SHARED
  src/pseudo_inverse_ik.cpp
  src/nlp_ik.cpp
)

# 当前包编译 + 下游包使用的头文件路径
target_include_directories(robot_motion
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${CASADI_INCLUDE_DIR}
    ${QPOASES_INCLUDE_DIR}
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
  ${CASADI_LIBRARY}
  qpOASES
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
  CasADi
)

ament_package()