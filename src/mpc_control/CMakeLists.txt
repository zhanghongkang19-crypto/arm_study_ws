cmake_minimum_required(VERSION 3.8)
project(mpc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(pinocchio REQUIRED)
find_package(ocs2_core REQUIRED)
find_package(ocs2_oc REQUIRED)
find_package(ocs2_mpc REQUIRED)
find_package(ocs2_ddp REQUIRED)

add_library(mpc_controller_core
  src/pinocchio_arm_dynamics.cpp
  src/end_effector_pose_cost.cpp
  src/mpc_controller.cpp
)
target_compile_features(mpc_controller_core PUBLIC cxx_std_17)
target_include_directories(mpc_controller_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(mpc_controller_core
  geometry_msgs
  sensor_msgs
  rclcpp
  std_msgs
)
target_link_libraries(mpc_controller_core
  pinocchio::pinocchio
  ${ocs2_core_TARGETS}
  ${ocs2_oc_TARGETS}
  ${ocs2_mpc_TARGETS}
  ${ocs2_ddp_TARGETS}
)

add_executable(mpc_controller_node src/mpc_controller_node.cpp)
target_compile_features(mpc_controller_node PUBLIC cxx_std_17)
target_link_libraries(mpc_controller_node mpc_controller_core)
ament_target_dependencies(mpc_controller_node rclcpp)

install(
  TARGETS mpc_controller_core mpc_controller_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
