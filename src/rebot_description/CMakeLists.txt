cmake_minimum_required(VERSION 3.8)

project(rebot_description)


find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)


# python node
install(
  PROGRAMS
    script/arm_mujoco_node.py
  DESTINATION lib/${PROJECT_NAME}
)


# robot model
install(
  DIRECTORY
    urdf
  DESTINATION share/${PROJECT_NAME}
)


ament_package()
