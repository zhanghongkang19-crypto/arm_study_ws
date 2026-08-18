// generated from rosidl_typesupport_fastrtps_c/resource/idl__rosidl_typesupport_fastrtps_c.h.em
// with input from robot_msgs:msg/ArmPositionCmd.idl
// generated code does not contain a copyright notice
#ifndef ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
#define ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_


#include <stddef.h>
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "robot_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "robot_msgs/msg/detail/arm_position_cmd__struct.h"
#include "fastcdr/Cdr.h"

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
bool cdr_serialize_robot_msgs__msg__ArmPositionCmd(
  const robot_msgs__msg__ArmPositionCmd * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
bool cdr_deserialize_robot_msgs__msg__ArmPositionCmd(
  eprosima::fastcdr::Cdr &,
  robot_msgs__msg__ArmPositionCmd * ros_message);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t get_serialized_size_robot_msgs__msg__ArmPositionCmd(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t max_serialized_size_robot_msgs__msg__ArmPositionCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
bool cdr_serialize_key_robot_msgs__msg__ArmPositionCmd(
  const robot_msgs__msg__ArmPositionCmd * ros_message,
  eprosima::fastcdr::Cdr & cdr);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t get_serialized_size_key_robot_msgs__msg__ArmPositionCmd(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
size_t max_serialized_size_key_robot_msgs__msg__ArmPositionCmd(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, robot_msgs, msg, ArmPositionCmd)();

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__ROSIDL_TYPESUPPORT_FASTRTPS_C_H_
