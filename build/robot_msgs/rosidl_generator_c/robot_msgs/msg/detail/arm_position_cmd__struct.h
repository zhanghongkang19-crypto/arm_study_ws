// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from robot_msgs:msg/ArmPositionCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/arm_position_cmd.h"


#ifndef ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__STRUCT_H_
#define ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// Constants defined in the message

// Include directives for member types
// Member 'position'
// Member 'velocity'
// Member 'effort'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/ArmPositionCmd in the package robot_msgs.
typedef struct robot_msgs__msg__ArmPositionCmd
{
  rosidl_runtime_c__double__Sequence position;
  rosidl_runtime_c__double__Sequence velocity;
  rosidl_runtime_c__double__Sequence effort;
} robot_msgs__msg__ArmPositionCmd;

// Struct for a sequence of robot_msgs__msg__ArmPositionCmd.
typedef struct robot_msgs__msg__ArmPositionCmd__Sequence
{
  robot_msgs__msg__ArmPositionCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} robot_msgs__msg__ArmPositionCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__STRUCT_H_
