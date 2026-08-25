// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from robot_msgs:msg/ArmPositionCmd.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "robot_msgs/msg/detail/arm_position_cmd__rosidl_typesupport_introspection_c.h"
#include "robot_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "robot_msgs/msg/detail/arm_position_cmd__functions.h"
#include "robot_msgs/msg/detail/arm_position_cmd__struct.h"


// Include directives for member types
// Member `position`
// Member `velocity`
// Member `effort`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

#ifdef __cplusplus
extern "C"
{
#endif

void robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  robot_msgs__msg__ArmPositionCmd__init(message_memory);
}

void robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_fini_function(void * message_memory)
{
  robot_msgs__msg__ArmPositionCmd__fini(message_memory);
}

size_t robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__size_function__ArmPositionCmd__position(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__position(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__position(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__fetch_function__ArmPositionCmd__position(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__position(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__assign_function__ArmPositionCmd__position(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__position(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__resize_function__ArmPositionCmd__position(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__size_function__ArmPositionCmd__velocity(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__velocity(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__velocity(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__fetch_function__ArmPositionCmd__velocity(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__velocity(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__assign_function__ArmPositionCmd__velocity(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__velocity(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__resize_function__ArmPositionCmd__velocity(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

size_t robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__size_function__ArmPositionCmd__effort(
  const void * untyped_member)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return member->size;
}

const void * robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__effort(
  const void * untyped_member, size_t index)
{
  const rosidl_runtime_c__double__Sequence * member =
    (const rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void * robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__effort(
  void * untyped_member, size_t index)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  return &member->data[index];
}

void robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__fetch_function__ArmPositionCmd__effort(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const double * item =
    ((const double *)
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__effort(untyped_member, index));
  double * value =
    (double *)(untyped_value);
  *value = *item;
}

void robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__assign_function__ArmPositionCmd__effort(
  void * untyped_member, size_t index, const void * untyped_value)
{
  double * item =
    ((double *)
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__effort(untyped_member, index));
  const double * value =
    (const double *)(untyped_value);
  *item = *value;
}

bool robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__resize_function__ArmPositionCmd__effort(
  void * untyped_member, size_t size)
{
  rosidl_runtime_c__double__Sequence * member =
    (rosidl_runtime_c__double__Sequence *)(untyped_member);
  rosidl_runtime_c__double__Sequence__fini(member);
  return rosidl_runtime_c__double__Sequence__init(member, size);
}

static rosidl_typesupport_introspection_c__MessageMember robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_message_member_array[3] = {
  {
    "position",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__ArmPositionCmd, position),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__size_function__ArmPositionCmd__position,  // size() function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__position,  // get_const(index) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__position,  // get(index) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__fetch_function__ArmPositionCmd__position,  // fetch(index, &value) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__assign_function__ArmPositionCmd__position,  // assign(index, value) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__resize_function__ArmPositionCmd__position  // resize(index) function pointer
  },
  {
    "velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__ArmPositionCmd, velocity),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__size_function__ArmPositionCmd__velocity,  // size() function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__velocity,  // get_const(index) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__velocity,  // get(index) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__fetch_function__ArmPositionCmd__velocity,  // fetch(index, &value) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__assign_function__ArmPositionCmd__velocity,  // assign(index, value) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__resize_function__ArmPositionCmd__velocity  // resize(index) function pointer
  },
  {
    "effort",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is key
    true,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(robot_msgs__msg__ArmPositionCmd, effort),  // bytes offset in struct
    NULL,  // default value
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__size_function__ArmPositionCmd__effort,  // size() function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_const_function__ArmPositionCmd__effort,  // get_const(index) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__get_function__ArmPositionCmd__effort,  // get(index) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__fetch_function__ArmPositionCmd__effort,  // fetch(index, &value) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__assign_function__ArmPositionCmd__effort,  // assign(index, value) function pointer
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__resize_function__ArmPositionCmd__effort  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_message_members = {
  "robot_msgs__msg",  // message namespace
  "ArmPositionCmd",  // message name
  3,  // number of fields
  sizeof(robot_msgs__msg__ArmPositionCmd),
  false,  // has_any_key_member_
  robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_message_member_array,  // message members
  robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_init_function,  // function to initialize message memory (memory has to be allocated)
  robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_message_type_support_handle = {
  0,
  &robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_message_members,
  get_message_typesupport_handle_function,
  &robot_msgs__msg__ArmPositionCmd__get_type_hash,
  &robot_msgs__msg__ArmPositionCmd__get_type_description,
  &robot_msgs__msg__ArmPositionCmd__get_type_description_sources,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_robot_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, robot_msgs, msg, ArmPositionCmd)() {
  if (!robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_message_type_support_handle.typesupport_identifier) {
    robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &robot_msgs__msg__ArmPositionCmd__rosidl_typesupport_introspection_c__ArmPositionCmd_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
