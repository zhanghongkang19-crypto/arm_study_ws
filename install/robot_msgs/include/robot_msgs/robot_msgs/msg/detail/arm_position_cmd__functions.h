// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from robot_msgs:msg/ArmPositionCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/arm_position_cmd.h"


#ifndef ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__FUNCTIONS_H_
#define ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/action_type_support_struct.h"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_runtime_c/service_type_support_struct.h"
#include "rosidl_runtime_c/type_description/type_description__struct.h"
#include "rosidl_runtime_c/type_description/type_source__struct.h"
#include "rosidl_runtime_c/type_hash.h"
#include "rosidl_runtime_c/visibility_control.h"
#include "robot_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "robot_msgs/msg/detail/arm_position_cmd__struct.h"

/// Initialize msg/ArmPositionCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * robot_msgs__msg__ArmPositionCmd
 * )) before or use
 * robot_msgs__msg__ArmPositionCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__msg__ArmPositionCmd__init(robot_msgs__msg__ArmPositionCmd * msg);

/// Finalize msg/ArmPositionCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__msg__ArmPositionCmd__fini(robot_msgs__msg__ArmPositionCmd * msg);

/// Create msg/ArmPositionCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * robot_msgs__msg__ArmPositionCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
robot_msgs__msg__ArmPositionCmd *
robot_msgs__msg__ArmPositionCmd__create(void);

/// Destroy msg/ArmPositionCmd message.
/**
 * It calls
 * robot_msgs__msg__ArmPositionCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__msg__ArmPositionCmd__destroy(robot_msgs__msg__ArmPositionCmd * msg);

/// Check for msg/ArmPositionCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__msg__ArmPositionCmd__are_equal(const robot_msgs__msg__ArmPositionCmd * lhs, const robot_msgs__msg__ArmPositionCmd * rhs);

/// Copy a msg/ArmPositionCmd message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__msg__ArmPositionCmd__copy(
  const robot_msgs__msg__ArmPositionCmd * input,
  robot_msgs__msg__ArmPositionCmd * output);

/// Retrieve pointer to the hash of the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__msg__ArmPositionCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__msg__ArmPositionCmd__get_type_description(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the single raw source text that defined this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__msg__ArmPositionCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support);

/// Retrieve pointer to the recursive raw sources that defined the description of this type.
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__msg__ArmPositionCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support);

/// Initialize array of msg/ArmPositionCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * robot_msgs__msg__ArmPositionCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__msg__ArmPositionCmd__Sequence__init(robot_msgs__msg__ArmPositionCmd__Sequence * array, size_t size);

/// Finalize array of msg/ArmPositionCmd messages.
/**
 * It calls
 * robot_msgs__msg__ArmPositionCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__msg__ArmPositionCmd__Sequence__fini(robot_msgs__msg__ArmPositionCmd__Sequence * array);

/// Create array of msg/ArmPositionCmd messages.
/**
 * It allocates the memory for the array and calls
 * robot_msgs__msg__ArmPositionCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
robot_msgs__msg__ArmPositionCmd__Sequence *
robot_msgs__msg__ArmPositionCmd__Sequence__create(size_t size);

/// Destroy array of msg/ArmPositionCmd messages.
/**
 * It calls
 * robot_msgs__msg__ArmPositionCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
void
robot_msgs__msg__ArmPositionCmd__Sequence__destroy(robot_msgs__msg__ArmPositionCmd__Sequence * array);

/// Check for msg/ArmPositionCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__msg__ArmPositionCmd__Sequence__are_equal(const robot_msgs__msg__ArmPositionCmd__Sequence * lhs, const robot_msgs__msg__ArmPositionCmd__Sequence * rhs);

/// Copy an array of msg/ArmPositionCmd messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
bool
robot_msgs__msg__ArmPositionCmd__Sequence__copy(
  const robot_msgs__msg__ArmPositionCmd__Sequence * input,
  robot_msgs__msg__ArmPositionCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__FUNCTIONS_H_
