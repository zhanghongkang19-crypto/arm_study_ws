// generated from rosidl_generator_c/resource/idl__description.c.em
// with input from robot_msgs:msg/ArmPositionCmd.idl
// generated code does not contain a copyright notice

#include "robot_msgs/msg/detail/arm_position_cmd__functions.h"

ROSIDL_GENERATOR_C_PUBLIC_robot_msgs
const rosidl_type_hash_t *
robot_msgs__msg__ArmPositionCmd__get_type_hash(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_type_hash_t hash = {1, {
      0x13, 0x66, 0xc6, 0x97, 0xda, 0xf2, 0x99, 0x5d,
      0xc8, 0x46, 0x22, 0xa7, 0x6a, 0x2c, 0x3e, 0x39,
      0x70, 0xdb, 0x8a, 0x78, 0xca, 0x93, 0xc1, 0xdb,
      0x46, 0xc7, 0x1e, 0x72, 0x8f, 0xd1, 0x4a, 0x0f,
    }};
  return &hash;
}

#include <assert.h>
#include <string.h>

// Include directives for referenced types

// Hashes for external referenced types
#ifndef NDEBUG
#endif

static char robot_msgs__msg__ArmPositionCmd__TYPE_NAME[] = "robot_msgs/msg/ArmPositionCmd";

// Define type names, field names, and default values
static char robot_msgs__msg__ArmPositionCmd__FIELD_NAME__position[] = "position";
static char robot_msgs__msg__ArmPositionCmd__FIELD_NAME__velocity[] = "velocity";
static char robot_msgs__msg__ArmPositionCmd__FIELD_NAME__effort[] = "effort";

static rosidl_runtime_c__type_description__Field robot_msgs__msg__ArmPositionCmd__FIELDS[] = {
  {
    {robot_msgs__msg__ArmPositionCmd__FIELD_NAME__position, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ArmPositionCmd__FIELD_NAME__velocity, 8, 8},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
  {
    {robot_msgs__msg__ArmPositionCmd__FIELD_NAME__effort, 6, 6},
    {
      rosidl_runtime_c__type_description__FieldType__FIELD_TYPE_DOUBLE_UNBOUNDED_SEQUENCE,
      0,
      0,
      {NULL, 0, 0},
    },
    {NULL, 0, 0},
  },
};

const rosidl_runtime_c__type_description__TypeDescription *
robot_msgs__msg__ArmPositionCmd__get_type_description(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static bool constructed = false;
  static const rosidl_runtime_c__type_description__TypeDescription description = {
    {
      {robot_msgs__msg__ArmPositionCmd__TYPE_NAME, 29, 29},
      {robot_msgs__msg__ArmPositionCmd__FIELDS, 3, 3},
    },
    {NULL, 0, 0},
  };
  if (!constructed) {
    constructed = true;
  }
  return &description;
}

static char toplevel_type_raw_source[] =
  "float64[] position\n"
  "float64[] velocity\n"
  "float64[] effort";

static char msg_encoding[] = "msg";

// Define all individual source functions

const rosidl_runtime_c__type_description__TypeSource *
robot_msgs__msg__ArmPositionCmd__get_individual_type_description_source(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static const rosidl_runtime_c__type_description__TypeSource source = {
    {robot_msgs__msg__ArmPositionCmd__TYPE_NAME, 29, 29},
    {msg_encoding, 3, 3},
    {toplevel_type_raw_source, 54, 54},
  };
  return &source;
}

const rosidl_runtime_c__type_description__TypeSource__Sequence *
robot_msgs__msg__ArmPositionCmd__get_type_description_sources(
  const rosidl_message_type_support_t * type_support)
{
  (void)type_support;
  static rosidl_runtime_c__type_description__TypeSource sources[1];
  static const rosidl_runtime_c__type_description__TypeSource__Sequence source_sequence = {sources, 1, 1};
  static bool constructed = false;
  if (!constructed) {
    sources[0] = *robot_msgs__msg__ArmPositionCmd__get_individual_type_description_source(NULL),
    constructed = true;
  }
  return &source_sequence;
}
