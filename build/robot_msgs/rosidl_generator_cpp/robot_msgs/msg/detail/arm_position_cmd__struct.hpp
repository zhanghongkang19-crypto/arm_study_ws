// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from robot_msgs:msg/ArmPositionCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/arm_position_cmd.hpp"


#ifndef ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__STRUCT_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__robot_msgs__msg__ArmPositionCmd __attribute__((deprecated))
#else
# define DEPRECATED__robot_msgs__msg__ArmPositionCmd __declspec(deprecated)
#endif

namespace robot_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ArmPositionCmd_
{
  using Type = ArmPositionCmd_<ContainerAllocator>;

  explicit ArmPositionCmd_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
  }

  explicit ArmPositionCmd_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_init;
    (void)_alloc;
  }

  // field types and members
  using _position_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _position_type position;
  using _velocity_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _velocity_type velocity;
  using _effort_type =
    std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>>;
  _effort_type effort;

  // setters for named parameter idiom
  Type & set__position(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->position = _arg;
    return *this;
  }
  Type & set__velocity(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->velocity = _arg;
    return *this;
  }
  Type & set__effort(
    const std::vector<double, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<double>> & _arg)
  {
    this->effort = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    robot_msgs::msg::ArmPositionCmd_<ContainerAllocator> *;
  using ConstRawPtr =
    const robot_msgs::msg::ArmPositionCmd_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<robot_msgs::msg::ArmPositionCmd_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<robot_msgs::msg::ArmPositionCmd_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::ArmPositionCmd_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::ArmPositionCmd_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      robot_msgs::msg::ArmPositionCmd_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<robot_msgs::msg::ArmPositionCmd_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<robot_msgs::msg::ArmPositionCmd_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<robot_msgs::msg::ArmPositionCmd_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__robot_msgs__msg__ArmPositionCmd
    std::shared_ptr<robot_msgs::msg::ArmPositionCmd_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__robot_msgs__msg__ArmPositionCmd
    std::shared_ptr<robot_msgs::msg::ArmPositionCmd_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ArmPositionCmd_ & other) const
  {
    if (this->position != other.position) {
      return false;
    }
    if (this->velocity != other.velocity) {
      return false;
    }
    if (this->effort != other.effort) {
      return false;
    }
    return true;
  }
  bool operator!=(const ArmPositionCmd_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ArmPositionCmd_

// alias to use template instance with default allocator
using ArmPositionCmd =
  robot_msgs::msg::ArmPositionCmd_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__STRUCT_HPP_
