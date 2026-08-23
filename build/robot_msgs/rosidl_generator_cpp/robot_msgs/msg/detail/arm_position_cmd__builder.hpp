// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from robot_msgs:msg/ArmPositionCmd.idl
// generated code does not contain a copyright notice

// IWYU pragma: private, include "robot_msgs/msg/arm_position_cmd.hpp"


#ifndef ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__BUILDER_HPP_
#define ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "robot_msgs/msg/detail/arm_position_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace robot_msgs
{

namespace msg
{

namespace builder
{

class Init_ArmPositionCmd_effort
{
public:
  explicit Init_ArmPositionCmd_effort(::robot_msgs::msg::ArmPositionCmd & msg)
  : msg_(msg)
  {}
  ::robot_msgs::msg::ArmPositionCmd effort(::robot_msgs::msg::ArmPositionCmd::_effort_type arg)
  {
    msg_.effort = std::move(arg);
    return std::move(msg_);
  }

private:
  ::robot_msgs::msg::ArmPositionCmd msg_;
};

class Init_ArmPositionCmd_velocity
{
public:
  explicit Init_ArmPositionCmd_velocity(::robot_msgs::msg::ArmPositionCmd & msg)
  : msg_(msg)
  {}
  Init_ArmPositionCmd_effort velocity(::robot_msgs::msg::ArmPositionCmd::_velocity_type arg)
  {
    msg_.velocity = std::move(arg);
    return Init_ArmPositionCmd_effort(msg_);
  }

private:
  ::robot_msgs::msg::ArmPositionCmd msg_;
};

class Init_ArmPositionCmd_position
{
public:
  Init_ArmPositionCmd_position()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ArmPositionCmd_velocity position(::robot_msgs::msg::ArmPositionCmd::_position_type arg)
  {
    msg_.position = std::move(arg);
    return Init_ArmPositionCmd_velocity(msg_);
  }

private:
  ::robot_msgs::msg::ArmPositionCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::robot_msgs::msg::ArmPositionCmd>()
{
  return robot_msgs::msg::builder::Init_ArmPositionCmd_position();
}

}  // namespace robot_msgs

#endif  // ROBOT_MSGS__MSG__DETAIL__ARM_POSITION_CMD__BUILDER_HPP_
