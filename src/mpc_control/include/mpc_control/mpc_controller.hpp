#pragma once

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/initialization/Initializer.h>
#include <ocs2_ddp/GaussNewtonDDP_MPC.h>
#include <ocs2_oc/oc_problem/OptimalControlProblem.h>
#include <ocs2_oc/rollout/RolloutBase.h>
#include <ocs2_oc/synchronized_module/ReferenceManager.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

namespace mpc_control
{

/**
 * ROS 2 wrapper around the OCS2 torque NMPC.
 *
 * Subscriptions:
 *   /joint_states : q and dq feedback
 *   /target_pose  : desired link6 pose in base_link
 *
 * Publications:
 *   /arm_effort_controller/commands : [tau1 ... tau6]
 *   /mpc_joint_torque               : named torque diagnostic
 *   /mpc_link6_pose                 : current Pinocchio FK pose
 */
class MpcController final : public rclcpp::Node {
public:
  explicit MpcController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void setupMpc();
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message);
  void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr message);
  void controlCallback();

  bool buildState(ocs2::vector_t * state) const;
  void setPoseReference(double time, const ocs2::vector_t & desired_pose);
  ocs2::vector_t currentEndEffectorPose(const ocs2::vector_t & q);
  void publishTorque(
    const ocs2::vector_t & torque,
    const rclcpp::Time & stamp);
  void publishCurrentPose(
    const ocs2::vector_t & q,
    const rclcpp::Time & stamp);
  static ocs2::vector_t poseMessageToVector(
    const geometry_msgs::msg::Pose & pose);
  double solverTime(const rclcpp::Time & time) const;

  std::string urdf_file_;
  std::string task_file_;
  std::string end_effector_frame_;
  std::string base_frame_;
  std::string command_topic_;
  std::array<std::string, 6> joint_names_;
  ocs2::vector_t torque_limits_;
  double control_frequency_;
  double state_timeout_;
  double horizon_;

  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex frame_id_;

  ocs2::OptimalControlProblem problem_;
  std::unique_ptr<ocs2::RolloutBase> rollout_;
  std::unique_ptr<ocs2::Initializer> initializer_;
  std::shared_ptr<ocs2::ReferenceManager> reference_manager_;
  std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;

  std::array<double, 6> measured_position_{};
  std::array<double, 6> measured_velocity_{};
  rclcpp::Time time_origin_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_state_stamp_{0, 0, RCL_ROS_TIME};
  ocs2::vector_t desired_pose_;
  bool has_state_ = false;
  bool has_target_ = false;
  bool reference_initialized_ = false;

  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_diagnostic_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mpc_control
