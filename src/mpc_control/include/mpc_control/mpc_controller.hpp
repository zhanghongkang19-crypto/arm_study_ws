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
 * @brief 使用 OCS2 求解关节力矩的 ROS 2 MPC 控制节点。
 *
 * 输入：
 *   /joint_states：六个关节的位置和速度。
 *   /target_pose：link6 在 base_link 坐标系中的目标位姿。
 *
 * 输出：
 *   /arm_effort_controller/commands：发送给力矩控制器的 6 个关节力矩。
 *   /mpc_joint_torque：带关节名称的力矩诊断消息。
 *   /mpc_link6_pose：根据当前关节角计算出的 link6 位姿。
 */
class MpcController final : public rclcpp::Node {
public:
  // 读取参数、创建 MPC，并建立 ROS 话题和控制定时器。
  explicit MpcController(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  // 从 URDF 和配置文件建立动力学、代价函数及 OCS2 求解器。
  void setupMpc();

  // 保存最新关节状态，并按 joint_names_ 统一关节顺序。
  void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr message);

  // 接收新的 link6 目标位姿，并更新 OCS2 参考轨迹。
  void targetPoseCallback(const geometry_msgs::msg::Pose::SharedPtr message);

  // 每个控制周期求解一次 MPC，然后发布第一步关节力矩。
  void controlCallback();

  // 把关节反馈拼成 OCS2 状态 x = [q, dq]。
  bool buildState(ocs2::vector_t * state) const;

  // 将目标位姿设置成预测时域内保持不变的参考轨迹。
  void setPoseReference(double time, const ocs2::vector_t & desired_pose);

  // 使用 Pinocchio 正运动学计算当前 link6 位姿。
  ocs2::vector_t currentEndEffectorPose(const ocs2::vector_t & q);

  // 发布控制力矩和便于观察的诊断消息。
  void publishTorque(
    const ocs2::vector_t & torque,
    const rclcpp::Time & stamp);
  // 发布当前 link6 位姿。
  void publishCurrentPose(
    const ocs2::vector_t & q,
    const rclcpp::Time & stamp);
  // 将 ROS Pose 转成 [px, py, pz, qw, qx, qy, qz]。
  static ocs2::vector_t poseMessageToVector(
    const geometry_msgs::msg::Pose & pose);
  // 将 ROS 绝对时间转换成从节点启动时刻开始的相对时间。
  double solverTime(const rclcpp::Time & time) const;

  // 文件、坐标系、关节名和话题等基础配置。
  std::string urdf_file_;
  std::string task_file_;
  std::string end_effector_frame_;
  std::string base_frame_;
  std::string command_topic_;
  std::array<std::string, 6> joint_names_;
  ocs2::vector_t torque_limits_;
  double control_frequency_;   // MPC 控制循环频率。
  double state_timeout_;       // 关节状态允许的最大延迟。
  double horizon_;             // MPC 预测时域长度。

  // Pinocchio 模型及正运动学计算数据。
  pinocchio::Model model_;
  std::unique_ptr<pinocchio::Data> data_;
  pinocchio::FrameIndex frame_id_;

  // OCS2 最优控制问题、仿真器、初值和求解器。
  ocs2::OptimalControlProblem problem_;
  std::unique_ptr<ocs2::RolloutBase> rollout_;
  std::unique_ptr<ocs2::Initializer> initializer_;
  std::shared_ptr<ocs2::ReferenceManager> reference_manager_;
  std::unique_ptr<ocs2::GaussNewtonDDP_MPC> mpc_;

  // 最近一次收到的机器人状态和目标状态。
  std::array<double, 6> measured_position_{};
  std::array<double, 6> measured_velocity_{};
  rclcpp::Time time_origin_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_state_stamp_{0, 0, RCL_ROS_TIME};
  ocs2::vector_t desired_pose_;
  bool has_state_ = false;              // 是否收到过完整关节状态。
  bool has_target_ = false;             // 是否收到过外部目标位姿。
  bool reference_initialized_ = false;  // OCS2 参考轨迹是否已初始化。

  // ROS 2 订阅器、发布器和控制定时器。
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr target_pose_subscriber_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr torque_publisher_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr torque_diagnostic_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_publisher_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace mpc_control
