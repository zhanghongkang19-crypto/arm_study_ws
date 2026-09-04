#pragma once

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/cost/StateCost.h>
#include <ocs2_core/cost/StateInputCost.h>

#include <string>

namespace mpc_control
{

struct ArmCostWeights
{
  double position = 120.0;       // link6 位置误差权重。
  double orientation = 0.0;      // link6 姿态误差权重，0 表示不跟踪姿态。
  double joint_velocity = 0.5;   // 关节速度权重，用于抑制快速运动。
  double torque = 2.0e-3;        // 力矩权重，用于抑制过大的控制输入。
  double torque_limit = 20.0;    // 超出力矩限制后的软惩罚权重。
  double joint_limit = 30.0;     // 超出关节角限制后的软惩罚权重。
  double terminal_scale = 8.0;   // 预测终点代价的放大倍数。
};

/**
 * @brief MPC 预测过程中的运行代价。
 *
 * 同时考虑 link6 位姿误差、关节速度、关节力矩以及软限位。
 */
class EndEffectorPoseCost final : public ocs2::StateInputCost {
public:
  EndEffectorPoseCost(
    const pinocchio::Model & model,
    const std::string & end_effector_frame,
    ArmCostWeights weights,
    ocs2::vector_t torque_limits);
  EndEffectorPoseCost(const EndEffectorPoseCost & other);

  // OCS2 多线程求解时通过 clone() 创建互不干扰的代价对象。
  EndEffectorPoseCost * clone() const override;

  // 返回当前状态和输入对应的标量代价值。
  ocs2::scalar_t getValue(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::vector_t & input,
    const ocs2::TargetTrajectories & target_trajectories,
    const ocs2::PreComputation & pre_computation) const override;

  // 返回代价的一阶、二阶近似，供 iLQR 反向求解控制律。
  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::vector_t & input,
    const ocs2::TargetTrajectories & target_trajectories,
    const ocs2::PreComputation & pre_computation) const override;

private:
  // 计算当前 link6 位姿与目标位姿之间的 6 维误差。
  Eigen::Matrix<double, 6, 1> poseResidual(
    const ocs2::vector_t & q,
    const ocs2::vector_t & desired_pose) const;
  // 计算位姿误差对 6 个关节角的 Jacobian。
  Eigen::Matrix<double, 6, 6> poseResidualJacobian(
    const ocs2::vector_t & q,
    const ocs2::vector_t & desired_pose) const;
  // 从 OCS2 目标轨迹中读取当前时刻的目标位姿。
  ocs2::vector_t desiredPose(
    ocs2::scalar_t time,
    const ocs2::TargetTrajectories & target_trajectories) const;
  // 当关节角越界时，累加软限位代价及其导数。
  void addJointLimitCost(
    const ocs2::vector_t & q,
    ocs2::scalar_t * value,
    ocs2::vector_t * gradient,
    ocs2::matrix_t * hessian) const;
  // 当力矩越界时，累加软限位代价及其导数。
  void addTorqueLimitCost(
    const ocs2::vector_t & input,
    ocs2::scalar_t * value,
    ocs2::vector_t * gradient,
    ocs2::matrix_t * hessian) const;

  pinocchio::Model model_;           // 机械臂模型。
  mutable pinocchio::Data data_;     // 正运动学计算缓存。
  pinocchio::FrameIndex frame_id_;   // link6 在 Pinocchio 中的编号。
  ArmCostWeights weights_;           // 各项代价权重。
  ocs2::vector_t torque_limits_;     // 六个关节的力矩限制。
};

/**
 * @brief MPC 预测终点的代价。
 *
 * 在预测终点更强地要求 link6 到达目标，同时让关节速度接近零。
 */
class EndEffectorTerminalCost final : public ocs2::StateCost {
public:
  EndEffectorTerminalCost(
    const pinocchio::Model & model,
    const std::string & end_effector_frame,
    ArmCostWeights weights);
  EndEffectorTerminalCost(const EndEffectorTerminalCost & other);

  // 返回当前终端代价对象的独立副本。
  EndEffectorTerminalCost * clone() const override;

  // 计算预测终点的代价值。
  ocs2::scalar_t getValue(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::TargetTrajectories & target_trajectories,
    const ocs2::PreComputation & pre_computation) const override;

  // 计算终端代价的一阶、二阶近似。
  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::TargetTrajectories & target_trajectories,
    const ocs2::PreComputation & pre_computation) const override;

private:
  // 以下三个辅助函数与运行代价中的含义相同。
  Eigen::Matrix<double, 6, 1> poseResidual(
    const ocs2::vector_t & q,
    const ocs2::vector_t & desired_pose) const;
  Eigen::Matrix<double, 6, 6> poseResidualJacobian(
    const ocs2::vector_t & q,
    const ocs2::vector_t & desired_pose) const;
  ocs2::vector_t desiredPose(
    ocs2::scalar_t time,
    const ocs2::TargetTrajectories & target_trajectories) const;

  pinocchio::Model model_;           // 机械臂模型。
  mutable pinocchio::Data data_;     // 正运动学计算缓存。
  pinocchio::FrameIndex frame_id_;   // link6 的 Frame 编号。
  ArmCostWeights weights_;           // 终端代价使用的权重。
};

}  // namespace mpc_control
