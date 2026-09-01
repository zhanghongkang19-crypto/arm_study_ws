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
  double position = 120.0;
  double orientation = 0.0;
  double joint_velocity = 0.5;
  double torque = 2.0e-3;
  double torque_limit = 20.0;
  double joint_limit = 30.0;
  double terminal_scale = 8.0;
};

/** Running link6-pose, velocity, torque and soft-limit cost. */
class EndEffectorPoseCost final : public ocs2::StateInputCost {
public:
  EndEffectorPoseCost(
    const pinocchio::Model & model,
    const std::string & end_effector_frame,
    ArmCostWeights weights,
    ocs2::vector_t torque_limits);
  EndEffectorPoseCost(const EndEffectorPoseCost & other);

  EndEffectorPoseCost * clone() const override;

  ocs2::scalar_t getValue(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::vector_t & input,
    const ocs2::TargetTrajectories & target_trajectories,
    const ocs2::PreComputation & pre_computation) const override;

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::vector_t & input,
    const ocs2::TargetTrajectories & target_trajectories,
    const ocs2::PreComputation & pre_computation) const override;

private:
  Eigen::Matrix<double, 6, 1> poseResidual(
    const ocs2::vector_t & q,
    const ocs2::vector_t & desired_pose) const;
  Eigen::Matrix<double, 6, 6> poseResidualJacobian(
    const ocs2::vector_t & q,
    const ocs2::vector_t & desired_pose) const;
  ocs2::vector_t desiredPose(
    ocs2::scalar_t time,
    const ocs2::TargetTrajectories & target_trajectories) const;
  void addJointLimitCost(
    const ocs2::vector_t & q,
    ocs2::scalar_t * value,
    ocs2::vector_t * gradient,
    ocs2::matrix_t * hessian) const;
  void addTorqueLimitCost(
    const ocs2::vector_t & input,
    ocs2::scalar_t * value,
    ocs2::vector_t * gradient,
    ocs2::matrix_t * hessian) const;

  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  pinocchio::FrameIndex frame_id_;
  ArmCostWeights weights_;
  ocs2::vector_t torque_limits_;
};

/** Strong terminal link6-pose and zero joint-velocity cost. */
class EndEffectorTerminalCost final : public ocs2::StateCost {
public:
  EndEffectorTerminalCost(
    const pinocchio::Model & model,
    const std::string & end_effector_frame,
    ArmCostWeights weights);
  EndEffectorTerminalCost(const EndEffectorTerminalCost & other);

  EndEffectorTerminalCost * clone() const override;

  ocs2::scalar_t getValue(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::TargetTrajectories & target_trajectories,
    const ocs2::PreComputation & pre_computation) const override;

  ocs2::ScalarFunctionQuadraticApproximation getQuadraticApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::TargetTrajectories & target_trajectories,
    const ocs2::PreComputation & pre_computation) const override;

private:
  Eigen::Matrix<double, 6, 1> poseResidual(
    const ocs2::vector_t & q,
    const ocs2::vector_t & desired_pose) const;
  Eigen::Matrix<double, 6, 6> poseResidualJacobian(
    const ocs2::vector_t & q,
    const ocs2::vector_t & desired_pose) const;
  ocs2::vector_t desiredPose(
    ocs2::scalar_t time,
    const ocs2::TargetTrajectories & target_trajectories) const;

  pinocchio::Model model_;
  mutable pinocchio::Data data_;
  pinocchio::FrameIndex frame_id_;
  ArmCostWeights weights_;
};

}  // namespace mpc_control
