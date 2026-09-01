#include <pinocchio/fwd.hpp>  // Pinocchio requires forward declarations first.

#include "mpc_control/end_effector_pose_cost.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/explog.hpp>

#include <Eigen/Geometry>

#include <cmath>
#include <initializer_list>
#include <stdexcept>
#include <utility>

namespace mpc_control
{
namespace
{

constexpr int kArmDof = 6;
constexpr int kStateDim = 12;
constexpr double kFiniteDifferenceStep = 1.0e-6;
constexpr double kHessianRegularization = 1.0e-8;

pinocchio::FrameIndex requireFrame(
  const pinocchio::Model & model,
  const std::string & frame_name)
{
  if (!model.existFrame(frame_name)) {
    throw std::invalid_argument("Pinocchio model has no frame named '" +
                                frame_name + "'.");
  }
  return model.getFrameId(frame_name);
}

ocs2::vector_t readDesiredPose(
  ocs2::scalar_t time,
  const ocs2::TargetTrajectories & target_trajectories)
{
  const auto desired = target_trajectories.getDesiredState(time);
  if (desired.size() != 7) {
    throw std::runtime_error(
        "The MPC reference must be [px,py,pz,qw,qx,qy,qz] (7 values).");
  }
  return desired;
}

Eigen::Matrix<double, 6, 1> calculatePoseResidual(
  const pinocchio::Model & model, pinocchio::Data & data,
  pinocchio::FrameIndex frame_id, const ocs2::vector_t & q,
  const ocs2::vector_t & desired_pose)
{
  pinocchio::forwardKinematics(model, data, q);
  const pinocchio::SE3 & world_to_ee =
    pinocchio::updateFramePlacement(model, data, frame_id);

  Eigen::Quaterniond desired_quaternion(
    desired_pose(3), desired_pose(4), desired_pose(5), desired_pose(6));
  if (desired_quaternion.norm() < 1.0e-8) {
    throw std::runtime_error("target_pose contains a zero-length quaternion.");
  }
  desired_quaternion.normalize();

  Eigen::Matrix<double, 6, 1> residual;
  residual.head<3>() = world_to_ee.translation() - desired_pose.head<3>();
  residual.tail<3>() = pinocchio::log3(
      desired_quaternion.toRotationMatrix().transpose() *
      world_to_ee.rotation());
  return residual;
}

template<typename ResidualFunction>
Eigen::Matrix<double, 6, 6> finiteDifferenceJacobian(
  const ocs2::vector_t & q, ResidualFunction && residual_function)
{
  Eigen::Matrix<double, 6, 6> jacobian;
  for (int index = 0; index < kArmDof; ++index) {
    ocs2::vector_t q_plus = q;
    ocs2::vector_t q_minus = q;
    q_plus(index) += kFiniteDifferenceStep;
    q_minus(index) -= kFiniteDifferenceStep;
    jacobian.col(index) =
      (residual_function(q_plus) - residual_function(q_minus)) /
      (2.0 * kFiniteDifferenceStep);
  }
  return jacobian;
}

Eigen::Matrix<double, 6, 6> poseWeightMatrix(
  const ArmCostWeights & weights, double scale = 1.0)
{
  Eigen::Matrix<double, 6, 6> matrix =
    Eigen::Matrix<double, 6, 6>::Zero();
  matrix.diagonal().head<3>().setConstant(scale * weights.position);
  matrix.diagonal().tail<3>().setConstant(scale * weights.orientation);
  return matrix;
}

void checkStateInput(
  const ocs2::vector_t & state,
  const ocs2::vector_t * input = nullptr)
{
  if (state.size() != kStateDim ||
    (input != nullptr && input->size() != kArmDof))
  {
    throw std::invalid_argument(
        "Pose cost expects a 12-element [q,dq] state and 6 torques.");
  }
}

void validateWeights(const ArmCostWeights & weights)
{
  const std::initializer_list<double> values{
    weights.position, weights.orientation, weights.joint_velocity,
    weights.torque, weights.torque_limit, weights.joint_limit,
    weights.terminal_scale};
  for (const double value : values) {
    if (!std::isfinite(value) || value < 0.0) {
      throw std::invalid_argument(
          "All MPC cost weights must be finite and non-negative.");
    }
  }
  if (weights.torque <= 0.0 || weights.terminal_scale <= 0.0) {
    throw std::invalid_argument(
        "weights.torque and weights.terminal_scale must be positive.");
  }
}

}  // namespace

EndEffectorPoseCost::EndEffectorPoseCost(
  const pinocchio::Model & model, const std::string & end_effector_frame,
  ArmCostWeights weights, ocs2::vector_t torque_limits)
: model_(model),
  data_(model_),
  frame_id_(requireFrame(model_, end_effector_frame)),
  weights_(weights),
  torque_limits_(std::move(torque_limits))
{
  validateWeights(weights_);
  if (torque_limits_.size() != kArmDof ||
    !torque_limits_.allFinite() || (torque_limits_.array() <= 0.0).any())
  {
    throw std::invalid_argument(
        "Six finite, positive torque limits are required.");
  }
}

EndEffectorPoseCost::EndEffectorPoseCost(const EndEffectorPoseCost & other)
: ocs2::StateInputCost(other),
  model_(other.model_),
  data_(model_),
  frame_id_(other.frame_id_),
  weights_(other.weights_),
  torque_limits_(other.torque_limits_) {}

EndEffectorPoseCost * EndEffectorPoseCost::clone() const
{
  return new EndEffectorPoseCost(*this);
}

ocs2::vector_t EndEffectorPoseCost::desiredPose(
  ocs2::scalar_t time,
  const ocs2::TargetTrajectories & target_trajectories) const
{
  return readDesiredPose(time, target_trajectories);
}

Eigen::Matrix<double, 6, 1> EndEffectorPoseCost::poseResidual(
  const ocs2::vector_t & q, const ocs2::vector_t & desired_pose) const
{
  return calculatePoseResidual(model_, data_, frame_id_, q, desired_pose);
}

Eigen::Matrix<double, 6, 6> EndEffectorPoseCost::poseResidualJacobian(
  const ocs2::vector_t & q, const ocs2::vector_t & desired_pose) const
{
  return finiteDifferenceJacobian(q, [&](const ocs2::vector_t & perturbed_q) {
             return poseResidual(perturbed_q, desired_pose);
  });
}

void EndEffectorPoseCost::addJointLimitCost(
  const ocs2::vector_t & q, ocs2::scalar_t * value,
  ocs2::vector_t * gradient, ocs2::matrix_t * hessian) const
{
  for (int index = 0; index < kArmDof; ++index) {
    double violation = 0.0;
    double sign = 0.0;
    if (q(index) < model_.lowerPositionLimit(index)) {
      violation = model_.lowerPositionLimit(index) - q(index);
      sign = -1.0;
    } else if (q(index) > model_.upperPositionLimit(index)) {
      violation = q(index) - model_.upperPositionLimit(index);
      sign = 1.0;
    }
    if (violation > 0.0) {
      *value += 0.5 * weights_.joint_limit * violation * violation;
      if (gradient != nullptr) {
        (*gradient)(index) += weights_.joint_limit * violation * sign;
        (*hessian)(index, index) += weights_.joint_limit;
      }
    }
  }
}

void EndEffectorPoseCost::addTorqueLimitCost(
  const ocs2::vector_t & input, ocs2::scalar_t * value,
  ocs2::vector_t * gradient, ocs2::matrix_t * hessian) const
{
  for (int index = 0; index < kArmDof; ++index) {
    const double violation = std::abs(input(index)) - torque_limits_(index);
    if (violation > 0.0) {
      const double sign = std::copysign(1.0, input(index));
      *value += 0.5 * weights_.torque_limit * violation * violation;
      if (gradient != nullptr) {
        (*gradient)(index) += weights_.torque_limit * violation * sign;
        (*hessian)(index, index) += weights_.torque_limit;
      }
    }
  }
}

ocs2::scalar_t EndEffectorPoseCost::getValue(
  ocs2::scalar_t time, const ocs2::vector_t & state,
  const ocs2::vector_t & input,
  const ocs2::TargetTrajectories & target_trajectories,
  const ocs2::PreComputation & /*pre_computation*/) const
{
  checkStateInput(state, &input);
  const auto residual = poseResidual(state.head<6>(),
                                     desiredPose(time, target_trajectories));
  ocs2::scalar_t value =
    0.5 * residual.dot(poseWeightMatrix(weights_) * residual) +
    0.5 * weights_.joint_velocity * state.tail<6>().squaredNorm() +
    0.5 * weights_.torque * input.squaredNorm();
  addJointLimitCost(state.head<6>(), &value, nullptr, nullptr);
  addTorqueLimitCost(input, &value, nullptr, nullptr);
  return value;
}

ocs2::ScalarFunctionQuadraticApproximation
EndEffectorPoseCost::getQuadraticApproximation(
  ocs2::scalar_t time, const ocs2::vector_t & state,
  const ocs2::vector_t & input,
  const ocs2::TargetTrajectories & target_trajectories,
  const ocs2::PreComputation & /*pre_computation*/) const
{
  checkStateInput(state, &input);
  const auto desired_pose = desiredPose(time, target_trajectories);
  const auto residual = poseResidual(state.head<6>(), desired_pose);
  const auto jacobian = poseResidualJacobian(state.head<6>(), desired_pose);
  const auto pose_weight = poseWeightMatrix(weights_);

  ocs2::ScalarFunctionQuadraticApproximation approximation;
  approximation.f =
    0.5 * residual.dot(pose_weight * residual) +
    0.5 * weights_.joint_velocity * state.tail<6>().squaredNorm() +
    0.5 * weights_.torque * input.squaredNorm();
  approximation.dfdx = ocs2::vector_t::Zero(kStateDim);
  approximation.dfdu = weights_.torque * input;
  approximation.dfdxx = ocs2::matrix_t::Zero(kStateDim, kStateDim);
  approximation.dfduu =
    weights_.torque * ocs2::matrix_t::Identity(kArmDof, kArmDof);
  approximation.dfdux = ocs2::matrix_t::Zero(kArmDof, kStateDim);

  approximation.dfdx.head<6>() = jacobian.transpose() * pose_weight * residual;
  approximation.dfdx.tail<6>() =
    weights_.joint_velocity * state.tail<6>();
  approximation.dfdxx.topLeftCorner<6, 6>() =
    jacobian.transpose() * pose_weight * jacobian;
  approximation.dfdxx.bottomRightCorner<6, 6>().diagonal().setConstant(
      weights_.joint_velocity);

  addJointLimitCost(state.head<6>(), &approximation.f,
                    &approximation.dfdx, &approximation.dfdxx);
  addTorqueLimitCost(input, &approximation.f, &approximation.dfdu,
                     &approximation.dfduu);
  approximation.dfdxx =
    0.5 * (approximation.dfdxx + approximation.dfdxx.transpose());
  approximation.dfdxx.diagonal().array() += kHessianRegularization;
  approximation.dfduu =
    0.5 * (approximation.dfduu + approximation.dfduu.transpose());
  approximation.dfduu.diagonal().array() += kHessianRegularization;
  return approximation;
}

EndEffectorTerminalCost::EndEffectorTerminalCost(
  const pinocchio::Model & model, const std::string & end_effector_frame,
  ArmCostWeights weights)
: model_(model),
  data_(model_),
  frame_id_(requireFrame(model_, end_effector_frame)),
  weights_(weights)
{
  validateWeights(weights_);
}

EndEffectorTerminalCost::EndEffectorTerminalCost(
  const EndEffectorTerminalCost & other)
: ocs2::StateCost(other),
  model_(other.model_),
  data_(model_),
  frame_id_(other.frame_id_),
  weights_(other.weights_) {}

EndEffectorTerminalCost * EndEffectorTerminalCost::clone() const
{
  return new EndEffectorTerminalCost(*this);
}

ocs2::vector_t EndEffectorTerminalCost::desiredPose(
  ocs2::scalar_t time,
  const ocs2::TargetTrajectories & target_trajectories) const
{
  return readDesiredPose(time, target_trajectories);
}

Eigen::Matrix<double, 6, 1> EndEffectorTerminalCost::poseResidual(
  const ocs2::vector_t & q, const ocs2::vector_t & desired_pose) const
{
  return calculatePoseResidual(model_, data_, frame_id_, q, desired_pose);
}

Eigen::Matrix<double, 6, 6>
EndEffectorTerminalCost::poseResidualJacobian(
  const ocs2::vector_t & q, const ocs2::vector_t & desired_pose) const
{
  return finiteDifferenceJacobian(q, [&](const ocs2::vector_t & perturbed_q) {
             return poseResidual(perturbed_q, desired_pose);
  });
}

ocs2::scalar_t EndEffectorTerminalCost::getValue(
  ocs2::scalar_t time, const ocs2::vector_t & state,
  const ocs2::TargetTrajectories & target_trajectories,
  const ocs2::PreComputation & /*pre_computation*/) const
{
  checkStateInput(state);
  const auto residual = poseResidual(state.head<6>(),
                                     desiredPose(time, target_trajectories));
  return 0.5 * residual.dot(
                   poseWeightMatrix(weights_, weights_.terminal_scale) *
                   residual) +
         0.5 * weights_.terminal_scale * weights_.joint_velocity *
         state.tail<6>().squaredNorm();
}

ocs2::ScalarFunctionQuadraticApproximation
EndEffectorTerminalCost::getQuadraticApproximation(
  ocs2::scalar_t time, const ocs2::vector_t & state,
  const ocs2::TargetTrajectories & target_trajectories,
  const ocs2::PreComputation & /*pre_computation*/) const
{
  checkStateInput(state);
  const auto desired_pose = desiredPose(time, target_trajectories);
  const auto residual = poseResidual(state.head<6>(), desired_pose);
  const auto jacobian = poseResidualJacobian(state.head<6>(), desired_pose);
  const auto pose_weight =
    poseWeightMatrix(weights_, weights_.terminal_scale);
  const double velocity_weight =
    weights_.terminal_scale * weights_.joint_velocity;

  ocs2::ScalarFunctionQuadraticApproximation approximation;
  approximation.f = 0.5 * residual.dot(pose_weight * residual) +
    0.5 * velocity_weight *
    state.tail<6>().squaredNorm();
  approximation.dfdx = ocs2::vector_t::Zero(kStateDim);
  approximation.dfdu.resize(0);
  approximation.dfdxx = ocs2::matrix_t::Zero(kStateDim, kStateDim);
  approximation.dfduu.resize(0, 0);
  approximation.dfdux.resize(0, kStateDim);

  approximation.dfdx.head<6>() = jacobian.transpose() * pose_weight * residual;
  approximation.dfdx.tail<6>() = velocity_weight * state.tail<6>();
  approximation.dfdxx.topLeftCorner<6, 6>() =
    jacobian.transpose() * pose_weight * jacobian;
  approximation.dfdxx.bottomRightCorner<6, 6>().diagonal().setConstant(
      velocity_weight);
  approximation.dfdxx =
    0.5 * (approximation.dfdxx + approximation.dfdxx.transpose());
  approximation.dfdxx.diagonal().array() += kHessianRegularization;
  return approximation;
}

}  // namespace mpc_control
