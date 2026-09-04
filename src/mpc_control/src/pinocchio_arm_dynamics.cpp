#include <pinocchio/fwd.hpp>  // Pinocchio requires forward declarations first.

#include "mpc_control/pinocchio_arm_dynamics.hpp"

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include <stdexcept>

namespace mpc_control
{

// 创建模型对应的 Data。Data 保存中间结果，可减少循环中的重复分配。
PinocchioArmDynamics::PinocchioArmDynamics(const pinocchio::Model & model)
: model_(model), data_(model_)
{
  if (model_.nq != 6 || model_.nv != 6) {
    throw std::invalid_argument(
        "PinocchioArmDynamics expects a fixed-base 6-DoF model (nq=nv=6).");
  }
}

PinocchioArmDynamics::PinocchioArmDynamics(
  const PinocchioArmDynamics & other)
: ocs2::SystemDynamicsBase(other), model_(other.model_), data_(model_) {}

// OCS2 的每个求解线程都需要自己的 Data，因此这里进行深拷贝。
PinocchioArmDynamics * PinocchioArmDynamics::clone() const
{
  return new PinocchioArmDynamics(*this);
}

void PinocchioArmDynamics::checkDimensions(
  const ocs2::vector_t & state, const ocs2::vector_t & input) const
{
  if (state.size() != 12 || input.size() != 6) {
    throw std::invalid_argument(
        "Arm dynamics expects state [q,dq] with 12 elements and 6 torques.");
  }
}

ocs2::vector_t PinocchioArmDynamics::computeFlowMap(
  ocs2::scalar_t /*time*/, const ocs2::vector_t & state,
  const ocs2::vector_t & input,
  const ocs2::PreComputation & /*pre_computation*/)
{
  checkDimensions(state, input);

  // 状态前半部分是关节角 q，后半部分是关节速度 dq。
  const auto q = state.head<6>();
  const auto dq = state.tail<6>();

  // ABA 根据完整刚体动力学计算关节加速度 ddq。
  const ocs2::vector_t ddq = pinocchio::aba(model_, data_, q, dq, input);

  // 连续时间状态导数按 [dq, ddq] 排列。
  ocs2::vector_t flow_map(12);
  flow_map << dq, ddq;
  return flow_map;
}

ocs2::VectorFunctionLinearApproximation
PinocchioArmDynamics::linearApproximation(
  ocs2::scalar_t /*time*/, const ocs2::vector_t & state,
  const ocs2::vector_t & input,
  const ocs2::PreComputation & /*pre_computation*/)
{
  checkDimensions(state, input);

  const auto q = state.head<6>();
  const auto dq = state.tail<6>();

  // 先计算动力学，再计算 ddq 对 q、dq 和 tau 的解析导数。
  const ocs2::vector_t ddq = pinocchio::aba(model_, data_, q, dq, input);
  pinocchio::computeABADerivatives(model_, data_, q, dq, input);

  // approximation.f 是当前 x_dot，dfdx 和 dfdu 是线性化矩阵。
  ocs2::VectorFunctionLinearApproximation approximation;
  approximation.f.resize(12);
  approximation.f << dq, ddq;
  approximation.dfdx = ocs2::matrix_t::Zero(12, 12);
  approximation.dfdu = ocs2::matrix_t::Zero(12, 6);

  // q_dot=dq，所以对应位置是单位阵；下半部分来自 ABA 导数。
  approximation.dfdx.topRightCorner<6, 6>().setIdentity();
  approximation.dfdx.bottomLeftCorner<6, 6>() = data_.ddq_dq;
  approximation.dfdx.bottomRightCorner<6, 6>() = data_.ddq_dv;
  approximation.dfdu.bottomRows<6>() = data_.Minv;
  return approximation;
}

}  // namespace mpc_control
