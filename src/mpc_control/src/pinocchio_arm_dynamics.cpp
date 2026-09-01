#include <pinocchio/fwd.hpp>  // Pinocchio requires forward declarations first.

#include "mpc_control/pinocchio_arm_dynamics.hpp"

#include <pinocchio/algorithm/aba-derivatives.hpp>
#include <pinocchio/algorithm/aba.hpp>

#include <stdexcept>

namespace mpc_control
{

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

  const auto q = state.head<6>();
  const auto dq = state.tail<6>();
  const ocs2::vector_t ddq = pinocchio::aba(model_, data_, q, dq, input);

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
  const ocs2::vector_t ddq = pinocchio::aba(model_, data_, q, dq, input);
  pinocchio::computeABADerivatives(model_, data_, q, dq, input);

  ocs2::VectorFunctionLinearApproximation approximation;
  approximation.f.resize(12);
  approximation.f << dq, ddq;
  approximation.dfdx = ocs2::matrix_t::Zero(12, 12);
  approximation.dfdu = ocs2::matrix_t::Zero(12, 6);

  approximation.dfdx.topRightCorner<6, 6>().setIdentity();
  approximation.dfdx.bottomLeftCorner<6, 6>() = data_.ddq_dq;
  approximation.dfdx.bottomRightCorner<6, 6>() = data_.ddq_dv;
  approximation.dfdu.bottomRows<6>() = data_.Minv;
  return approximation;
}

}  // namespace mpc_control
