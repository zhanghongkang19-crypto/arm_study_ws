#pragma once

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace mpc_control
{

/**
 * Full rigid-body arm dynamics used by OCS2.
 *
 * State: x = [q(6), dq(6)]
 * Input: u = tau(6)
 * Flow map:
 *   q_dot  = dq
 *   dq_dot = M(q)^-1 (tau - C(q,dq)dq - g(q))
 *
 * Pinocchio's ABA evaluates the flow map, and computeABADerivatives provides
 * the exact first-order model required by SLQ/iLQR.
 */
class PinocchioArmDynamics final : public ocs2::SystemDynamicsBase {
public:
  explicit PinocchioArmDynamics(const pinocchio::Model & model);
  PinocchioArmDynamics(const PinocchioArmDynamics & other);

  PinocchioArmDynamics * clone() const override;

  ocs2::vector_t computeFlowMap(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::vector_t & input,
    const ocs2::PreComputation & pre_computation) override;

  ocs2::VectorFunctionLinearApproximation linearApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::vector_t & input,
    const ocs2::PreComputation & pre_computation) override;

private:
  void checkDimensions(
    const ocs2::vector_t & state,
    const ocs2::vector_t & input) const;

  pinocchio::Model model_;
  pinocchio::Data data_;
};

}  // namespace mpc_control
