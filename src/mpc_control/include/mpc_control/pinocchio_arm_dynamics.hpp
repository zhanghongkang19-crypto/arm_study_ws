#pragma once

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/multibody/model.hpp>

#include <ocs2_core/dynamics/SystemDynamicsBase.h>

namespace mpc_control
{

/**
 * @brief 提供给 OCS2 的六自由度机械臂刚体动力学模型。
 *
 * 状态 x = [q, dq]：前 6 维是关节角，后 6 维是关节速度。
 * 输入 u = tau：6 个关节力矩。
 *
 * 动力学方程：
 *   q_dot  = dq
 *   dq_dot = M(q)^-1 * (tau - C(q,dq)dq - g(q))
 */
class PinocchioArmDynamics final : public ocs2::SystemDynamicsBase {
public:
  // 保存一份 Pinocchio 模型，并为动力学计算创建缓存数据。
  explicit PinocchioArmDynamics(const pinocchio::Model & model);

  // OCS2 会复制动力学对象，因此需要提供拷贝构造函数。
  PinocchioArmDynamics(const PinocchioArmDynamics & other);

  // 返回当前动力学对象的独立副本。
  PinocchioArmDynamics * clone() const override;

  // 计算连续时间状态导数 x_dot = [dq, ddq]。
  ocs2::vector_t computeFlowMap(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::vector_t & input,
    const ocs2::PreComputation & pre_computation) override;

  // 计算动力学及其一阶导数，供 iLQR 在当前轨迹附近线性化。
  ocs2::VectorFunctionLinearApproximation linearApproximation(
    ocs2::scalar_t time,
    const ocs2::vector_t & state,
    const ocs2::vector_t & input,
    const ocs2::PreComputation & pre_computation) override;

private:
  // 检查状态和力矩向量的维度，避免 Eigen 在错误尺寸上运算。
  void checkDimensions(
    const ocs2::vector_t & state,
    const ocs2::vector_t & input) const;

  pinocchio::Model model_;  // 由 URDF 建立的机械臂模型。
  pinocchio::Data data_;    // Pinocchio 算法重复使用的计算缓存。
};

}  // namespace mpc_control
