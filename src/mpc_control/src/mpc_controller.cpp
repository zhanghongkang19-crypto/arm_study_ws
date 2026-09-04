#include <pinocchio/fwd.hpp>  // Pinocchio requires forward declarations first.

#include "mpc_control/mpc_controller.hpp"

#include "mpc_control/end_effector_pose_cost.hpp"
#include "mpc_control/pinocchio_arm_dynamics.hpp"

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/parsers/urdf.hpp>

#include <ocs2_core/control/ControllerBase.h>
#include <ocs2_core/initialization/DefaultInitializer.h>
#include <ocs2_ddp/DDP_Settings.h>
#include <ocs2_mpc/MPC_Settings.h>
#include <ocs2_oc/oc_data/PrimalSolution.h>
#include <ocs2_oc/rollout/TimeTriggeredRollout.h>
#include <ocs2_oc/rollout/RolloutSettings.h>

#include <Eigen/Geometry>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <stdexcept>
#include <unordered_map>
#include <utility>

namespace mpc_control
{
namespace
{

constexpr int kArmDof = 6;  // 当前控制器固定控制六个机械臂关节。

// 未配置 joint_names 时使用这组默认关节顺序。
std::array<std::string, kArmDof> defaultJointNames()
{
  return {"joint1", "joint2", "joint3", "joint4", "joint5", "joint6"};
}

ocs2::vector_t vectorFromParameter(
  const std::vector<double> & values,
  const std::string & parameter_name)
{
  // 把 ROS 参数数组转换为 OCS2/Eigen 向量。
  if (values.size() != kArmDof) {
    throw std::invalid_argument(parameter_name + " must contain 6 values.");
  }
  return Eigen::Map<const ocs2::vector_t>(values.data(), values.size());
}

}  // namespace

MpcController::MpcController(const rclcpp::NodeOptions & options)
: Node("mpc_controller", options),
  joint_names_(defaultJointNames()),
  desired_pose_(ocs2::vector_t::Zero(7))
{
  // 读取模型文件、OCS2 配置和控制器基础参数。
  urdf_file_ = declare_parameter<std::string>("urdf_file", "");
  task_file_ = declare_parameter<std::string>("task_file", "");
  end_effector_frame_ =
    declare_parameter<std::string>("end_effector_frame", "link6");
  base_frame_ = declare_parameter<std::string>("base_frame", "base_link");
  command_topic_ = declare_parameter<std::string>(
      "torque_command_topic", "/arm_effort_controller/commands");
  control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
  state_timeout_ = declare_parameter<double>("state_timeout", 0.2);

  // 关节名决定了状态读取和力矩发布的统一顺序。
  const auto configured_joint_names = declare_parameter<std::vector<std::string>>(
      "joint_names", std::vector<std::string>(joint_names_.begin(),
                                               joint_names_.end()));
  if (configured_joint_names.size() != kArmDof) {
    throw std::invalid_argument("joint_names must contain exactly 6 names.");
  }
  std::copy(configured_joint_names.begin(), configured_joint_names.end(),
            joint_names_.begin());

  // 力矩限制既用于优化中的软约束，也用于发布前的硬限幅。
  torque_limits_ = vectorFromParameter(
      declare_parameter<std::vector<double>>(
          "torque_limits", {27.0, 27.0, 27.0, 7.0, 7.0, 7.0}),
      "torque_limits");

  // 在启动阶段检查参数，避免进入控制循环后才发现配置错误。
  if (urdf_file_.empty() || task_file_.empty()) {
    throw std::invalid_argument(
        "Parameters 'urdf_file' and 'task_file' are required. Use the "
        "provided mpc_controller.launch.py.");
  }
  if (!std::filesystem::exists(urdf_file_)) {
    throw std::invalid_argument("URDF file does not exist: " + urdf_file_);
  }
  if (!std::filesystem::exists(task_file_)) {
    throw std::invalid_argument("OCS2 task file does not exist: " + task_file_);
  }
  if (control_frequency_ <= 0.0 || state_timeout_ <= 0.0) {
    throw std::invalid_argument(
        "control_frequency and state_timeout must be positive.");
  }

  // 先创建完整 MPC，再建立 ROS 通信接口。
  setupMpc();
  time_origin_ = now();

  // 接收关节反馈和 link6 目标位姿。
  joint_state_subscriber_ = create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", rclcpp::SensorDataQoS(),
      std::bind(&MpcController::jointStateCallback, this,
                std::placeholders::_1));
  target_pose_subscriber_ = create_subscription<geometry_msgs::msg::Pose>(
      "/target_pose", 1,
      std::bind(&MpcController::targetPoseCallback, this,
                std::placeholders::_1));
  // 发布控制器指令、调试力矩和当前末端位姿。
  torque_publisher_ =
    create_publisher<std_msgs::msg::Float64MultiArray>(command_topic_, 1);
  torque_diagnostic_publisher_ =
    create_publisher<sensor_msgs::msg::JointState>("/mpc_joint_torque", 1);
  pose_publisher_ =
    create_publisher<geometry_msgs::msg::PoseStamped>("/mpc_link6_pose", 1);

  // 定时器按 control_frequency_ 周期调用 MPC。
  control_timer_ = create_wall_timer(
      std::chrono::duration<double>(1.0 / control_frequency_),
      std::bind(&MpcController::controlCallback, this));

  RCLCPP_INFO(get_logger(),
              "OCS2 torque MPC ready: x=[q,dq](12), u=tau(6), EE=%s, "
              "command=%s",
              end_effector_frame_.c_str(), command_topic_.c_str());
  RCLCPP_INFO(get_logger(),
              "Waiting for /joint_states. /target_pose is optional at startup; "
              "the current link6 pose is held until a target arrives.");
}

void MpcController::setupMpc()
{
  // 1. 从 URDF 读取完整刚体模型。
  pinocchio::urdf::buildModel(urdf_file_, model_);
  if (model_.nq != kArmDof || model_.nv != kArmDof) {
    throw std::runtime_error(
        "The configured URDF must be a fixed-base 6-DoF arm. Got nq=" +
        std::to_string(model_.nq) + ", nv=" + std::to_string(model_.nv) +
        ". Use reBot-DevArm_fixend.urdf (the gripper joints must not be "
        "optimization states).");
  }
  if (!model_.existFrame(end_effector_frame_)) {
    throw std::runtime_error("URDF has no end-effector frame '" +
                             end_effector_frame_ + "'.");
  }
  // 保存末端 Frame 编号，后续正运动学可直接使用。
  frame_id_ = model_.getFrameId(end_effector_frame_);
  data_ = std::make_unique<pinocchio::Data>(model_);

  // 2. 从 ROS 参数读取每一项代价权重。
  ArmCostWeights weights;
  weights.position = declare_parameter<double>("weights.position", 120.0);
  weights.orientation = declare_parameter<double>("weights.orientation", 0.0);
  weights.joint_velocity =
    declare_parameter<double>("weights.joint_velocity", 0.5);
  weights.torque = declare_parameter<double>("weights.torque", 2.0e-3);
  weights.torque_limit =
    declare_parameter<double>("weights.torque_limit", 20.0);
  weights.joint_limit =
    declare_parameter<double>("weights.joint_limit", 30.0);
  weights.terminal_scale =
    declare_parameter<double>("weights.terminal_scale", 8.0);

  // 3. 组成最优控制问题：动力学、运行代价和终端代价。
  problem_.dynamicsPtr = std::make_unique<PinocchioArmDynamics>(model_);
  problem_.costPtr->add(
      "link6PoseAndTorque",
      std::make_unique<EndEffectorPoseCost>(
          model_, end_effector_frame_, weights, torque_limits_));
  problem_.finalCostPtr->add(
      "terminalLink6Pose",
      std::make_unique<EndEffectorTerminalCost>(
          model_, end_effector_frame_, weights));

  // 4. 读取 iLQR、MPC 和正向积分参数。
  auto ddp_settings = ocs2::ddp::loadSettings(task_file_, "ddp");
  auto mpc_settings = ocs2::mpc::loadSettings(task_file_, "mpc");
  const auto rollout_settings =
    ocs2::rollout::loadSettings(task_file_, "rollout");
  horizon_ = mpc_settings.timeHorizon_;
  if (!std::isfinite(horizon_) || horizon_ <= 0.0) {
    throw std::invalid_argument("The OCS2 MPC time horizon must be positive.");
  }

  // 5. 创建状态预测器、默认输入初值、参考管理器和 MPC 求解器。
  rollout_ = std::make_unique<ocs2::TimeTriggeredRollout>(
      *problem_.dynamicsPtr, rollout_settings);
  initializer_ = std::make_unique<ocs2::DefaultInitializer>(kArmDof);
  reference_manager_ = std::make_shared<ocs2::ReferenceManager>();
  mpc_ = std::make_unique<ocs2::GaussNewtonDDP_MPC>(
      mpc_settings, ddp_settings, *rollout_, problem_, *initializer_);
  mpc_->getSolverPtr()->setReferenceManager(reference_manager_);
}

void MpcController::jointStateCallback(
  const sensor_msgs::msg::JointState::SharedPtr message)
{
  // 消息至少要包含相同数量的关节名和关节角。
  if (message->name.empty() || message->position.size() != message->name.size()) {
    RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                         "Ignoring malformed /joint_states message.");
    return;
  }

  // 消息中的关节顺序可能变化，先建立“名称到下标”的索引。
  std::unordered_map<std::string, std::size_t> index_by_name;
  for (std::size_t index = 0; index < message->name.size(); ++index) {
    index_by_name[message->name[index]] = index;
  }
  // 按配置的 joint1...joint6 顺序保存位置和速度。
  for (int joint = 0; joint < kArmDof; ++joint) {
    const auto iterator = index_by_name.find(joint_names_[joint]);
    if (iterator == index_by_name.end()) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000,
                           "/joint_states is missing %s.",
                           joint_names_[joint].c_str());
      return;
    }
    const auto index = iterator->second;
    measured_position_[joint] = message->position[index];
    // 某些 JointState 不提供速度，此时暂时按零速度处理。
    measured_velocity_[joint] =
      index < message->velocity.size() ? message->velocity[index] : 0.0;
  }

  last_state_stamp_ = now();
  has_state_ = true;
}

void MpcController::targetPoseCallback(
  const geometry_msgs::msg::Pose::SharedPtr message)
{
  try {
    // 检查并归一化目标四元数，然后更新参考轨迹。
    desired_pose_ = poseMessageToVector(*message);
    has_target_ = true;
    if (has_state_) {
      setPoseReference(solverTime(now()), desired_pose_);
      reference_initialized_ = true;
    }
    RCLCPP_INFO(get_logger(), "New link6 target: p=[%.3f, %.3f, %.3f]",
                desired_pose_(0), desired_pose_(1), desired_pose_(2));
  } catch (const std::exception & error) {
    RCLCPP_ERROR(get_logger(), "Rejected /target_pose: %s", error.what());
  }
}

bool MpcController::buildState(ocs2::vector_t * state) const
{
  if (!has_state_) {
    return false;
  }
  // OCS2 状态固定排列为 [q1...q6, dq1...dq6]。
  state->resize(2 * kArmDof);
  for (int index = 0; index < kArmDof; ++index) {
    (*state)(index) = measured_position_[index];
    (*state)(kArmDof + index) = measured_velocity_[index];
  }
  return state->allFinite();
}

void MpcController::setPoseReference(
  double time,
  const ocs2::vector_t & desired_pose)
{
  // 用两个相同端点表示预测区间内不变化的目标位姿。
  const ocs2::scalar_array_t times{time, time + 2.0 * horizon_};
  const ocs2::vector_array_t poses{desired_pose, desired_pose};
  const ocs2::vector_array_t inputs{
    ocs2::vector_t::Zero(kArmDof), ocs2::vector_t::Zero(kArmDof)};
  reference_manager_->setTargetTrajectories(
      ocs2::TargetTrajectories(times, poses, inputs));
}

ocs2::vector_t MpcController::currentEndEffectorPose(
  const ocs2::vector_t & q)
{
  // 由关节角计算所有连杆位姿，再取出 link6。
  pinocchio::forwardKinematics(model_, *data_, q);
  const pinocchio::SE3 & placement =
    pinocchio::updateFramePlacement(model_, *data_, frame_id_);
  const Eigen::Quaterniond quaternion(placement.rotation());
  // 返回格式与参考轨迹一致：[位置, 四元数 wxyz]。
  ocs2::vector_t pose(7);
  pose << placement.translation(), quaternion.w(), quaternion.x(),
    quaternion.y(), quaternion.z();
  return pose;
}

void MpcController::controlCallback()
{
  const auto stamp = now();
  ocs2::vector_t state;

  // 尚未收到关节状态时不运行 MPC，也不发布无意义的力矩。
  if (!buildState(&state)) {
    return;
  }

  // 状态超时可能代表通信中断，此时输出零力矩作为安全回退。
  if ((stamp - last_state_stamp_).seconds() > state_timeout_) {
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "Joint state is stale; publishing zero torque.");
    publishTorque(ocs2::vector_t::Zero(kArmDof), stamp);
    return;
  }

  // 发布当前末端位姿，方便对比目标和实际位置。
  publishCurrentPose(state.head<6>(), stamp);

  // 如果还没有外部目标，就把当前位置作为目标，避免启动时突然运动。
  if (!reference_initialized_) {
    if (!has_target_) {
      desired_pose_ = currentEndEffectorPose(state.head<6>());
    }
    setPoseReference(solverTime(stamp), desired_pose_);
    reference_initialized_ = true;
  }

  try {
    const double current_time = solverTime(stamp);

    // 在 [当前时刻, 当前时刻+horizon] 上求解最优控制问题。
    if (!mpc_->run(current_time, state)) {
      throw std::runtime_error("OCS2 rejected the current MPC time window.");
    }

    // 取出本次优化得到的反馈控制器。
    ocs2::PrimalSolution solution;
    mpc_->getSolverPtr()->getPrimalSolution(
        mpc_->getSolverPtr()->getFinalTime(), &solution);
    if (!solution.controllerPtr_) {
      throw std::runtime_error("OCS2 returned no feedback controller.");
    }
    // 只执行优化轨迹的第一步力矩，下个周期会重新求解。
    ocs2::vector_t torque =
      solution.controllerPtr_->computeInput(current_time, state);
    if (torque.size() != kArmDof || !torque.allFinite()) {
      throw std::runtime_error("OCS2 returned an invalid torque vector.");
    }
    // 发布前再次硬限幅，确保指令不会超过配置值。
    torque = torque.cwiseMax(-torque_limits_).cwiseMin(torque_limits_);
    publishTorque(torque, stamp);
  } catch (const std::exception & error) {
    // 求解失败时记录错误并发布零力矩，避免沿用旧指令。
    RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 1000,
                          "MPC solve failed: %s. Publishing zero torque.",
                          error.what());
    publishTorque(ocs2::vector_t::Zero(kArmDof), stamp);
  }
}

void MpcController::publishTorque(
  const ocs2::vector_t & torque,
  const rclcpp::Time & stamp)
{
  // 这条消息直接发送给 ros2_control 的 effort 控制器。
  std_msgs::msg::Float64MultiArray command;
  command.data.assign(torque.data(), torque.data() + torque.size());
  torque_publisher_->publish(command);

  // 诊断话题带有关节名，使用 ros2 topic echo 时更容易阅读。
  sensor_msgs::msg::JointState diagnostic;
  diagnostic.header.stamp = stamp;
  diagnostic.name.assign(joint_names_.begin(), joint_names_.end());
  diagnostic.effort.assign(torque.data(), torque.data() + torque.size());
  torque_diagnostic_publisher_->publish(diagnostic);
}

void MpcController::publishCurrentPose(
  const ocs2::vector_t & q,
  const rclcpp::Time & stamp)
{
  // 将内部 7 维向量转换成标准 ROS PoseStamped 消息。
  const auto pose = currentEndEffectorPose(q);
  geometry_msgs::msg::PoseStamped message;
  message.header.stamp = stamp;
  message.header.frame_id = base_frame_;
  message.pose.position.x = pose(0);
  message.pose.position.y = pose(1);
  message.pose.position.z = pose(2);
  message.pose.orientation.w = pose(3);
  message.pose.orientation.x = pose(4);
  message.pose.orientation.y = pose(5);
  message.pose.orientation.z = pose(6);
  pose_publisher_->publish(message);
}

ocs2::vector_t MpcController::poseMessageToVector(
  const geometry_msgs::msg::Pose & pose)
{
  // 检查位置和四元数，拒绝 NaN、无穷大或零长度四元数。
  Eigen::Quaterniond quaternion(pose.orientation.w, pose.orientation.x,
    pose.orientation.y, pose.orientation.z);
  if (!std::isfinite(pose.position.x) ||
    !std::isfinite(pose.position.y) ||
    !std::isfinite(pose.position.z) || !quaternion.coeffs().allFinite() ||
    quaternion.norm() < 1.0e-8)
  {
    throw std::invalid_argument("position/quaternion contains invalid values");
  }
  // 归一化后再交给代价函数，避免旋转误差计算失真。
  quaternion.normalize();
  ocs2::vector_t result(7);
  result << pose.position.x, pose.position.y, pose.position.z, quaternion.w(),
    quaternion.x(), quaternion.y(), quaternion.z();
  return result;
}

double MpcController::solverTime(const rclcpp::Time & time) const
{
  // 使用较小的相对时间，避免绝对 Unix 时间影响数值精度。
  return (time - time_origin_).seconds();
}

}  // namespace mpc_control
