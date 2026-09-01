# reBot OCS2 动力学力矩 MPC Demo

这个包实现了一个写在 `MpcController` 内的 6 自由度机械臂 NMPC：

- 状态：`x = [q, dq]`，共 12 维；
- 控制输入：`u = tau`，直接优化并输出 6 个关节力矩；
- 动力学：从 URDF 惯性参数构造 Pinocchio 刚体模型，使用 ABA 计算完整正向动力学；
- MPC 求解器：OCS2 `GaussNewtonDDP_MPC`（iLQR）；
- 目标：默认最小化 `link6` 与 `/target_pose` 的位置误差，同时惩罚关节速度和力矩；姿态跟踪可通过权重开启；
- 安全处理：优化代价中包含关节/力矩软限位，最终发布前再次做硬限幅；状态超时或求解失败时发布零力矩。

动力学不是双积分器近似，而是：

```text
M(q) ddq + C(q,dq)dq + g(q) = tau

x_dot = [dq, ABA(q,dq,tau)]
```

`PinocchioArmDynamics` 使用 `computeABADerivatives()` 给 OCS2 提供 `df/dx` 和 `df/du`。`EndEffectorPoseCost` 对 `link6` 位姿残差做 Gauss-Newton 二次近似。

## 代码结构

```text
include/mpc_control/
  pinocchio_arm_dynamics.hpp   完整刚体动力学
  end_effector_pose_cost.hpp   link6 任务空间代价
  mpc_controller.hpp           ROS 2 + OCS2 闭环控制器
src/
  pinocchio_arm_dynamics.cpp
  end_effector_pose_cost.cpp
  mpc_controller.cpp
  mpc_controller_node.cpp
config/
  task.info                    OCS2 iLQR/MPC/rollout 参数
  mpc_controller.yaml         权重、关节名和力矩限制
  effort_controller.yaml      ros2_control 力矩控制器示例
```

## ROS 接口

输入：

- `/joint_states` (`sensor_msgs/msg/JointState`)：必须包含 `joint1` 到 `joint6` 的位置；应提供速度。
- `/target_pose` (`geometry_msgs/msg/Pose`)：目标位姿，坐标系约定为 `base_link`。

输出：

- `/arm_effort_controller/commands` (`std_msgs/msg/Float64MultiArray`)：按 `joint1 ... joint6` 排列的关节力矩；
- `/mpc_joint_torque` (`sensor_msgs/msg/JointState`)：同一力矩的带名称诊断消息；
- `/mpc_link6_pose` (`geometry_msgs/msg/PoseStamped`)：Pinocchio 计算的当前 `link6` 位姿。

## 依赖与构建

这个工作区是 ROS 2 Jazzy。本 demo 已使用
[`legubiao/ocs2_ros2`](https://github.com/legubiao/ocs2_ros2) 的 `ros2`
分支完成编译和闭环测试。需要先安装/编译该 OCS2 分支，并保证下列包能够被
`find_package()` 找到：

```text
ocs2_core  ocs2_oc  ocs2_mpc  ocs2_ddp
```

Pinocchio 也必须可用。然后在工作区根目录构建：

```bash
colcon build --packages-select mpc_control --symlink-install
source install/setup.bash
```

启动控制器：

```bash
ros2 launch mpc_control mpc_controller.launch.py
```

仓库已有的交互 Marker 会发布 `/target_pose`：

```bash
ros2 run rebot_description target_marker
```

也可以手动发布一次目标：

```bash
ros2 topic pub --once /target_pose geometry_msgs/msg/Pose \
  "{position: {x: 0.30, y: 0.00, z: 0.35}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}"
```

检查力矩：

```bash
ros2 topic echo /mpc_joint_torque
```

## 接到 ros2_control 前必须确认

当前 MoveIt 配置中的 `arm_controller` 是位置轨迹控制器，不能与力矩控制器同时占用六个关节。使用本 demo 驱动硬件/仿真时：

1. 六个关节的 `<ros2_control>` 配置必须暴露 `effort` command interface；
2. 使用 `config/effort_controller.yaml` 中的 `arm_effort_controller`；
3. 不要同时加载原来的 `arm_controller`；
4. 先悬空/仿真验证重力方向、关节顺序和力矩符号，再连接实机；
5. 实机侧还应有独立急停、通信看门狗和驱动器力矩限制。

提供的默认 URDF 是 `reBot-DevArm_fixend.urdf`。它只有六个可动关节，且包含惯性参数。带夹爪的 URDF 还有两个棱柱关节，不应直接作为本 12 状态 MPC 的模型；如需使用夹爪模型，应在建模时锁定夹爪关节。

## 调参

主要权重在 `config/mpc_controller.yaml`：

- `weights.position`：增大后末端位置跟踪更强；
- `weights.orientation`：默认为 `0.0`，设为正数可同时跟踪姿态；
- `weights.torque`：增大后输出更平滑/保守；
- `weights.joint_velocity`：增大后更倾向于静止到达目标；
- `weights.terminal_scale`：终端目标权重倍率。

预测时域、iLQR 迭代数与积分步长在 `config/task.info`。第一次运行建议保持较低力矩限制，从近距离目标开始测试。
