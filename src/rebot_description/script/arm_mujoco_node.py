#!/usr/bin/env python3
import os
import threading
import time
import mujoco
import mujoco.viewer
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from robot_msgs.msg import ArmPositionCmd


class RebotMujocoNode(Node):

  def __init__(self):
    super().__init__('rebot_mujoco_node')

    # 1. 加载 MuJoCo 模型
    # 自动定位包目录下的 XML 模型文件
    package_dir = os.path.dirname(
        os.path.dirname(os.path.abspath(__file__))
    )  # 获取源码路径或安装路径
    xml_path = os.path.join(package_dir, 'urdf', 'reBot_B601_DM_with_gripper.xml')

    # 如果是从 colcon install 运行，尝试寻找 share 目录
    if not os.path.exists(xml_path):
      from ament_index_python.packages import get_package_share_directory

      xml_path = os.path.join(
          get_package_share_directory('rebot_description'),
          'urdf',
          'reBot_B601_DM_with_gripper.xml',
      )

    self.get_logger().info(f'Loading MuJoCo model from: {xml_path}')
    self.model = mujoco.MjModel.from_xml_path(xml_path)
    self.data = mujoco.MjData(self.model)

    # 2. 状态与控制初始化
    self.num_actuators = self.model.nu
    self.num_joints = self.model.njnt

    # 获取关节名称
    self.joint_names = [
        mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_JOINT, i)
        for i in range(self.num_joints)
    ]
    self.get_logger().info(
        f'Loaded joints ({self.num_joints}): {self.joint_names}'
    )

    self.cmd_lock = threading.Lock()
    self.ctrl_cmd = np.zeros(self.num_actuators)

    self.joint_map = []
    for name in self.joint_names:
      joint_id = self.model.joint(name).id
      self.joint_map.append({
          "name": name,
          "joint_id": joint_id,
          "qpos_addr": int(self.model.jnt_qposadr[joint_id]),
          "dof_addr": int(self.model.jnt_dofadr[joint_id]),
          "limited": bool(self.model.jnt_limited[joint_id]),
          "lower": float(self.model.jnt_range[joint_id, 0])
                    if self.model.jnt_limited[joint_id] else None,
          "upper": float(self.model.jnt_range[joint_id, 1])
                    if self.model.jnt_limited[joint_id] else None,
      })

    # 3. ROS 2 订阅者与发布者
    # 接收来自外部控制包的指令
    self.sub_cmd = self.create_subscription(
        ArmPositionCmd,
        '/rebot/joint_group_cmd',
        self.cmd_callback,
        1,  # QoS depth
    )

    # 发布关节当前状态
    self.pub_joint_states = self.create_publisher(JointState, '/joint_states', 10)

    # 创建 ROS 2 定时器发布 /joint_states (50Hz)
    self.timer = self.create_timer(0.2, self.publish_joint_states)

  def cmd_callback(self, msg:ArmPositionCmd):
    """接收来自外部 ROS 2 控制包的命令"""

    if len(msg.position) != self.num_actuators:
        self.get_logger().warn(
            f'Cmd dim mismatch! Expected {self.num_actuators}, got '
            f'{len(msg.position)}'
        )
        return

    with self.cmd_lock:
        self.ctrl_cmd = np.array(msg.position)

  def publish_joint_states(self):
    msg = JointState()
    msg.header.stamp = self.get_clock().now().to_msg()
    msg.name = self.joint_names
    positions = []
    velocities = []
    tol = 1e-3
    for info in self.joint_map:
        q = float(self.data.qpos[info["qpos_addr"]])
        dq = float(self.data.qvel[info["dof_addr"]])
        if info["limited"]:
            lower = info["lower"]
            upper = info["upper"]
            if upper < q <= upper + tol:
                q = upper
            elif lower - tol <= q < lower:
                q = lower
        positions.append(q)
        velocities.append(dq)

    msg.position = positions
    msg.velocity = velocities

    self.pub_joint_states.publish(msg)

  def run_simulation(self):
    """在单独的线程或主循环中运行物理仿真和 Viewer"""
    with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
      self.get_logger().info('MuJoCo Viewer started. Running simulation...')

      while viewer.is_running() and rclpy.ok():
        step_start = time.time()

        # 应用控制指令
        with self.cmd_lock:
          self.data.ctrl[:] = self.ctrl_cmd

        # 物理步进
        mujoco.mj_step(self.model, self.data)

        # 刷新渲染 GUI(mujoco) 
        viewer.sync()

        # 控制仿真步频与真实时间一致
        time_until_next_step = self.model.opt.timestep - (
            time.time() - step_start
        )
        if time_until_next_step > 0:
          time.sleep(time_until_next_step)


def main(args=None):
  rclpy.init(args=args)
  node = RebotMujocoNode()

  # 在后台线程运行 ROS 2 事件 Spin 响应回调
  ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
  ros_thread.start()

  try:
    # 主线程运行物理渲染与仿真
    node.run_simulation()
  except KeyboardInterrupt:
    pass
  finally:
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
  main()