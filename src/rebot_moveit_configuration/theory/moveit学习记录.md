## MoveIt 2 & ROS 2 架构与工作流全景概括
### 1. 核心定位与关系
* **ROS 2（底层通信基础设施）：** 提供 Node、Topic、Service 等基本通信机制，负责数据收发与节点间调度。
* **MoveIt 2（上层运动规划框架）：** 建立在 ROS 2 之上的集成式机器人运动规划框架，整合运动学、碰撞检测、路径规划与轨迹控制。

### 2. 整体框架架构
```text
                     MoveIt 2 (组件生态)
                              │
     ┌────────────────────────┼────────────────────────┐
     ▼                        ▼                        ▼
 运动学 (IK)              路径规划 (Planner)        碰撞检测 (Collision)
  • KDL                    • OMPL (RRT / PRM)       • FCL
  • TRAC-IK                • CHOMP / STOMP
                           • Pilz (工业直线/圆弧)
```

### 3. 核心概念拆解
| 核心组件 | 职责与功能 |
| --- | --- |
| **Robot Model** | 基于 URDF/SRDF 解析的机器人物理模型（关节、连杆、运动限制）。 |
| **Planning Group** | 在 SRDF 中定义的规划组（例如 `arm`），指定需要协同规划的关节链。 |
| **Planning Scene** | MoveIt 的 3D 世界模型，维护机器人自身状态、环境中的障碍物及碰撞关系。 |
| **`move_group`** | MoveIt 的**后台服务节点（大脑）**，集成 IK、规划器、碰撞检测，对外暴露 ROS 2 服务/动作接口。 |
| **`MoveGroupInterface`** | 用户 C++ 程序访问 `move_group` 的**客户端 API 封装**。 |


### 4. 完整控制数据流
```text
【用户节点】               【MoveIt 后台节点】               【硬件/仿真】
 
 Publish /target_pose 
        │
        ▼
 MoveGroupInterface  ──(ROS 2)──►  move_group 节点
                                        │
                                        ├── 1. 目标位姿 ──► IK 解算
                                        ├── 2. 环境信息 ──► 碰撞检测 (FCL)
                                        ├── 3. 路径生成 ──► 规划算法 (OMPL)
                                        └── 4. 轨迹生成 ──► 速度/加速度限制
                                        │
                                        ▼
                              JointTrajectory (轨迹)
                                        │
                                        ▼
                                  Controller (控制器)
                                        │
                                        ▼
                                   Real / Gazebo Robot

```