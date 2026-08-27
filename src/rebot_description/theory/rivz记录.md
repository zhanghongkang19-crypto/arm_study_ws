# 实现：
## ROS InteractiveMarker 结构关系:
```text
InteractiveMarker
    │
    └── controls[] (多个 InteractiveMarkerControl)
            │
            └── InteractiveMarkerControl
                      │
                      └── markers[] (0个或多个 Marker)
                              │
                              └── Marker
                                  └── sphere_marker

```

> **⚠️ 注意事项与常见误区：**
> * **结构层级：** 一个 `InteractiveMarker` 包含多个 `InteractiveMarkerControl`；一个 `InteractiveMarkerControl` 包含 **0 个或多个** `Marker`。
> * **可选的 Marker：** **并不是每一个 `InteractiveMarkerControl` 都必须包含 `Marker`。**
> * **使用场景区别：**
> * **有 Marker 的 Control：** 用于在 3D 场景中渲染可自定义形态的几何体（如球体、方块、Mesh），用户点击或拖拽该 Marker 触发交互。
> * **无 Marker 的 Control：** 通常仅配置控制模式（例如 `interaction_mode = InteractiveMarkerControl::MOVE_AXIS`），RViz 会根据 `orientation` 自动为你生成默认的操控手柄（如箭头、旋转环），无需手动填充 `markers[]` 数组。

##  RVIZ监听操作是由 `robot_state_publisher` 节点在后台自动完成的。**
当你启动 `robot_state_publisher` 时，它的默认行为就是自动订阅 ROS2 网络中的 `/joint_states` 话题。只要有节点（比如你的 MuJoCo 仿真脚本）向 `/joint_states` 发布关节数据，`robot_state_publisher` 就会实时接收，计算出连杆的 TF 变换并传递给 RViz2 显示。
**配置细节补充**
如果你在 MuJoCo 中发布的关节话题不是默认的 `/joint_states`，而是自定义的话题名称（例如 `/mujoco/joint_states`），你就需要在 Launch 文件中给 `robot_state_publisher` **添加话题重定向（remap）**：
```python
Node(
    package='robot_state_publisher',
    executable='robot_state_publisher',
    name='robot_state_publisher',
    output='screen',
    parameters=[{
        'robot_description': robot_description
    }],
    # 如果 MuJoCo 发布的话题名不是 /joint_states，需要在这里做映射
    remappings=[
        ('/joint_states', '/mujoco/joint_states')
    ]
),
```