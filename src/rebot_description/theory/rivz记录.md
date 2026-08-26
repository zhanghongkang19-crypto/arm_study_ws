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