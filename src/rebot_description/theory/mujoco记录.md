1、urdf转成xml命令：
python -c "import mujoco; model = mujoco.MjModel.from_xml_path('reBot_B601_DM_with_gripper.urdf'); mujoco.mj_saveLastXML('reBot_B601_DM_with_gripper.xml', model)"


2、转换成xml，需要加上```xml<actuator>，<sensor> <option>```，例子：

```xml
<option timestep="0.001" integrator="RK4" gravity="0 0 -9.81"/>
```
或者：
```xml
<option timestep="0.002"/>
```

### 添加力传感器
首先添加 site：
```xml
<site  name="ft_sensor_site"  pos="0 0 0.05"  size="0.01"/>```

然后添加 sensor：
```xml
<sensor>
    <!-- 三维力 -->
    <force  name="ee_force_sensor"  site="ft_sensor_site"/>
    <!-- 三维力矩 -->
    <torque name="ee_torque_sensor" site="ft_sensor_site"/>
</sensor>
```



3、一些标签的意义：

damping（粘性阻尼 / 速度阻尼）含义：表示关节运动时的粘性阻尼系数（Damping coefficient），单位通常为 $\text{N}\cdot\text{s}/\text{m}$（滑动关节）或 $\text{N}\cdot\text{m}\cdot\text{s}/\text{rad}$（转动关节）。物理机制：阻尼力（或力矩）与关节的运动速度成正比，方向与运动方向相反：$$\text{Force}_{\text{damping}} = -c \cdot v$$作用：吸收动能（防抖/去震荡）：当连杆高速运动或被 PD 控制器快速拉回目标位置时，阻尼能将动能转化为热能，迅速抑制高频震荡和超调。提高数值稳定性：在求解器中，适当的 damping 相当于给系统的刚度矩阵增加了主对角线元素，防止微分方程解爆炸。

frictionloss（干摩擦 / 库仑摩擦）含义：表示关节内部的静/动干摩擦力上限（Coulomb Friction Loss），单位为 $\text{N}$（滑动关节）或 $\text{N}\cdot\text{m}$（转动关节）。物理机制：提供一个恒定方向相反的阻抗力，其大小不受速度快慢影响（只要速度不为零，阻力就等于该设定值）。作用：消除零速附近的小幅漂移/微抖：在 PID 控制器逼近目标位置且残余误差极小时，控制器输出的微小力矩不足以克服 frictionloss，系统就会完全静止，从而消除了末端的微小抖动。模拟真实电机/减速机的机械摩擦：真实机械臂的齿轮减速箱和滑轨都有固有干摩擦，配置该值能让仿真更逼真。

solreflimit="0.02 1" 是 MuJoCo XML 中接触约束（contact constraint）的求解参数，全称是：solref limit它用于控制 关节限位（joint limit）约束 或其他约束的弹簧-阻尼响应特性。
    1. 先理解 MuJoCo 的约束求解
    MuJoCo 里面很多东西不是直接用刚性约束实现，而是通过**软约束（soft constraint）**求解。
    例如：关节限位你的：
    ```xml
    <joint
        name="joint1"
        range="-2.8 2.8"/>
    ```
    表示：−2.8≤q1≤2.8。当：q1>2.8，MuJoCo 不会直接：位置截断而是产生一个约束力：τlimit 把关节推回来。这个恢复力由：solreflimit 控制。
    2. solref格式 写法：solref="timeconst dampratio" 所以： solreflimit="0.02 1" 表示：
    参数	含义
    0.02	时间常数
    1	阻尼比
    3. 第一个参数：time constant 例如： solref="0.02 1" 第一个：τ=0.02s代表：约束恢复速度。简单理解：小 → 约束很硬，恢复快，大 → 约束柔软，恢复慢。例如 很硬 solreflimit="0.002 1" 效果：超过limit -> 很强恢复力 -> 快速拉回。很软 solreflimit="0.1 1" 效果：超过limit -> 慢慢回来。
    4. 第二个参数：阻尼比 第二个: ζ 类似二阶系统：mx¨+cx˙+kx=0 阻尼比：ζ=2mkc  zeta=1 solreflimit="0.02 1" 表示：临界阻尼。 效果：最快回到限制位置,且不过冲 zeta<1。
    例如： solreflimit="0.02 0.5" 欠阻尼： 越界. 会振荡。 zeta>1 例如： solreflimit="0.02 2" 过阻尼：慢慢恢复
    5. 和 actuator 的区别。很多人容易混淆：actuator 例如：
    ```xml
    <position
    joint="joint6"
    kp="100"
    kv="10"/>
    ``` 控制： τ=Kp​(qd−q)−Kv​q˙ 作用：控制目标位置。 solreflimit 控制：τlimit ​作用：防止：超过机械限位。 二者关系：目标位置-> actuator -> 关节运动 -> 超过limit? -> solreflimit产生恢复力

amature含义：MuJoCo 里的 armature（注意不是 amature）指的是：关节电机转子的等效转动惯量（joint armature inertia），也叫关节附加惯量。它不是机械臂连杆本身的惯量，而是人为添加到关节上的惯量项，用于模拟电机、减速器、传动系统带来的惯性。1. 为什么需要 armature？ 真实机器人：电机 -> 减速器 -> 关节 -> 连杆。当电机转动时，不只是连杆在运动：电机转子有惯量,齿轮有惯量,传动轴有惯量。所以真实动力学：(Mlink+Mmotor)q¨=τ 但是 URDF/MJCF 里面通常只描述：link mass，link inertia 没有电机惯量。因此 MuJoCo 提供：
    ```xml
    <joint armature="..."/>
    ```
人为增加：M(q)+Jmotor   2. 数学上它加在哪里？机械臂动力学：M(q)q¨+C(q,q˙)q˙+G(q)=τ 。加入 armature：(M(q)+A)q¨+C(q,˙)q˙+G(q)=τ 。其中：$$A = \begin{bmatrix}a_1 &0 & \dots \\0 & a_2 & \dots \\\dots\end{bmatrix}$$每个 joint 一个附加惯量。 4.armature 对控制有什么影响？ 非常重要。armature 小例如：armature="0", 系统：轻 -> 响应快
->容易震荡,因为：机器人动力学：M(q)q¨+C(q,q˙)q˙+G(q)=τ  简化：Mq¨=τ 所以：q¨= τ/M 也就是：α=τ/M。 惯量小：α大。armature 大例如：armature="0.1"，系统：重 -> 响应慢 -> 更稳定。因为：α=M+Aτ 5. 和 kp/kd 的关系
你的机械臂：
```xml
<position
joint="joint6"
kp="50"
kv="5"/>
```
本质：τ=Kp(qd−q)−Kvq˙ 系统近似：(M+A)q¨=Kpe−Kvq˙ 二阶系统：(M+A)e¨+Kve˙+Kpe=0  自然频率：ωn=M+AKp  阻尼：ζ=2Kp(M+A)Kv 所以：armature 增大 ，等效：M+A↑导致：ωn↓ 响应慢。同时：ζ↑ 更不容易震。6. 为什么机器人仿真经常加 armature？尤其是：人形机器人，四足机器人，强化学习。因为真实机器人：电机+减速器惯量不能忽略。例如：Isaac Gym / MuJoCo RL里面：armature="0.01"非常常见。

contype conaffinity 含义：这两个参数是 MuJoCo 中用来控制碰撞分组与过滤（Collision Filtering）的核心属性，属于 bitmask（位掩码）机制。简而言之：只要将某个 geom 的这两个参数都设为 0，它就会变成“无碰撞体”（Ghost/Visual Geometry），任何东西都会直接穿过它。 1. 参数的各自含义 contype（Contact Type - 我是谁）表示当前几何体（Geom）所属的碰撞类别位掩码。它定义了“我属于哪些碰撞组”。 conaffinity（Contact Affinity - 我和谁碰）表示当前几何体感兴趣/允许碰撞的目标类别位掩码。它定义了“我想与哪些碰撞组发生碰撞”。 2. 碰撞发生的触发条件在 MuJoCo 中，两个几何体（假设为 A 和 B）要发生物理碰撞，必须同时满足以下条件： CanCollide(A,B)=(A.contype&B.conaffinity)=0AND(B.contype&A.conaffinity)=0 即：A 的类型要符合 B 的意向，并且 B 的类型也要符合 A 的意向。3. 为什么设为 0 就能禁用碰撞？ 当你设置 contype="0" 时，意味着这个物体不属于任何碰撞组（没有任何类型位被置 1）。当你设置 conaffinity="0" 时，意味着这个物体不与任何碰撞组发生交互。根据上面的逻辑与（AND）公式，任何数与 0 做按位与（&）运算结果均为 0。因此，只要设置了 contype="0" conaffinity="0"：
它不会与任何其他几何体发生物理阻挡或产生碰撞力。它仅保留视觉渲染（Visual Rendering）功能（在 GUI 中依然可见，但物理引擎会完全忽略它）。

4、初步导入mujoco环境时，机械臂末端出现异常抖动：
    1、排查是否相邻link是否碰撞，用
```xml   
    <contact>
    <exclude body1="link1" body2="link2"/>
    <exclude body1="link2" body2="link3"/>
    <exclude body1="link3" body2="link4"/>
    <exclude body1="link4" body2="link5"/>
    <exclude body1="link5" body2="link6"/>
    <exclude body1="link6" body2="gripper_left"/>
    <exclude body1="link6" body2="gripper_right"/>
    <exclude body1="gripper_left" body2="gripper_right"/>
    </contact>
```
  避免。2、看joint的kp,kd设置是否正常，经验为小质量，小kp,kd，大质量，大kp,kd。 3、看一些关节是否加入 damping, frictionloss。



## MuJoCo 执行器参数详解：`actuatorfrcrange` 与 `position` 控制器的关系
这两个参数控制的是两个不同层面的东西：
```xml
<joint name="joint2" ... actuatorfrcrange="-27 27" />
<actuator>
    <position joint="joint2" kp="100" kv="20"/>
</actuator>
```
它们共同决定最终关节运动效果。
### 1. $k_p$、$k_v$ 是控制器参数
```xml
<position joint="joint2" kp="100" kv="20"/>
```
表示 MuJoCo 给 `joint2` 配置了一个位置 PD 控制器。
**数学形式近似：**
$$\tau_{\text{cmd}} = K_p (q_d - q) + K_v (\dot{q}_d - \dot{q})$$
**其中：**  
* $q_d$：你通过 `data.ctrl` 给的目标角度  
* $q$：当前关节角  
* $K_p$：位置刚度  
* $K_v$：阻尼   
### 2. `joint4` 实际计算示例
对于以下配置：
```xml
<joint name="joint4" actuatorfrcrange="-7 7"/>
<position joint="joint4" kp="25" kv="8"/>
```
假设：  
* 当前角：$q = 0.3 \text{ rad}$  
* 目标角：$q_d = 0 \text{ rad}$  
* 误差：$e = -0.3 \text{ rad}$  
**PD 输出：**  
$$\tau = 25 \times (-0.3) = -7.5 \text{ N}\cdot\text{m}$$ 
**最终限制：**  
由于 `joint4` 的力矩限制为 $[-7, 7]$，所以实际输出：  
$$\tau = -7 \text{ N}\cdot\text{m}$$  
虽然你设置了 $k_p = 25$，但是此时输出**已经打满**。  
### 3. 一个简单经验公式
对于 `position` 执行器，建议在**最大允许误差**时，控制器计算出的力矩刚好接近**最大力矩**：
$$K_p e_{\text{max}} \approx \tau_{\text{max}}$$
以 `joint4` 为例：
* 假设最大允许误差 $e_{\text{max}} = 0.3 \text{ rad}$
* 最大力矩 $\tau_{\text{max}} = 7 \text{ N}\cdot\text{m}$
计算得到：
$$K_p = \frac{7}{0.3} \approx 23$$
所以设置 `kp=25` 是非常有道理且合理的。
### 7. 两者关系总结
| 参数 | 作用 | 类比 |
| --- | --- | --- |
| **$k_p$** | 想拉回目标的力度 | 弹簧刚度 |
| **$k_v$** | 抑制速度震荡 | 阻尼 |
| **`actuatorfrcrange`** | 最大输出能力 | 电机最大扭矩 |
