# MPC
## 1. 系统模型 (System Model)
顾名思义，MPC 高度依赖对系统的建模。通常，我们研究的是一个**线性时不变系统 (LTI)**，其**离散化状态空间方程**如下：
$$x_{k+1} = A x_k + B u_k$$

* $x_k, x_{k+1}$：分别为 $k$ 和 $k+1$ 时刻的**系统状态向量**。   
* $u_k$：为 $k$ 时刻的系统控制输入向量。  
* $A, B$：分别为已知的**系统状态矩阵**和**输入矩阵**。  

## 2. 优化问题与代价函数 (Cost Function)
MPC 本质上是在线求解一个优化问题。我们通过构建预测步数为 $N$ 的**二次型代价函数 (Quadratic Cost Function)**，来实现状态误差与能量消耗的最小化。
采用二次型的原因有两点：其一，平方项能让数值大的偏差（如严重的轨迹偏离）产生极大的代价，迫使系统优先纠正大误差；其二，二次函数是**凸函数**且处处连续可导，比绝对值函数更容易用数值方法高效求解。  
具体的目标函数 $J$ 构建如下：
$$J = \sum_{i=0}^{N-1} \left( x_{k+i\vert{}k}^T Q x_{k+i\vert{}k} + u_{k+i\vert{}k}^T R u_{k+i\vert{}k} \right) + x_{k+N\vert{}k}^T P x_{k+N\vert{}k}$$
其中各项的**物理意义**如下：  
* **状态偏差 ($x^T Q x$)**：惩罚系统状态偏离目标的程度。  
* **控制能量 ($u^T R u$)**：惩罚控制输入的剧烈变化，减少能量消耗。  
* **终端代价 ($x^T P x$)**：保证在预测时域结束时，系统能够稳定在目标点。  
* **$Q, R, P$**：分别为对应的半正定和正定权重矩阵。在实际工程中，可以通过调节这些权重来改变系统的性格（例如：调大 $R$ 会让系统更省油，调大 $Q$ 会让系统响应更快）。  
此外，MPC 最大的亮点在于可以显式地加入**系统约束**（如 $u_{min} \le u_k \le u_{max}$）。

## 3. 矩阵化预测方程 (Matrix Formulation)
为了求解控制输入 $u$，我们需要将未来 $N$ 步的预测状态全部用**当前初始状态 $x_k$** 和**未来控制输入 $U_k$** 来表示。利用 $x_{k+1} = A x_k + B u_k$ 不断**递归**代入，最终可以化简为紧凑的**矩阵乘法**形式：  
设预测状态序列 $X_k$ 和控制输入序列 $U_k$ 为：
$$X_k = \begin{bmatrix} x_{k+1\vert{}k} \\ x_{k+2\vert{}k} \\ \vdots \\ x_{k+N\vert{}k} \end{bmatrix}, \quad U_k = \begin{bmatrix} u_{k\vert{}k} \\ u_{k+1\vert{}k} \\ \vdots \\ u_{k+N-1\vert{}k} \end{bmatrix}$$
则多步预测方程可表示为：  
$$X_k = M x_k + C U_k$$
其中，**状态转移矩阵 $M$** 和**控制矩阵 $C$** 的构造如下：
$$M = \begin{bmatrix} A \\ A^2 \\ \vdots \\ A^N \end{bmatrix}, \quad C = \begin{bmatrix} B & 0 & \dots & 0 \\ AB & B & \dots & 0 \\ \vdots & \vdots & \ddots & \vdots \\ A^{N-1}B & A^{N-2}B & \dots & B \end{bmatrix}$$
后续需要把这些式子代入 $J$ 的运算，然后凑出 **QP 问题**的标准形式。

## 4.为什么只有终端输出，没有终端状态？
### 1. 先看离散系统
MPC 通常写成：
$$x_{k+1} = f(x_k, u_k)$$
如果预测时域长度是 $N$，那么：
$$x_0 \xrightarrow{u_0} x_1 \xrightarrow{u_1} x_2 \xrightarrow{u_2} \dots \xrightarrow{u_{N-1}} x_N$$
**注意**：  
* **状态有 $N+1$ 个**：$x_0, x_1, \dots, x_N$  
* **控制输入只有 $N$ 个**：$u_0, u_1, \dots, u_{N-1}$  
因为最后一个状态 $x_N$ 已经由 $u_{N-1}$ 推出来了。  

### 2. 为什么只有终端状态，没有终端输入？
因为：  
$$x_N = f(x_{N-1}, u_{N-1})$$
到达终端状态以后，预测时域就结束了：  
```text
x0      x1      x2             xN
● ----> ● ----> ● ----> ... ---> ●
  u0      u1              u(N-1)
```
如果再定义 $u_N$，那么它的作用是：
$$x_{N+1} = f(x_N, u_N)$$
这意味着你的预测时域实际上已经变成 **$N+1$** 步了。  
因此，标准有限时域最优控制问题一般写成：
$$\min_{\{u_k\}} \sum_{k=0}^{N-1} l(x_k, u_k) + \Phi(x_N)$$
其中：  
* $l(x_k, u_k)$：阶段代价（Stage Cost）  
* $\Phi(x_N)$：终端代价（Terminal Cost）  
所以：  
* **阶段代价**：$(x_0, u_0), (x_1, u_1), \dots, (x_{N-1}, u_{N-1})$  
* **终端代价**：$x_N$  
终端节点通常没有控制输入。  

## 目标函数推导至二次型
这个推导过程是将 **MPC 矩阵化预测方程** 代入 **二次型代价函数** 中，并通过矩阵运算展开、凑出标准二次规划（QP）形式的全过程。
以下是逐步推导过程：

### 第一步：带入状态预测方程
已知 MPC 预测范围内的状态向量可以写成当前状态 $x(k)$ 和控制序列 $U(k)$ 的线性组合：
$$X'(k) = M x(k) + C U(k)$$
其中代价函数形式：原始的目标函数是 $(x - x_{\text{ref}})^T \bar{Q} (x - x_{\text{ref}})$，将偏差直接代入后，目标函数就简化为了上个公式中的 $X'(k)^T \bar{Q} X'(k)$。
对 $X'(k)$ 进行转置（利用矩阵转置性质 $(A+B)^T = A^T + B^T$ 及 $(AB)^T = B^T A^T$）：
$$X'(k)^T = \left( M x(k) + C U(k) \right)^T = x(k)^T M^T + U(k)^T C^T$$
将 $X'(k)$ 和 $X'(k)^T$ 直接代入原始代价函数 $J = X'(k)^T \bar{Q} X'(k) + U(k)^T \bar{R} U(k)$ 中：
$$J = \left( x(k)^T M^T + U(k)^T C^T \right) \bar{Q} \left( M x(k) + C U(k) \right) + U(k)^T \bar{R} U(k)$$

### 第二步：展开括号与交叉项合并
展开第一项乘积：
$$J = x(k)^T M^T \bar{Q} M x(k) + \underbrace{x(k)^T M^T \bar{Q} C U(k)}_{\text{交叉项 1}} + \underbrace{U(k)^T C^T \bar{Q} M x(k)}_{\text{交叉项 2}} + U(k)^T C^T \bar{Q} C U(k) + U(k)^T \bar{R} U(k)$$
**关键细节（交叉项化简）**：  
因为代价函数 $J$ 的每一项最终都是一个**标量（1×1 矩阵）**，而标量的转置等于其自身（即 $a = a^T$）。  
注意到交叉项 1 是交叉项 2 的转置：
$$\left( x(k)^T M^T \bar{Q} C U(k) \right)^T = U(k)^T C^T \bar{Q}^T (M x(k))^T = U(k)^T C^T \bar{Q} M x(k)$$
*（其中权重矩阵 $\bar{Q}$ 是对称矩阵，即 $\bar{Q}^T = \bar{Q}$）*  
所以两个交叉项数值相等，可以合二为一：  
$$\text{交叉项 1} + \text{交叉项 2} = 2 x(k)^T C^T \bar{Q} M U(k) \quad \text{或} \quad 2 x(k)^T M^T \bar{Q} C U(k)$$
代回原式并合并 $U(k)$ 的二次项：  
$$J = x(k)^T M^T \bar{Q} M x(k) + U(k)^T \left( C^T \bar{Q} C + \bar{R} \right) U(k) + 2 x(k)^T C^T \bar{Q} M U(k)$$

### 第三步：定义新矩阵系数
为了简化表达，令：
* **$G = M^T \bar{Q} M$**  
* **$H = C^T \bar{Q} C + \bar{R}$**  
* **$E = C^T \bar{Q} M$**  

### 第四步：得出最终表达形式
代入上述定义的系数矩阵，得到最终形式：
$$J = \underbrace{x(k)^T G x(k)}_{\text{常数项}} + \underbrace{2 x(k)^T E U(k)}_{\text{线性部分}} + \underbrace{U(k)^T H U(k)}_{\text{二次项部分}}$$

### 各项在求解时的物理与数学意义
1. **常数项 ($x(k)^T G x(k)$)**：在当前时刻 $k$，初始状态 $x(k)$ 是已知量，因此这一项对于决策变量 $U(k)$ 来说是一个常数，在 QP 求解时可以直接忽略（不影响求极小值点）。  
2. **线性部分 ($2 x(k)^T E U(k)$)**：与控制量 $U(k)$ 呈线性关系。  
3. **二次项部分 ($U(k)^T H U(k)$)**：控制量的二次型，矩阵 $H$（Hessian 矩阵）决定了二次规划问题的凸性与求解稳定性。  
最终该式可直接丢入二次规划（QP）求解器 $\min_U \left( \frac{1}{2} U^T H U + g^T U \right)$ 中求得最佳控制序列 $U(k)$。  

## $x(k)^T M^T \bar{Q} M x(k)$  为什么是常数项？
在当前时刻 $k$，**$x(k)^T M^T \bar{Q} M x(k)$ 被归为常数项**，原因在于 MPC 的求解机制：

### 1. 核心原因：决策变量与已知量不同
* **决策变量（未知量）**：MPC 求解优化问题的目标是找到**未来的最佳控制输入序列 $U(k)$**。在求解这一刻的优化问题时，$U(k)$ 是唯一的未知数（变量）。  
* **已知量（常量）**：在时刻 $k$，**系统的当前状态 $x(k)$ 是通过传感器测量或状态观测器直接获取的**。对于这个时刻的求解过程来说，$x(k)$ 是一个已知确定的数值。  

### 2. 数学结构与维度
* **$x(k)$** 是一个确定的向量；  
* **$M$ 和 $\bar{Q}$** 是已知且固定维度的矩阵；  
* 经过矩阵乘法后：  

$$\underbrace{x(k)^T}_{1 \times n} \cdot \underbrace{M^T}_{n \times Nn} \cdot \underbrace{\bar{Q}}_{Nn \times Nn} \cdot \underbrace{M}_{Nn \times n} \cdot \underbrace{x(k)}_{n \times 1}$$

得到的运算结果是一个**具体的数值（标量，大小为 $1 \times 1$）**。
由于这一项中**完全不包含任何决策变量 $U(k)$**，它在数学上对 $U(k)$ 的求导或求极值结果没有任何影响。

### 3. 对二次规划（QP）求解的影响
标准二次规划（QP）问题的标准形式为：  
$$\min_U \left( \frac{1}{2} U^T H U + g^T U \right)$$
如果我们在目标函数后加上一个不含 $U$ 的常数项 $C_{\text{const}}$：
$$\min_U \left( \frac{1}{2} U^T H U + g^T U + C_{\text{const}} \right)$$
使得目标函数取得最小值时的最优解 $U^*$ **完全不会改变**（仅仅是最终得到的最小代价 $J_{\min}$ 的绝对数值变了）。因此，在将代价函数输入给二次规划求解器时，这一项通常直接被当作常数忽略或剔除。


# NMPC
## Euler离散化
### 1. 为什么需要离散化？
机械臂动力学通常写成连续时间形式：
$$\dot{x}(t) = f(x(t), u(t))$$
**符号含义**：
* $x(t)$：当前状态  
* $u(t)$：控制输入  
* $\dot{x}(t)$：状态随时间的变化速度  
* $f(x, u)$：动力学模型  
例如在机械臂控制中：  
$$x = \begin{bmatrix} q \\ \dot{q} \end{bmatrix}$$
其中 $q$ 是关节角，$\dot{q}$ 是关节速度。  
但是计算机与 MPC（模型预测控制）不能直接在连续的时间区间 $t \in [0, \infty)$ 上进行数值计算与优化，必须将其切分为离散的数字时间节点： 
$$k=0 \to k=1 \to k=2 \to k=3 \to \dots$$
将连续时间标记 $t$ 转变为离散步数标记 $k$ 的过程，就是**离散化**。

### 2. Forward Euler（前向欧拉）离散化的核心思想
从最基本的导数定义开始：
$$\dot{x}(t) = \frac{dx}{dt}$$
采用很小的时间间隔（采样周期） $\Delta t$ 进行一阶有限差分近似：
$$\dot{x}(t) \approx \frac{x(t + \Delta t) - x(t)}{\Delta t}$$
两边同乘以 $\Delta t$ 展开移项：
$$x(t + \Delta t) - x(t) \approx \Delta t \cdot \dot{x}(t)$$
$$x(t + \Delta t) \approx x(t) + \Delta t \cdot \dot{x}(t)$$
将连续动力学方程 $\dot{x}(t) = f(x(t), u(t))$ 代入上式，并写成离散步数 $k$ 的形式：
$$x_{k+1} = x_k + \Delta t \cdot f(x_k, u_k)$$
这就是最基本、工程中最常用的 **Forward Euler（前向欧拉）离散化公式**。

## NMPC的特点
NMPC 不要求动力学必须是线性的。  
例如机械臂动力学方程：
$$M(q)\ddot{q} + C(q,\dot{q})\dot{q} + g(q) = \tau$$
这里明显存在复杂的非线性耦合： 
* $M(q)$：关节角度决定的惯性矩阵  
* $C(q,\dot{q})$：科里奥利力与离心力项  
* $g(q)$：重力项
因此，系统无法写成简单的线性离散形式 $x_{k+1} = A x_k + B u_k$，而是写成连续非线性状态方程：  
$$\dot{x} = f(x, u)$$
定义状态向量 $x = [q^T, \dot{q}^T]^T$，控制输入 $u = \tau$。经过离散化后，NMPC 的优化目标形式为：  
$$\min_{u} J(x, u)$$
$$\text{subject to: } x_{k+1} = f(x_k, u_k)$$


## 求解方法
### 方法一：基于 LTV-MPC 的连续线性化与 QP 构建（工程最常用）
这种方法在每个控制周期 $k$，沿着**当前参考轨迹**或**上一时刻预测轨迹**对非线性系统进行一阶泰勒展开，将其转化为线性 MPC 求解。
**1. 连续非线性系统及其离散化**
已知离散非线性状态方程：
$$x_{k+1} = f(x_k, u_k)$$
在参考点 $(x_k^{\text{ref}}, u_k^{\text{ref}})$ 处进行一阶泰勒展开：
$$f(x_k, u_k) \approx f(x_k^{\text{ref}}, u_k^{\text{ref}}) + \left. \frac{\partial f}{\partial x} \right\vert{}_{(x_k^{\text{ref}}, u_k^{\text{ref}})} (x_k - x_k^{\text{ref}}) + \left. \frac{\partial f}{\partial u} \right\vert{}_{(x_k^{\text{ref}}, u_k^{\text{ref}})} (u_k - u_k^{\text{ref}})$$
定义 Jacobian 矩阵与状态偏差：
* $A_k = \left. \frac{\partial f}{\partial x} \right\vert{}_{(x_k^{\text{ref}}, u_k^{\text{ref}})}$
* $B_k = \left. \frac{\partial f}{\partial u} \right\vert{}_{(x_k^{\text{ref}}, u_k^{\text{ref}})}$
* $\delta x_k = x_k - x_k^{\text{ref}}$
* $\delta u_k = u_k - u_k^{\text{ref}}$
得到**线性时变偏差系统**：
$$\delta x_{k+1} = A_k \delta x_k + B_k \delta u_k$$
**2. 预测方程的矩阵化**
因为 $A_i, B_i$ 随着预测步数 $i$ 变化，多步预测展开为：
$$\begin{aligned} \delta x_{k+1\vert{}k} &= A_0 \delta x_k + B_0 \delta u_{k\vert{}k} \\ \delta x_{k+2\vert{}k} &= A_1 \delta x_{k+1\vert{}k} + B_1 \delta u_{k+1\vert{}k} = A_1 A_0 \delta x_k + A_1 B_0 \delta u_{k\vert{}k} + B_1 \delta u_{k+1\vert{}k} \\ &\ \ \vdots \end{aligned}$$
写成紧凑矩阵形式：
$$\Delta X = \mathcal{A}_k \delta x_k + \mathcal{B}_k \Delta U$$
其中，**时变系统矩阵**的构造为：
$$\Delta X = \begin{bmatrix} \delta x_{k+1\vert{}k} \\ \delta x_{k+2\vert{}k} \\ \vdots \\ \delta x_{k+N\vert{}k} \end{bmatrix}, \quad \Delta U = \begin{bmatrix} \delta u_{k\vert{}k} \\ \delta u_{k+1\vert{}k} \\ \vdots \\ \delta u_{k+N-1\vert{}k} \end{bmatrix}$$
$$\mathcal{A}_k = \begin{bmatrix} A_0 \\ A_1 A_0 \\ \vdots \\ A_{N-1} \dots A_1 A_0 \end{bmatrix}, \quad \mathcal{B}_k = \begin{bmatrix} B_0 & 0 & \dots & 0 \\ A_1 B_0 & B_1 & \dots & 0 \\ \vdots & \vdots & \ddots & \vdots \\ A_{N-1}\dots A_1 B_0 & A_{N-1}\dots A_2 B_1 & \dots & B_{N-1} \end{bmatrix}$$
**3. 二次规划（QP）标准型推导**
将 $\Delta X = \mathcal{A}_k \delta x_k + \mathcal{B}_k \Delta U$ 代入以偏差量表示的代价函数：
$$J = \Delta X^T Q \Delta X + \Delta U^T R \Delta U$$
完全展开：
$$J = (\mathcal{A}_k \delta x_k + \mathcal{B}_k \Delta U)^T Q (\mathcal{A}_k \delta x_k + \mathcal{B}_k \Delta U) + \Delta U^T R \Delta U$$
$$J = \delta x_k^T \mathcal{A}_k^T Q \mathcal{A}_k \delta x_k + \Delta U^T (\mathcal{B}_k^T Q \mathcal{B}_k + R) \Delta U + 2 \delta x_k^T \mathcal{A}_k^T Q \mathcal{B}_k \Delta U$$
忽略与决策变量 $\Delta U$ 无关的常数项 $\delta x_k^T \mathcal{A}_k^T Q \mathcal{A}_k \delta x_k$，定义：
* **$H = 2 (\mathcal{B}_k^T Q \mathcal{B}_k + R)$** （Hessian 矩阵）  
* **$g = 2 \mathcal{B}_k^T Q \mathcal{A}_k \delta x_k$** （梯度向量）  
最终化为标准 QP 问题：
$$\min_{\Delta U} \left( \frac{1}{2} \Delta U^T H \Delta U + g^T \Delta U \right)$$
求解出最优控制偏差序列 $\Delta U^*$ 后，第一步控制量即为：
$$u_k^* = u_k^{\text{ref}} + \delta u_{k\vert{}k}^*$$

### 方法二：完全非线性约束的直接多次打靶法（Direct Multiple Shooting）
如果不进行线性化近似，而是保留完全非线性动力学，优化问题会直接转化为一个**高维非线性规划（NLP）问题**，通常使用 SQP（序列二次规划）或 IPM（内点法）求解。
**1. 决策变量组装**
将**所有未来状态**与**控制序列**同时作为决策变量进行优化：
$$w = \begin{bmatrix} x_0 \\ u_0 \\ x_1 \\ u_1 \\ \vdots \\ x_{N-1} \\ u_{N-1} \\ x_N \end{bmatrix}$$
**2. 包含系统动力学的通用 NLP 表达**
$$\min_{w} \sum_{i=0}^{N-1} l(x_i, u_i) + \Phi(x_N)$$
$$\text{subject to:} \begin{cases} x_0 - x(k) = 0 & \text{(初始状态约束)} \\ x_{i+1} - f(x_i, u_i) = 0, \quad i = 0, \dots, N-1 & \text{(非线性动力学打靶约束)} \\ u_{\min} \le u_i \le u_{\max} & \text{(控制输入约束)} \\ h(x_i, u_i) \le 0 & \text{(状态/状态-控制组合不等式约束)} \end{cases}$$
**两种方法的区别**：
* **LTV-NMPC**：每一帧将非线性动力学展开成 $A_k, B_k$，化成 QP 求解，速度极快（微秒/毫秒级），非常适合自动驾驶、四足机器人等高频实时控制。  
* **Full-NLP (SQP)**：直接迭代求解原非线性模型，跟踪精度和大偏差下的鲁棒性更高，但算力开销相对较大。  
你目前的项目场景更倾向于使用 **LTV-MPC（QP 求解）** 还是 **纯 NLP（如 CasADi / Ipopt / acados 求解）** 呢？我可以为你针对性提供对应的 C++ 或 Python 示例代码！

# DDP
## 1. **最优控制**问题
给定一个具有状态 $x \in \mathbb{R}^N$ $u \in \mathbb{R}^M$ 和控制的连续时间系统，其**动态特性**由以下描述：
$$\frac{dx}{dt} = f(\mathbf{x}(t), \mathbf{u}(t), t)$$
我们想要最小化**成本函数**：
$$J(\mathbf{x}(t_0), \pi) = \underbrace{h(\mathbf{x}(t_f), t_f)}_{\text{terminal cost}} + \int_{t_0}^{t_f} \underbrace{g(\mathbf{x}(t), \mathbf{u}(t), t)dt}_{\text{cost-to-go}}$$
这可以理解为从状态 $\mathbf{x}(t_0)$ 开始遵循策略 $\pi$ 的累积成本。我们现在的目标是找到**最优策略** $\pi^*$，以找到最小成本：
$$\pi^* = \arg\min_{\pi} J(\mathbf{x}(t_0), \pi)$$
假设时间不变动态，此问题也可**离散化**如下。给定固定时间范围 **N**，定义我们的**时间步长** $\Delta t = \frac{t_f - t_0}{N}$
$$\begin{aligned} \frac{\mathbf{x}(t_k + \Delta t) - \mathbf{x}(t_k)}{\Delta t} &\approx f(\mathbf{x}(t_k), \mathbf{u}(t_k)) \\ \mathbf{x}(t_k + \Delta t_k) &= \mathbf{x}(t_k) + \Delta t f(\mathbf{x}(t_k), \mathbf{u}(t_k)) \\ \mathbf{x}(t_{k+1}) &= \mathbf{x}(t_k) + \Delta t f(\mathbf{x}(t_k), \mathbf{u}(t_k)) \\ \mathbf{x}_{k+1} &= \mathbf{x}_k + \Delta t f(\mathbf{x}_k, \mathbf{u}_k) \end{aligned}$$
在最后一行中，为了便于阅读，我们做了符号简化 $\mathbf{x}_k := \mathbf{x}(t_k)$。这将在后续操作中变得紧凑。
以类似的方式，我们也可以将成本函数离散化为：
$$J(\mathbf{x}_0, \pi) = h(\mathbf{x}_N, t_N) + \sum_{k=0}^{N-1} g(\mathbf{x}_k, \mathbf{u}_k, t_k)$$
在接下来的内容中，我们将假设我们的成本函数都是时不变的：
$$J(\mathbf{x}_0, \pi) = h(\mathbf{x}_N) + \sum_{k=0}^{N-1} g(\mathbf{x}_k, \mathbf{u}_k)$$

## 2. **动态规划（DP）**
让我们考虑一个**离散图**，我们希望以最小的成本从起始节点遍历到目标节点，如下图所示。将每个节点视为一个状态 $\mathbf{x}$，其值 $\mathbf{V}(\mathbf{x})$ 表示从该状态到达目标的剩余成本。从节点出发的每条边可以被视为一个**离散动作**，具有相关的到达成本 $g(\mathbf{x}, \mathbf{u})$，自然地，目标状态将具有终端成本 $h(\mathbf{x}_{goal}) = 0$
```text
       ●--------(2)------->●--------(4)------->●
      / \                 / \                 / \
     /   \--(3)-------\  /   \-(3)---\       /   \
    (1)                \/             \     (1)   (2)
   /                    ●---------(1)--\-->●-----\-->●
  /                    /                \         \ /
● start             (2)                  (3)       ● goal
  \                  /                    \       /
   \                /                      \     /
    (1)------------●-------------(2)---------->●/

```
通过图找到最优路径的一种方法是利用**贝尔曼最优性原理**，该原理指出，一个状态的最优值是最小化的“到达成本”与后续状态值 $\mathbf{x}'$ 的组合
$$V^*(\mathbf{x}) = \min_{\mathbf{u}} [g(\mathbf{x}, \mathbf{u}) + V^*(\mathbf{x}')]$$
如果我们有一个神奇的预言机来给我们提供 $\mathbf{x}'$ 的**最优值**，那么这个公式允许我们回溯时间并计算每个 $\mathbf{x}$ 的最优值。幸运的是，对于我们的玩具问题（以及更广泛的问题），我们已经有了最终状态的最优值，因为无需做出任何决策：
$$V^*(\mathbf{x}_{goal}) = \min_{\mathbf{u}} [h(\mathbf{x}_{goal})] = \min_{\mathbf{u}} [0] = 0$$
现在，我们继续回溯时间，利用**贝尔曼方程**，可以估计每个状态的价值
一旦有了每个节点/状态的值，我们就可以通过简单地查看下一个可到达的状态并**选择成本**最低的状态来轻松找到最低成本的路径。事实上，我们可以从任何想要的节点开始，而不仅仅是起始节点。现在让我们更加严谨，值 $\mathbf{V}(\mathbf{x})$ 的定义是什么？它是如果我们从 $\mathbf{x}$ 开始并尝试达到目标时将产生的成本。之前我们看到了最优值 $\mathbf{V}^*(\mathbf{x})$，它为我们提供了最佳情况下的成本。然而，更一般的表述是在某个策略 $\pi$ 下的值，这个策略可能是最优的，也可能不是，这并不重要：
$$\begin{aligned} V^\pi(\mathbf{x}_n, t_n) &= \min_{\mathbf{u}} [J(\mathbf{x}_n, \pi)] = \min_{\mathbf{u}} \left[ \sum_{k=n}^{N-1} g(\mathbf{x}_k, \mathbf{u}_k) + h(\mathbf{x}_N) \right] \\ &= \min_{\mathbf{u}} \left[ g(\mathbf{x}_n, \mathbf{u}_n) + \underbrace{\sum_{k=n+1}^{N-1} g(\mathbf{x}_k, \mathbf{u}_k) + h(\mathbf{x}_N)}_{V^\pi(\mathbf{x}_{n+1}, n+1)} \right] \\ &= \min_{\mathbf{u}} \left[ g(\mathbf{x}_n, \mathbf{u}_n) + V^\pi(\mathbf{x}_{n+1}, n+1) \right] \end{aligned}$$
其中 $\mathbf{x}_{n+1} = f(\mathbf{x}_n, \mathbf{u}_n)$。这种**递归关系**被称为离散贝尔曼方程。

**DDP（动态微分编程，Dynamic Differential Programming）** 的核心推导逻辑，本质上就是把前面你提过的两个理论结合在一起：**“离散贝尔曼方程（动态规划）”** $+$ **“二阶泰勒展开（局部近似）”**。

在连续非线性系统中，传统的动态规划（DP）面临“维度灾难”无法直接求解。DDP 的思想是：**不再求解全状态空间的最优值，而是沿着一条给定的“名义轨迹”，对贝尔曼方程进行局部二阶逼近，从而得到局部最优解，并通过迭代逐步逼近全局最优。**

## DDP 的完整推导四步曲
#### 1. 起点：离散贝尔曼方程（Bellman Equation）
根据上一张图的推导，第 $n$ 步的价值函数 $V(\mathbf{x}_n)$ 满足递归关系：
$$V(\mathbf{x}_n) = \min_{\mathbf{u}_n} \Big[ \underbrace{g(\mathbf{x}_n, \mathbf{u}_n)}_{\text{当步代价}} + \underbrace{V(\mathbf{x}_{n+1})}_{\text{未来累积代价}} \Big]$$
其中状态转移方程为 $\mathbf{x}_{n+1} = \tilde{f}(\mathbf{x}_n, \mathbf{u}_n)$。

#### 2. 定义 Q 函数（动作-价值函数）
为了求极值方便，把括号里的表达式定义为 $Q$ 函数：
$$Q(\mathbf{x}_n, \mathbf{u}_n) = g(\mathbf{x}_n, \mathbf{u}_n) + V(\tilde{f}(\mathbf{x}_n, \mathbf{u}_n))$$
此时贝尔曼方程简化为：$V(\mathbf{x}_n) = \min_{\mathbf{u}_n} Q(\mathbf{x}_n, \mathbf{u}_n)$。

#### 3. 二阶泰勒展开（局部近似）
非线性的 $Q$ 极其复杂，直接求 $\min$ 无法得到解析解。因此，DDP 在名义轨迹点 $(\bar{\mathbf{x}}_n, \bar{\mathbf{u}}_n)$ 处，对 $Q$ 函数做**二阶泰勒展开**（也就是你前面提问的那两张公式图）：
把偏差量设为 $\delta \mathbf{x}_n = \mathbf{x}_n - \bar{\mathbf{x}}_n$，$\delta \mathbf{u}_n = \mathbf{u}_n - \bar{\mathbf{u}}_n$，展开得到：
$$Q(\delta \mathbf{x}_n, \delta \mathbf{u}_n) \approx Q + Q_{\mathbf{x}} \delta \mathbf{x}_n + Q_{\mathbf{u}} \delta \mathbf{u}_n + \frac{1}{2}\delta \mathbf{x}_n^T Q_{\mathbf{xx}} \delta \mathbf{x}_n + \delta \mathbf{x}_n^T Q_{\mathbf{xu}} \delta \mathbf{u}_n + \frac{1}{2}\delta \mathbf{u}_n^T Q_{\mathbf{uu}} \delta \mathbf{u}_n$$
*(通过链式法则，利用 $\nabla_{\mathbf{x}}g, \nabla_{\mathbf{x}}\tilde{f}, \nabla^2 V$ 等导数拼接，即可得到 $Q_{\mathbf{x}}, Q_{\mathbf{u}}, Q_{\mathbf{xx}}, Q_{\mathbf{xu}}, Q_{\mathbf{uu}}$)*。

#### 4. 对决策变量 $\delta \mathbf{u}_n$ 极值化（导出控制律）
因为状态偏置 $\delta \mathbf{x}_n$ 是被动的，只有控制偏置 $\delta \mathbf{u}_n$ 是算法可以主动决策的自变量。
对 $\delta \mathbf{u}_n$ 求偏导并令导数为 0：
$$\frac{\partial Q}{\partial \delta \mathbf{u}_n} = Q_{\mathbf{u}} + Q_{\mathbf{ux}}\delta \mathbf{x}_n + Q_{\mathbf{uu}}\delta \mathbf{u}_n = \mathbf{0}$$
直接解出**最优控制增量** $\delta \mathbf{u}_n^*$：
$$\delta \mathbf{u}_n^* = \underbrace{- Q_{\mathbf{uu}}^{-1} Q_{\mathbf{u}}}_{\mathbf{k}_n \text{ (前馈项)}} \; \underbrace{- Q_{\mathbf{uu}}^{-1} Q_{\mathbf{ux}}}_{\mathbf{K}_n \text{ (反馈增益)}} \cdot \delta \mathbf{x}_n$$
将解出的 $\delta \mathbf{u}_n^*$ 代回原 $Q$ 函数二阶展开式中，就能得到当前时刻价值函数 $V(\mathbf{x}_n)$ 的导数（$V_{\mathbf{x}}$ 和 $V_{\mathbf{xx}}$），从而继续向前一步（第 $n-1$ 步）反向递推。  

### DDP 与 iLQR 的关键区别
推导到这一步时，有两个分支：
* **标准 DDP**：在计算 $Q$ 函数的二阶导数（$Q_{\mathbf{xx}}, Q_{\mathbf{xu}}, Q_{\mathbf{uu}}$）时，**保留了系统运动学/动力学方程 $\tilde{f}$ 的二阶导数**（$\nabla^2 \tilde{f}$）。  
* **iLQR（迭代 LQR）**：忽略了系统动力学 $\tilde{f}$ 的二阶导数（假定系统局部线性化），只保留代价函数 $g$ 的二阶导数。  
**一句话总结 DDP 的推导脉络**：  
用**贝尔曼方程**将复杂的多步最优控制化简为单步问题 $\to$ 引入 **$Q$ 函数** $\to$ 进行**二阶泰勒展开**转化为二次型求极值问题 $\to$ 对控制量求导，推导出由 **前馈 $\mathbf{k}_n$** 和 **反馈 $\mathbf{K}_n$** 构成的控制律。  