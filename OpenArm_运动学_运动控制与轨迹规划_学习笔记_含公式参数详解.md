# OpenArm Motion SDK：机器人运动学、运动控制与轨迹规划学习笔记

> **定位**：OpenArm Motion SDK 的理论学习与工程实现配套文档。与 `OpenArm_Motion_SDK_开发设计文档.md` 配合使用。  
> **适用环境**：Ubuntu 22.04、C++20、Pinocchio、Eigen、Ruckig、TOPPRA；核心代码不依赖 ROS。  
> **重点**：对数学符号、集合记号、算法参数、数据维度、单位、坐标系和控制模式逐一解释。  
> **硬件说明**：以下七关节数值、速度和周期均是**教学示例**，不是已核验的 OpenArm / DM-MC02 真实限值。未完成硬件验证前只能离线计算或使用 Mock Driver。

---

## 目录

1. [全局概念与 SDK 架构](#1-全局概念与-sdk-架构)
2. [数学符号和维度速查表](#2-数学符号和维度速查表)
3. [旋转与位姿：SO(3)、SE(3) 详解](#3-旋转与位姿so3se3-详解)
4. [机械臂关节空间、坐标系与 URDF](#4-机械臂关节空间坐标系与-urdf)
5. [正运动学 FK 与 Pinocchio](#5-正运动学-fk-与-pinocchio)
6. [Jacobian：速度运动学](#6-jacobian速度运动学)
7. [逆运动学 IK：DLS 与七自由度冗余](#7-逆运动学-ikdls-与七自由度冗余)
8. [路径、轨迹与 MoveJ](#8-路径轨迹与-movej)
9. [MoveL：笛卡尔路径与连续 IK](#9-movel笛卡尔路径与连续-ik)
10. [TOPPRA：路径时间参数化](#10-toppra路径时间参数化)
11. [轨迹执行与电机控制模式](#11-轨迹执行与电机控制模式)
12. [动力学、零重力与阻抗控制](#12-动力学零重力与阻抗控制)
13. [参数配置规范与常见误区](#13-参数配置规范与常见误区)
14. [分阶段实践与验收](#14-分阶段实践与验收)
15. [学习路线与参考资料](#15-学习路线与参考资料)

---

## 1. 全局概念与 SDK 架构

### 1.1 一条 `move_linear()` 指令背后的计算链

```text
用户：arm.move_linear(target_pose, speed=0.05)
                  │
                  ▼
           读取当前机器人状态 q, dq
                  │
                  ▼
         Robot Model：FK / IK / Jacobian
                  │
                  ▼
         笛卡尔路径：p(s), R(s)
                  │
                  ▼
       连续 IK：获得关节路径 q(s)
                  │
                  ▼
      路径检查：限位、碰撞、奇异性
                  │
                  ▼
       TOPPRA：计算时间规律 s(t)
                  │
                  ▼
       轨迹 q(t), dq(t), ddq(t)
                  │
                  ▼
       Trajectory Executor（执行器）
                  │
                  ▼
     Hardware Interface → DM-MC02 / CAN
                  │
                  ▼
              OpenArm 实体
```

**关键区分**：

| 名称 | 回答的问题 | 输入示例 | 输出示例 |
|---|---|---|---|
| 运动学 Kinematics | “给定关节，末端在哪里？” | `q` | `T_base_tcp` |
| 逆运动学 IK | “给定末端目标，关节怎么摆？” | `T_target`、`q_seed` | 合法 `q` 或失败 |
| 路径规划 Path Planning | “沿哪里走？” | 起终点及约束 | `q(s)` 或 `T(s)` |
| 时间参数化 Time Parameterization | “多快走？” | 几何路径和运动限制 | `s(t)` |
| 轨迹规划 Trajectory Planning | “每个时刻应该处于什么状态？” | 路径、时间参数 | `q(t), dq(t), ddq(t)` |
| 轨迹执行 Trajectory Execution | “什么时间发什么命令？” | 时间轨迹 | 实时目标与状态 |
| 运动控制 Control | “电机如何实际跟踪目标？” | 目标、反馈、增益 | 位置/速度/力矩命令 |

### 1.2 术语：MoveJ、MoveL、Servo、Gravity

| 模式 | 输入主要是什么 | 约束的路径 | 对应实现 |
|---|---|---|---|
| **MoveJ** | 目标关节角 `q_target` | 主要是关节运动规律；TCP **不保证直线** | Ruckig + 轨迹检查 |
| **MoveL** | 目标 TCP 位姿 `T_target` | TCP 位置沿直线，姿态按定义插值 | 笛卡尔插值 + 连续 IK + TOPPRA |
| **MoveC** | 过渡点与终点等 | TCP 圆弧（需要定义平面和姿态规律） | 后续版本 |
| **ServoJ** | 连续更新的关节目标/速度 | 在线局部跟踪 | 实时控制与速度/加速度限制 |
| **ServoL** | TCP Twist：线速度和角速度 | 在线笛卡尔速度 | Jacobian、奇异性处理 |
| **Gravity** | 当前 `q, dq` | 不是几何路径，而是持续输出补偿力矩 | 动力学 `g(q)` + 阻尼 |
| **Impedance** | 参考位置与阻抗参数 | 与外力相关的柔顺行为 | 刚度/阻尼/动力学 |

---

## 2. 数学符号和维度速查表

### 2.1 希腊字母与特殊记号

| 记号 | 读法或名称 | 在本文的含义 | 典型维度 / 单位 |
|---|---|---|---|
| `\in` | 属于 | 例：`R ∈ SO(3)` 表示 `R` 是合法三维旋转矩阵 | 无 |
| `\mathbb{R}` | 实数集合 | `x ∈ R³` 表示 x 是三个实数组成的向量 | 由上下文确定 |
| `SO(3)` | 三维特殊正交群 | 合法三维旋转矩阵的集合 | 三维旋转，3 个独立自由度 |
| `SE(3)` | 三维特殊欧氏群 | 三维刚体位置与姿态的集合 | 刚体位姿，6 个独立自由度 |
| `so(3)` | SO(3) 的李代数 | 用反对称矩阵表达局部角运动 | 3 个独立量 |
| `se(3)` | SE(3) 的李代数 | 用 6 维 Twist 表达局部刚体运动 | 6 个独立量 |
| `q` | 关节配置 | 所有关节的位置变量 | 转动关节 rad |
| `\dot q` / `dq` | 关节速度 | 对时间的一阶导数 | rad/s |
| `\ddot q` / `ddq` | 关节加速度 | 对时间的二阶导数 | rad/s² |
| `\dddot q` / `j` | jerk（加加速度） | 对时间的三阶导数 | rad/s³ |
| `p` | 平移向量 | 坐标系原点位置 | 3×1，m |
| `R` | 旋转矩阵 | 一个坐标系相对另一个的姿态 | 3×3，无量纲 |
| `T` | 刚体变换 / 位姿 | 同时包括 `R` 和 `p` | 4×4，混合量纲 |
| `J(q)` | Jacobian | 关节速度到末端 Twist 的线性映射 | 通常 6×7 |
| `J^#` | 广义逆/伪逆 | 非方阵 Jacobian 的逆映射近似 | 通常 7×6 |
| `\lambda` | 阻尼系数 | 防止 IK 在奇异处数值放大 | 依 Jacobian 缩放约定 |
| `\tau` | 关节驱动力矩 | 电机作用在关节上的广义力 | N·m |
| `s` | 路径参数 | 规定沿几何路径的进度 | 一般 0～1，无量纲 |
| `t` | 时间 | 轨迹执行时刻 | s |
| `\Delta t` | 采样或控制周期 | 两次轨迹采样的间隔 | s |
| `\|x\|` | 范数 | 衡量向量大小，常用二范数 | 同 x 的单位 |
| `I_n` | 单位矩阵 | 对角线为 1 的 n×n 矩阵 | 无量纲 |
| `^T` | 转置 | 行列互换；如 `R^T` | — |
| `^{-1}` | 矩阵逆 | 仅对可逆方阵有定义 | — |
| `argmin` | 取最小值处的变量 | 优化时寻找“最好”的解 | — |

> **读公式的方法**：先看变量属于什么集合、是什么维度、有哪些单位，再看它们之间的映射关系。不要只记住公式而忽略坐标系。

### 2.2 Pinocchio 常见模型参数

| 字段 | 含义 | 注意事项 |
|---|---|---|
| `model.nq` | 配置向量 `q` 的长度 | 不一定等于关节数量；四元数配置尤其要注意 |
| `model.nv` | 切空间速度向量 `v` 的长度 | Jacobian 的列数是 `nv` |
| `model.njoints` | 模型关节对象数量 | 通常包含 universe 根关节，不等于独立运动轴数 |
| `model.nframes` | Frame 数量 | Frame 包含关节、连杆或工具坐标系等 |
| `frame_id` | 指定 TCP / 法兰等 Frame 的索引 | 必须从实际 URDF 名称获取 |
| `model` | 不随状态改变的机器人模型 | 关节树、惯量、几何等 |
| `data` | 可变的计算缓存 | 多线程通常各自维护一份 Data |

对于**固定底座、7 个单自由度旋转关节**的示例：通常 `nq = nv = 7`；但编写通用 SDK 时不要把这个关系硬编码到所有机器人模型。

---

## 3. 旋转与位姿：SO(3)、SE(3) 详解

### 3.1 `R \in SO(3)` 到底是什么意思？

公式：

$$
\boxed{R\in SO(3)}
$$

逐项解释：

| 符号 | 详细说明 |
|---|---|
| `R` | **旋转矩阵**；用来把向量从一个坐标系的表示转换到另一个坐标系的表示；大小为 3×3 |
| `\in` | 数学集合中的“属于”，不是“约等于”或者“包含” |
| `SO(3)` | Special Orthogonal Group in 3 Dimensions，即**三维特殊正交群**；它是所有合法三维纯旋转矩阵的集合 |
| `3` | 指三维空间（x、y、z），不是说矩阵里只有 3 个数字 |

这个集合的严格定义是：

$$
\boxed{SO(3)=\{R\in\mathbb R^{3\times3}\mid R^TR=I_3,\ \det(R)=+1\}}
$$

| 参数 / 条件 | 含义 | 为什么重要 |
|---|---|---|
| `R ∈ R^(3×3)` | R 是一个 3 行 3 列的实数矩阵 | 表达三维坐标的旋转变换 |
| `R^T R = I₃` | R 的列向量相互正交，长度为 1 | 旋转后不改变向量长度与夹角 |
| `I₃` | 3×3 单位矩阵 | `I₃ = diag(1,1,1)` |
| `det(R)=+1` | 行列式为 +1 | 排除镜像反射；镜像不是纯旋转 |

**特别注意**：`R` 有 9 个矩阵元素，但因为正交性等约束，实际上只有 **3 个独立旋转自由度**。所以 `SO(3)` 的“3”不是“矩阵含三个自由参数”的简写，而是三维空间旋转群的名称。

单位矩阵是最容易理解的合法旋转：

$$
R=I_3=\begin{bmatrix}1&0&0\\0&1&0\\0&0&1\end{bmatrix}
$$

它表示**姿态没有发生旋转**。

以绕 z 轴逆时针旋转 90° 为例（列向量、右手系约定）：

$$
R_z(\pi/2)=\begin{bmatrix}0&-1&0\\1&0&0\\0&0&1\end{bmatrix}
$$

如果原始向量为 $p=[1,0,0]^T$，那么旋转后：

$$
Rp=\begin{bmatrix}0&1&0\end{bmatrix}^{T}
$$

这说明 x 正方向被旋转到 y 正方向；向量长度仍为 1。式中的 $\pi$ 是圆周率，$\pi/2$ rad = 90°。这一例子可以帮助你理解：**旋转矩阵作用的是向量，而不只是存储三个姿态角**。

### 3.2 为什么不能把任意 3×3 矩阵当旋转矩阵？

例如：

$$
A=\begin{bmatrix}2&0&0\\0&1&0\\0&0&1\end{bmatrix}
$$

这个矩阵把 x 方向拉长两倍，是**缩放**，不是纯旋转，因为：

$$
A^TA\neq I_3
$$

如果在 SDK 中长期对旋转矩阵做数值计算，浮点误差也可能让其偏离正交约束；需要检查矩阵合法性，并按合适方法重新投影到 `SO(3)`（不能直接随意改动矩阵元素）。

### 3.3 `R_z(yaw)R_y(pitch)R_x(roll)` 如何理解？

采用常见的 **ZYX RPY 约定**：

$$
R=R_z(\psi)R_y(\theta)R_x(\phi)
$$

| 符号 | 说明 | 单位 |
|---|---|---|
| `R_x(φ)` | 绕 x 轴旋转的 3×3 矩阵 | 角度 `φ` 使用 rad |
| `R_y(θ)` | 绕 y 轴旋转的 3×3 矩阵 | `θ` 为 Pitch，rad |
| `R_z(ψ)` | 绕 z 轴旋转的 3×3 矩阵 | `ψ` 为 Yaw，rad |
| `φ` | Roll（横滚角） | rad |
| `θ` | Pitch（俯仰角） | rad |
| `ψ` | Yaw（偏航角） | rad |
| 乘法顺序 | 矩阵从右到左作用于列向量 | **不允许随意更换顺序** |

注意：不同软件可能使用不同内禀/外禀旋转、主动/被动旋转约定。你的 SDK 应明确规定输入 RPY 的转换定义，并用单元测试与 xArm、相机标定数据的约定对齐。**RPY 不建议作为内部唯一的姿态数据结构**，原因是会存在万向节锁等参数化奇异性。

### 3.4 四元数、轴角、李代数的关系

| 表示方法 | 参数数目 | 优点 | 工程注意事项 |
|---|---:|---|---|
| 旋转矩阵 `R` | 9 个数，3 个独立自由度 | 组合变换直观 | 必须正交、det=+1 |
| RPY 欧拉角 | 3 | 用户易理解 | 旋转顺序与奇异问题 |
| 单位四元数 `u=[w,x,y,z]` | 4 个数，3 个自由度 | 插值方便、数值稳定 | 必须归一化；`u` 与 `-u` 表示同一旋转 |
| 轴角 `(a,θ)` | 旋转轴 + 角度 | 便于理解局部旋转 | 轴需归一化；零角时轴不唯一 |
| `so(3)` 旋转向量 | 3 | 适合局部姿态误差和优化 | 对数映射在特殊角度处需谨慎 |

`SO(3)` 描述**旋转姿态本身**；`so(3)` 是描述其局部变化的李代数。常使用：

$$
R=\exp([\omega]_{\times})
$$

其中 `ω` 在该式中表示**旋转向量**（模长为旋转角，单位 rad），不是一定表示“角速度”；`[ω]×` 为对应的 3×3 反对称矩阵，`exp` 为矩阵指数。时间微分控制中，`ω` 又经常表示角速度，**必须结合上下文区分**。

### 3.5 `T \in SE(3)` 是什么？

$$
\boxed{T\in SE(3),\qquad T=\begin{bmatrix}R&p\\\mathbf{0}_{1\times3}&1\end{bmatrix}}
$$

其中：

| 符号 | 形状 | 物理意义 | 单位 |
|---|---|---|---|
| `T` | 4×4 | 齐次刚体变换；包含姿态和位置 | 混合量纲 |
| `R` | 3×3 | 合法旋转矩阵，`R ∈ SO(3)` | 无量纲 |
| `p=[x,y,z]^T` | 3×1 | 原点的空间平移 | m |
| 底行 `[0,0,0,1]` | 1×4 | 齐次坐标的数学结构 | — |
| `SE(3)` | 集合 | 三维空间所有合法刚体位姿的集合 | 6 个独立自由度 |

这里的 4×4 并不意味着 16 个变量都独立：刚体位姿只有**3 个平移 + 3 个旋转自由度**。

> Markdown 公式里写作 `T ∈ SE(3)`；它**不是**“T 是三维空间里的点”，而是“**T 是合法三维刚体变换矩阵**”。

### 3.6 坐标变换的上、下标应该怎么读？

设 `^{A}T_{B}` 表示**把在 B 坐标系表示的点变换到 A 坐标系表示**的矩阵，也描述 B 坐标系在 A 坐标系中的位姿。

$$
{}^{A}p={}^AT_B\,{}^{B}p
$$

这里的 `p` 使用**齐次向量** `[x,y,z,1]^T`。实际坐标变换链：

$$
\boxed{{}^{base}T_{tcp}={}^{base}T_{flange}\,{}^{flange}T_{tcp}}
$$

| 记号 | 含义 | 工程实例 |
|---|---|---|
| `base` | 机械臂基座坐标系 | 固定在底座 |
| `flange` | 机械臂法兰坐标系 | 机械臂结构的末端 |
| `tcp` | Tool Center Point，工具中心点坐标系 | 夹爪抓取点或剪刀切割点 |
| `^{base}T_{flange}` | 法兰相对于 base 的位姿 | 通过 FK 计算 |
| `^{flange}T_{tcp}` | TCP 相对于法兰的安装偏移 | 由夹爪几何/标定确定 |
| `^{base}T_{tcp}` | TCP 相对于 base 的位姿 | MoveL 通常控制的量 |

**错误示例**：把法兰与 TCP 当作同一个点。这样即使 FK 正确，工具尖端轨迹也可能错误。

---

## 4. 机械臂关节空间、坐标系与 URDF

### 4.1 关节变量 `q`

七个旋转关节的示意配置：

$$
q=\begin{bmatrix}q_1&q_2&q_3&q_4&q_5&q_6&q_7\end{bmatrix}^T\in\mathbb R^7
$$

| 符号 | 说明 | 典型单位 |
|---|---|---|
| `q_i` | 第 i 个关节的位置（本例为旋转角） | rad |
| `i` | 关节序号，`i=1..7` | — |
| `q` | 包含全部关节角度的列向量 | 7×1，rad |
| `R⁷` | 7 维实数向量空间 | — |
| 上标 `T` | 把行向量转成列向量 | — |

这只是固定底座 7 旋转关节的教学简化。通用 Pinocchio 模型中，`q ∈ configuration space`，不一定都能当作普通欧氏向量相加。

### 4.2 实体机器人的三套角度要区分

1. **模型关节角**：URDF / Pinocchio 使用的 `q_model`。
2. **电机角**：电机编码器或驱动协议返回的 `q_motor`。
3. **电机零位/偏置**：从电机角到实际关节角的标定信息。

在简单、经过标定的单轴传动中，可以定义示意关系：

$$
q_{joint}=\sigma\frac{q_{motor}-q_{offset}}{N}
$$

| 参数 | 含义 | 单位 / 取值 |
|---|---|---|
| `q_joint` | 机器人模型使用的关节角 | rad |
| `q_motor` | 电机编码器返回的电机侧角度 | rad |
| `q_offset` | 电机侧零点偏移 | rad |
| `N` | 电机侧到关节侧的减速比，定义为电机转角/关节转角 | 无量纲、正数 |
| `σ` | 安装导致的方向映射 | `+1` 或 `-1` |

**实际是否需要除以 `N` 由反馈协议决定**：有些驱动已返回关节侧角度；此时再次除以减速比会出错。不能把上式未经确认地应用于 DM-MC02。

### 4.3 URDF 中要核对的项目

| 参数 | 用途 | 典型错误 |
|---|---|---|
| `origin xyz/rpy` | 关节和连杆之间的固定变换 | 旋转顺序弄错 |
| `axis xyz` | 关节旋转轴 | 电机方向与模型符号不一致 |
| `limit lower/upper` | 关节位置限制 | 单位混用、限位范围不正确 |
| `limit velocity/effort` | 运动约束的参考数据 | 误当为已验证的实机安全值 |
| `inertial mass` | 连杆质量 | 缺失会影响重力补偿 |
| `inertial origin` | 惯性质心位置和方向 | 质心偏移会造成力矩计算错误 |
| `inertia ixx...` | 惯性张量 | 错误惯量影响动力学 |
| `tcp frame` | 实际受控末端 | 使用了法兰帧而非工具帧 |

**约定**：参数来源必须有记录；未确认真实硬件限值时，不能把 URDF 中的值直接用作可执行的电机限幅。

---

## 5. 正运动学 FK 与 Pinocchio

### 5.1 定义

$$
\boxed{T_{tcp}=FK(q)}
$$

| 变量 | 定义 | 输入/输出 |
|---|---|---|
| `FK` | Forward Kinematics，正运动学函数 | 函数 |
| `q` | 当前关节配置 | 输入 |
| `T_tcp` | TCP 相对基座的位姿，属于 `SE(3)` | 输出 |

平面二连杆示例：

$$
\begin{aligned}
x&=L_1\cos q_1+L_2\cos(q_1+q_2)\\
y&=L_1\sin q_1+L_2\sin(q_1+q_2)
\end{aligned}
$$

| 参数 | 含义 | 单位 |
|---|---|---|
| `L₁, L₂` | 两段连杆长度 | m |
| `q₁, q₂` | 两个关节角度 | rad |
| `x, y` | 末端在平面上的位置 | m |
| `cos/sin` | 三角函数 | 输入使用 rad |

七自由度机械臂的原理相同，但需要按 URDF 定义的运动学树依次组合变换。

### 5.2 Pinocchio 的 `Model` 与 `Data`

```cpp
pinocchio::Model model; // 机器人静态模型：关节树、连杆惯量、Frame 等
pinocchio::Data data(model); // 当前算法的可变计算缓存
```

`Model` 是“机器人是什么”，`Data` 是“在当前 q 下计算出了什么”。

### 5.3 C++ FK 示例（离线，无电机控制）

> 此代码演示 Pinocchio 常见 API；需要先安装匹配版本的 Pinocchio，且使用真实 URDF 内存在的 TCP Frame 名称。示例只读取模型，不涉及 CAN 或电机使能。

```cpp
#include <iostream>
#include <string>
#include <Eigen/Core>
#include <pinocchio/parsers/urdf.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>

int main(int argc, char** argv) {
    if (argc != 3) {
        std::cerr << "usage: fk_demo robot.urdf tcp_frame\n";
        return 1;
    }

    pinocchio::Model model;
    pinocchio::urdf::buildModel(argv[1], model); // 从 URDF 构建运动学模型
    pinocchio::Data data(model);                 // 为本线程创建计算缓存

    const std::string tcp_name = argv[2];
    if (!model.existFrame(tcp_name)) {           // 检查 TCP Frame 名称是否存在
        std::cerr << "Unknown frame: " << tcp_name << '\n';
        return 2;
    }
    const auto frame_id = model.getFrameId(tcp_name);

    // neutral(model) 是模型定义的中性配置，不等于实机安全的上电姿态
    Eigen::VectorXd q = pinocchio::neutral(model);

    pinocchio::forwardKinematics(model, data, q); // 计算关节变换
    pinocchio::updateFramePlacements(model, data);// 更新 Frame 位姿缓存

    const pinocchio::SE3& T = data.oMf[frame_id]; // world/model 原点到 TCP
    std::cout << "nq=" << model.nq << " nv=" << model.nv << '\n';
    std::cout << "position (m): " << T.translation().transpose() << '\n';
    std::cout << "R (SO(3)):\n" << T.rotation() << '\n';
}
```

**结果如何检查？**

- `T.rotation().transpose() * T.rotation()` 应接近单位矩阵。
- `det(T.rotation())` 应接近 +1。
- 改变某个 `q_i` 时，末端位置/姿态应随对应关节轴规律变化。
- 不要用任意编造的 TCP Frame 名称；Frame 必须来自实际模型。

---

## 6. Jacobian：速度运动学

### 6.1 数学含义

$$
\boxed{V=J(q)\dot q}
$$

| 变量 | 形状（7 旋转关节示例） | 详细含义 | 单位 |
|---|---|---|---|
| `q` | 7×1 | 当前关节角 | rad |
| `dq = q̇` | 7×1 | 关节角速度 | rad/s |
| `J(q)` | 6×7 | 把关节速度映射为 TCP 六维空间速度 | 上 3 行 m/rad、下 3 行 1（按所用约定） |
| `V` | 6×1 | TCP Twist（空间速度） | 上 3 分量 m/s、下 3 分量 rad/s |

本文在解释物理意义时采用：

$$
V=\begin{bmatrix}v_x&v_y&v_z&\omega_x&\omega_y&\omega_z\end{bmatrix}^{T}
$$

其中 `v` 是线速度，`ω` 是角速度。实际读取 Pinocchio `Motion` 或 Jacobian 的原始数据时，要核对库对应类型的分量顺序和参考系，不能把角速度误当作 RPY 的导数。

### 6.2 参考系：`LOCAL`、`WORLD`、`LOCAL_WORLD_ALIGNED`

Pinocchio 允许在不同参考系下表达 Frame Jacobian：

| 模式 | 表达方式 | 何时用 | 易错点 |
|---|---|---|---|
| `LOCAL` | 在 TCP 自身坐标系中表达的空间速度 | Body IK、局部工具运动 | 误把局部 x 方向当世界 x |
| `LOCAL_WORLD_ALIGNED` | 原点位于 TCP、坐标轴方向与世界系对齐 | 世界系表达的 TCP 线速度控制 | 与 `WORLD` 误认为完全相同 |
| `WORLD` | 在世界原点表示的空间速度 | 特定空间运动计算 | 空间速度所参考的原点不同 |

**重要**：三个模式虽然都可能返回 6×`nv` 的矩阵，但几何含义不完全相同。目标速度/位姿误差必须与所用 Jacobian 采取一致的参考系。官方说明见参考资料。

### 6.3 Jacobian 计算示意

```cpp
// 前提：已建立 model、data、q，且 tcp_frame_id 有效。
// 表达：以 TCP 为原点，坐标轴与世界系对齐。
Eigen::MatrixXd J = pinocchio::computeFrameJacobian(
    model,
    data,
    q,
    tcp_frame_id,
    pinocchio::LOCAL_WORLD_ALIGNED
);
// J 的行数为 6，列数为 model.nv。
```

### 6.4 为什么不能直接求逆？

对常见 7 轴机械臂：`J ∈ R^(6×7)`，不是方阵，普通 `J^{-1}` 不存在。可使用伪逆 `J^#`：

$$
\dot q=J^{\#}V_{target}
$$

这里：

- `J^#`：Moore–Penrose 伪逆；通常 7×6。
- `V_target`：想让 TCP 产生的六维空间速度，单位为线速度 m/s 和角速度 rad/s。
- `dq`：计算得到的关节速度，rad/s。

### 6.5 阻尼最小二乘法 DLS

$$
\boxed{\dot q=J^T(JJ^T+\lambda^2I_6)^{-1}V_{target}}
$$

| 变量 | 说明 | 典型形状 |
|---|---|---|
| `J` | 当前配置对应的 Jacobian | 6×7 |
| `J^T` | Jacobian 转置 | 7×6 |
| `JJ^T` | 任务空间 Gram 矩阵 | 6×6 |
| `I₆` | 6×6 单位矩阵 | 6×6 |
| `λ` | 阻尼量，防止接近奇异位形时解放大 | 标量，非负 |
| `V_target` | 期望 TCP Twist | 6×1 |
| `dq` | 输出关节速度 | 7×1 |

**优点**：降低接近奇异位形时关节速度的爆炸风险。  
**代价**：末端速度可能不能被精确实现。

由于 Jacobian 的线速度行和角速度行物理单位不同，通用求解器还应明确使用的任务权重/尺度，否则固定 `λ` 的数值不具备跨任务的可比性。实用实现还需处理速度限制和关节约束。

### 6.6 七自由度零空间

$$
\boxed{\dot q=J^{\#}V+(I_7-J^{\#}J)\dot q_0}
$$

| 参数 | 含义 |
|---|---|
| `I₇` | 7×7 单位矩阵 |
| `dq₀` | 自己希望做的次要关节运动，例如向关节中位运动 |
| `I₇ - J^# J` | 零空间投影（精确伪逆时） |
| 第一项 | 主要任务：跟踪 TCP 目标速度 |
| 第二项 | 次要任务：尽量不影响主要末端运动 |

如使用**阻尼伪逆**，该投影一般不是完全精确的零空间投影；实践中可能需要任务优先级求解或专门的约束优化方法。

---

## 7. 逆运动学 IK：DLS 与七自由度冗余

### 7.1 逆运动学问题

$$
\boxed{q^*=IK(T_{target},q_{seed})}
$$

| 参数 | 说明 | 单位 / 类型 |
|---|---|---|
| `T_target` | 目标 TCP 位姿 | `SE(3)` |
| `q_seed` | IK 初始关节配置；通常来自当前反馈或上一采样点 | rad |
| `q*` | 满足目标位姿与合法性约束的候选解 | rad |
| `IK` | 逆运动学求解算法；可能失败或得到多组不同解 | 算法 |

对于常见七自由度、六维 TCP 任务，在一般非奇异点附近可存在 1 个局部冗余自由度（并不代表所有姿态一定可达）。

### 7.2 为什么需要 SE(3) 误差？

不能简单地对旋转矩阵做减法再把 9 个元素当作姿态误差。通常先计算**相对位姿**：

$$
T_{error}=T_{current}^{-1}T_{target}
$$

再使用李群对数得到局部六维误差：

$$
e=\mathrm{log}_6(T_{error})\in\mathbb R^6
$$

| 参数 | 含义 |
|---|---|
| `T_current` | 当前 TCP 位姿，由 FK 得到 |
| `T_target` | 期望 TCP 位姿 |
| `T_error` | 从当前到目标的相对位姿 |
| `log₆` | SE(3) 对数映射；从位姿差得到 6 维局部运动误差 |
| `e` | 6 维局部误差向量，平移和旋转分量采用同一参考系约定 |

**非常重要**：把 `log6()` 当作误差定义后，其对关节配置的导数**不总是直接等于原始 Frame Jacobian**。Pinocchio 官方 IK 教程使用 `Jlog6` 校正任务 Jacobian，并对符号/左右乘约定进行处理；照搬 `e = log6(...)` 和 `J = computeFrameJacobian(...)` 而不匹配约定，很容易出现“不收敛或反方向”的问题。

### 7.3 DLS IK 迭代逻辑

```text
输入 T_target, q_seed
          │
          ▼
q ← q_seed
          │
          ▼
T_current ← FK(q)
          │
          ▼
e ← SE(3) 位姿误差
          │
          ├── 位置和姿态误差均合格 → SUCCESS(q)
          │
          ▼
J_task ← 对应误差定义的任务 Jacobian
          │
          ▼
Δq ← 带阻尼的最小二乘求解
          │
          ▼
步长控制、关节限位、奇异性检查
          │
          ▼
q ← integrate(model, q, Δq)
          │
          └── 循环直到收敛 / 超时 / 失败
```

建议 IK 参数设计：

| 参数名 | 含义 | 单位 | 如何使用 |
|---|---|---|---|
| `position_tolerance` | TCP 平移误差允许值 | m | 判断位置收敛 |
| `orientation_tolerance` | TCP 姿态误差允许值 | rad | 判断旋转收敛 |
| `max_iterations` | 最大迭代次数 | 次 | 防止无限求解 |
| `damping` | DLS 阻尼系数 | 依缩放方案 | 抑制奇异数值放大 |
| `step_scale` | 迭代步长比例 | 无量纲 | 避免一步跨越过远 |
| `max_joint_step` | 单步允许关节变化 | rad/步 | 防止关节解跳变 |
| `seed` | 初始关节配置 | rad | 影响收敛到哪一支解 |
| `joint_limit_margin` | 离关节边界保留余量 | rad | 提前规避硬限位 |

这些参数数值应通过离线回归测试确定，**本文不提供未经硬件/模型验证的生产默认值**。

### 7.4 验收：比较 `FK(IK(T))`，而不是只比 `q`

因为七自由度冗余，可能存在 `q_solution ≠ q_original`，但它们都能到达同一个 TCP 位姿。

建议测试：

1. 随机生成合法 `q_test`。
2. 计算 `T_test = FK(q_test)`。
3. 以不同合法 `seed` 求 `q_solution = IK(T_test, seed)`。
4. 验证 `FK(q_solution)` 与 `T_test` 的平移、旋转误差。
5. 验证关节限位、解连续性、自碰撞等约束。

---

## 8. 路径、轨迹与 MoveJ

### 8.1 路径参数与时间的区别

关节空间直线路径：

$$
\boxed{q(s)=q_0+s(q_1-q_0),\quad s\in[0,1]}
$$

| 记号 | 含义 |
|---|---|
| `q₀` | 起点关节角，rad |
| `q₁` | 终点关节角，rad |
| `s` | 路径进度，没有时间单位 |
| `q(s)` | 路径上不同进度对应的关节角 |

只有给出时间规律 `s(t)` 后，才能得到真正的**轨迹** `q(t)`。

### 8.2 位置、速度、加速度与 jerk

$$
\begin{aligned}
q(t)&:\text{位置}\\
\dot q(t)&=\frac{dq}{dt}:\text{速度}\\
\ddot q(t)&=\frac{d^2q}{dt^2}:\text{加速度}\\
\dddot q(t)&=\frac{d^3q}{dt^3}:\text{jerk}
\end{aligned}
$$

| 量 | 七轴数据长度 | 单位 | 限制目的 |
|---|---:|---|---|
| `q` | 7 | rad | 不越过关节限位 |
| `dq` | 7 | rad/s | 不超过关节速度能力 |
| `ddq` | 7 | rad/s² | 降低过高惯性力矩与冲击 |
| `jerk` | 7 | rad/s³ | 限制加速度突变 |

### 8.3 三次、五次和 S 曲线有什么区别？

- **三次多项式**：可以通过四个边界条件约束起止位置和速度。
- **五次多项式**：可以通过六个边界条件约束起止位置、速度和加速度。
- **jerk 受限 S 曲线**：进一步约束加速度变化率，是设计平滑 MoveJ 的实用方法。

五次多项式常见的静止到静止时间缩放：

$$
\boxed{s(u)=10u^3-15u^4+6u^5,\quad u=t/T}
$$

| 参数 | 说明 | 单位 |
|---|---|---|
| `u` | 归一化时间，起点 0、终点 1 | 无量纲 |
| `t` | 当前时间 | s |
| `T` | 总运动时间，`T>0` | s |
| `s(u)` | 无量纲路径进度 | 0～1 |

五次多项式**不会自动保证**所有关节速度、加速度和 jerk 都符合硬件约束，因此不能把它当成完整的工业轨迹规划器。

### 8.4 Ruckig 参数详细说明

Ruckig 对离线 MoveJ 最直接的用途：输入当前状态、目标状态及运动约束，输出在时间上可采样的平滑轨迹。

| Ruckig 参数 | 说明 | 典型单位（旋转关节） | 必须注意 |
|---|---|---|---|
| `current_position` | 轨迹起始关节位置 | rad | 必须与反馈/模型关节顺序一致 |
| `current_velocity` | 起始关节速度 | rad/s | 运动中重规划时通常非零 |
| `current_acceleration` | 起始加速度 | rad/s² | 必须与连续轨迹状态匹配 |
| `target_position` | 终点关节位置 | rad | 先验证关节限位 |
| `target_velocity` | 终点速度 | rad/s | 静止到静止通常设零 |
| `target_acceleration` | 终点加速度 | rad/s² | 静止到静止通常设零 |
| `max_velocity` | 每关节最大速度 | rad/s | 必须来自已确认限值 |
| `max_acceleration` | 每关节最大加速度 | rad/s² | 必须来自已确认限值 |
| `max_jerk` | 每关节最大 jerk | rad/s³ | 决定加速度变化有多快 |
| `delta_time` | 输出采样周期 | s | 不是实时性保证 |
| `synchronization` | 多关节同步策略 | 枚举 | 需根据版本与任务选择 |
| `control_interface` | 位置/速度目标接口 | 枚举 | 并非自动切换真实电机模式 |
| `minimum_duration` | 最小轨迹时长（可选） | s | 不能代替速度/加速度限制 |

> **版本边界**：Ruckig Community 的部分中间路点能力可能通过云端 API；本项目要求离线运行时，MoveJ 使用本地单段在线轨迹生成能力。关节位置界限及部分选项在 Pro/不同版本中的可用性需要核对，不得假设所有接口都存在。

### 8.5 Ruckig C++ 离线示例（7 轴）

```cpp
#include <ruckig/ruckig.hpp>
#include <iostream>

int main() {
    constexpr size_t DOF = 7;   // 教学示例：固定底座 7 个转动关节
    constexpr double dt = 0.01; // 10 ms 轨迹采样时间，非实机频率保证

    ruckig::Ruckig<DOF> planner(dt);
    ruckig::InputParameter<DOF> input;
    ruckig::OutputParameter<DOF> output;

    input.current_position     = {0, 0, 0, 0, 0, 0, 0};
    input.current_velocity     = {0, 0, 0, 0, 0, 0, 0};
    input.current_acceleration = {0, 0, 0, 0, 0, 0, 0};

    input.target_position      = {0.2, -0.3, 0.1, 0, 0.15, 0, 0};
    input.target_velocity      = {0, 0, 0, 0, 0, 0, 0};
    input.target_acceleration  = {0, 0, 0, 0, 0, 0, 0};

    // 以下上限是脱机练习的示例数值，绝非 OpenArm 实机限值。
    input.max_velocity     = {0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5};
    input.max_acceleration = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0};
    input.max_jerk         = {2.0, 2.0, 2.0, 2.0, 2.0, 2.0, 2.0};

    ruckig::Result result;
    do {
        result = planner.update(input, output);
        if (static_cast<int>(result) < 0) {
            std::cerr << "Ruckig error\n";
            return 1;
        }

        // 仅采样轨迹数据，不能直接理解为已执行实体关节运动。
        // output.new_position / new_velocity / new_acceleration 可保存为 CSV。
        output.pass_to_input(input);
    } while (result == ruckig::Result::Working);
}
```

`pass_to_input()` 将上一采样输出的状态作为下一次规划输入，适用于 Ruckig 示例用法；真实闭环系统还必须检查**实测状态**与计划状态的偏差。不同版本 API 若发生变化，以锁定版本的官方文档为准。

---

## 9. MoveL：笛卡尔路径与连续 IK

### 9.1 为什么 `MoveJ(IK(target))` 不是 MoveL？

因为先对末端终点求一次 IK，再进行关节插补，只约束了**关节空间轨迹**。TCP 在中间时刻通常会走一条曲线。

MoveL 正确的大致顺序：

```text
起点/终点 TCP 位姿
      ↓
笛卡尔平移直线 + 姿态插值
      ↓
连续求解 IK：T(s) → q(s)
      ↓
检查连贯性、限位、碰撞、奇异性
      ↓
TOPPRA / 时间参数化
      ↓
重建并复检 TCP 路径
      ↓
Trajectory Executor
```

### 9.2 平移直线插值

$$
\boxed{p(s)=(1-s)p_0+sp_1,\quad s\in[0,1]}
$$

| 参数 | 详细说明 | 单位 |
|---|---|---|
| `p₀` | 起点 TCP 位置 `[x₀,y₀,z₀]^T` | m |
| `p₁` | 终点 TCP 位置 `[x₁,y₁,z₁]^T` | m |
| `s` | 路径进度 | 无量纲 |
| `p(s)` | 插值后的 TCP 位置；位于两点连线段上 | m |

如果需要姿态变化，可使用四元数球面线性插值：

$$
R(s)=\mathrm{SLERP}(R_0,R_1,s)
$$

| 参数 | 含义 |
|---|---|
| `R₀、R₁` | 起点和终点 TCP 姿态，属于 SO(3) |
| `SLERP` | Spherical Linear Interpolation，球面线性插值；实际常对单位四元数实现 |
| `R(s)` | 插值生成的姿态，保持为合法旋转 |

### 9.3 连续 IK 的作用

对每个路径样本 `T(s_i)`，以**上一个成功解**作为新 seed：

$$
q_i=IK(T(s_i),q_{i-1})
$$

| 参数 | 含义 |
|---|---|
| `s_i` | 第 i 个路径采样点的路径进度 |
| `T(s_i)` | 第 i 个目标 TCP 位姿 |
| `q_(i-1)` | 上一采样点的成功关节配置 |
| `q_i` | 当前样本的关节配置 |

这样能够降低分支跳变，但**不能保证**一定没有跳变、碰撞或奇异。必须验证连续 IK 的离散步长、关节差分、加速度可行性以及路径碰撞。

### 9.4 关键工程问题：离散路径不自动连续

仅在几个采样点通过 IK，不表示两个采样点之间一定沿完美的 TCP 直线。

- 如果执行器在相邻 `q_i` 之间做关节空间插值，中间 TCP 可能离开目标直线。
- 如果对 `q(s)` 做普通平滑样条，可能改变 TCP 几何路径。
- 如果只检查离散点碰撞，可能漏掉中间姿态的碰撞。

因此建议**自适应细分路径 + FK 重新投影检查 + 路径误差验证**；对需要严格几何精度的任务，使用能保持任务空间约束的规划和执行方式。

### 9.5 MoveL 常用参数

| 参数 | 单位 | 含义 | 注意 |
|---|---|---|---|
| `target_pose` | SE(3) | 目标 TCP 位姿 | 必须指定 base/world 参考系 |
| `linear_speed` | m/s | TCP 平移线速度限制 | 不是关节速度限制 |
| `angular_speed` | rad/s | TCP 姿态变化速度限制 | 大姿态变化时不可忽略 |
| `max_linear_acceleration` | m/s² | TCP 平移加速度上限 | 需检查路径时间规律 |
| `path_resolution` | m 或归一化步长 | 几何采样间距 | 间距越小计算越重 |
| `position_tolerance` | m | TCP 线性路径偏差容差 | 应验证整个路径 |
| `orientation_tolerance` | rad | 姿态误差容差 | 注意姿态插值定义 |
| `ik_seed` | rad | 连续 IK 起始参考关节角 | 优先真实反馈值 |
| `collision_check` | bool | 是否执行碰撞检查 | 禁用碰撞检查不能宣称轨迹安全 |

---

## 10. TOPPRA：路径时间参数化

### 10.1 什么是 `q(s)`、`q'(s)` 和 `q''(s)`？

`q(s)` 描述机器人沿几何路径的配置；`s` 是路径进度，而不是时间。

$$
q'(s)=\frac{dq(s)}{ds},\qquad q''(s)=\frac{d^2q(s)}{ds^2}
$$

| 记号 | 对什么求导 | 表示什么 | s 为无量纲时的单位 |
|---|---|---|---|
| `q(s)` | — | 几何路径上的关节角 | rad |
| `q'(s)` | 对路径进度 s 一阶求导 | 路径进度变化一点，关节角变化多少 | rad |
| `q''(s)` | 对路径进度 s 二阶求导 | 关节路径本身弯曲/变化的程度 | rad |
| `s(t)` | — | 执行时间对应的路径进度 | 无量纲 |
| `ṡ` | 对时间求导 | 路径进度速度 | 1/s |
| `s̈` | 对时间二阶求导 | 路径进度加速度 | 1/s² |

因此链式法则：

$$
\boxed{\dot q=q'(s)\dot s}
$$

$$
\boxed{\ddot q=q''(s)\dot s^2+q'(s)\ddot s}
$$

**理解这两式是理解 TOPPRA 的关键**：

- `q(s)` 决定“必须走哪条路”。
- `s(t)` 决定“什么时候走到哪里”。
- 通过限制 `ṡ` 和 `s̈`，间接满足每个关节的速度和加速度约束。

### 10.2 TOPPRA 要解决的问题

输入：一条已经合法、足够光滑的关节路径 `q(s)`，以及速度/加速度等约束。  
输出：一个满足约束的路径时间规律 `s(t)`，进而得到 `q(t)`。

| 常见参数 | 解释 | 单位 |
|---|---|---|
| `path` | 几何路径 `q(s)` | rad |
| `gridpoints` | 优化中使用的路径离散网格 | 路径参数 s |
| `velocity_limits` | 每个关节速度上下限 | rad/s |
| `acceleration_limits` | 每个关节加速度上下限 | rad/s² |
| `duration` | 规划得到的运动总时长 | s |
| `s(t)` | 规划得到的路径进度函数 | 无量纲 |

TOPPRA 不是完整 IK 求解器，也不会自动找到避障路径。基础版本的速度/加速度时间参数化也**不等于自动保证 jerk 限制**。若后处理对轨迹做平滑或重新采样，需要重新验证几何路径与全部运动约束。

### 10.3 为什么 MoveJ 与 MoveL 不能只用一个规划器？

- MoveJ 更适合直接在关节状态空间中规划平滑运动。
- MoveL 必须先满足 TCP 的**几何路径约束**，再沿该路径进行时间参数化。
- 对已经生成的 MoveL 关节点调用普通 MoveJ 平滑器，可能让 TCP 脱离目标直线。

推荐项目初版：**MoveJ 使用 Ruckig，本地 MoveL 使用笛卡尔插值 + 连续 IK + 时间参数化**。

---

## 11. 轨迹执行与电机控制模式

### 11.1 规划输出不等于实体运动

轨迹规划完成后，得到：

$$
q_d(t),\quad \dot q_d(t),\quad \ddot q_d(t)
$$

其中 `d` = desired，表示期望值。执行器负责把期望状态转化为按时间下发的命令，并监测实际反馈 `q_a(t)`。

### 11.2 常用控制模式和参数

| 模式 | 主要输入 | 控制目标 | 前提 |
|---|---|---|---|
| Position | `q_d` | 让关节趋近目标角度 | 电机支持可靠的位置伺服 |
| Velocity | `dq_d` | 控制关节转速 | 速度限制和通信安全 |
| Torque / Effort | `τ_d` | 输出期望力矩 | 真实有效的力矩控制能力 |
| MIT / Hybrid | `q_d, dq_d, Kp, Kd, τ_ff` | 组合位置、速度及前馈力矩 | 必须核对电机协议、参数编码 |
| Gravity Compensation | `q, dq, g(q)` | 抵消自重并提供适当阻尼 | 准确动力学模型、力矩符号验证 |
| Joint Impedance | `q_d, dq_d, Kp, Kd, g(q)` | 刚度/阻尼形式的柔顺跟踪 | 模式切换与力矩安全 |

MIT 风格示意式：

$$
\boxed{\tau=K_p(q_d-q)+K_d(\dot q_d-\dot q)+\tau_{ff}}
$$

| 参数 | 详细说明 | 理想物理单位 |
|---|---|---|
| `τ` | 目标关节输出力矩 | N·m |
| `Kp` | 位置误差对应的刚度系数 | N·m/rad |
| `Kd` | 速度误差对应的阻尼系数 | N·m·s/rad |
| `q_d` | 期望角度 | rad |
| `q` | 实际反馈角度 | rad |
| `dq_d` | 期望角速度 | rad/s |
| `dq` | 实际角速度 | rad/s |
| `τ_ff` | 前馈力矩，例如重力补偿项 | N·m |

**注意**：电机协议中 `Kp/Kd/torque` 的数值编码范围、标度、是否代表电机侧还是关节侧量，要以对应电机与固件文档为准。不能直接把上表的理想 SI 单位数值按原样写入 CAN 帧。

### 11.3 执行器所需的实时参数

| 参数 | 说明 | 单位 |
|---|---|---|
| `control_period` | 轨迹更新/命令周期的设计值 | s |
| `state_timeout` | 多久收不到反馈视为状态失效 | s |
| `command_timeout` | 多久没收到有效命令触发保护 | s |
| `following_error_limit` | 允许的最大关节跟踪误差 | rad |
| `velocity_limit` | 关节速度上限 | rad/s |
| `effort_limit` | 允许的输出力矩上限 | N·m |
| `stop_deceleration` | 受控停止允许的减速度 | rad/s² |
| `jitter` | 实际控制时刻与期望时刻的偏差 | s |

**特别注意**：

1. Linux `sleep_for(10ms)` 不保证执行周期精确等于 10ms。
2. CAN 发送成功不等于电机已执行；需要确认状态反馈。
3. 任何轨迹执行必须有通信超时与故障处理。
4. 对于可能因重力坠落的机械臂，直接切断力矩不等于安全停止。
5. 开发容器默认不连接实机；实体控制应在通过明确安全门槛后独立验证。

---

## 12. 动力学、零重力与阻抗控制

### 12.1 动力学与运动学有什么区别？

运动学只回答**位置/速度关系**；动力学进一步回答**为了产生运动需要多少力矩**。

简化的关节动力学方程：

$$
\boxed{M(q)\ddot q+h(q,\dot q)+g(q)=\tau}
$$

| 符号 | 说明 | 形状 / 单位 |
|---|---|---|
| `M(q)` | 质量/惯性矩阵，随姿态变化 | 7×7，适当的惯量单位 |
| `ddq` | 关节加速度 | 7×1，rad/s² |
| `h(q,dq)` | 科氏、离心等与速度相关项 | 7×1，N·m |
| `g(q)` | 机械臂自重在各关节形成的重力力矩 | 7×1，N·m |
| `τ` | 总关节驱动力矩（该简化模型暂未写摩擦和外力） | 7×1，N·m |

实际模型还可能包含摩擦、传动、外部接触力和负载。方程约定的正负号必须与实际模型一致。

### 12.2 零重力控制

基础补偿式：

$$
\boxed{\tau_g=g(q)}
$$

加粘性阻尼的示意式：

$$
\boxed{\tau=g(q)-K_d\dot q}
$$

| 参数 | 含义 | 单位 |
|---|---|---|
| `τ_g` | 关节重力补偿前馈力矩 | N·m |
| `g(q)` | Pinocchio 根据质量、质心、重力方向计算的重力项 | N·m |
| `Kd` | 阻尼系数或对角阻尼矩阵 | N·m·s/rad |
| `dq` | 实际关节速度 | rad/s |

这只是控制律原理，**不可直接作为无防护实机操作指令**。重力补偿尤其依赖质量/质心参数、安装方向、关节符号、夹爪和负载标定。

### 12.3 关节阻抗

$$
\boxed{\tau=K_p(q_d-q)+K_d(\dot q_d-\dot q)+g(q)}
$$

`Kp` 决定靠近目标的“弹簧刚度”，`Kd` 决定抑制振动的“阻尼”；这不是在任何接触场景下都可以直接应用的完整安全控制律。

### 12.4 笛卡尔阻抗与 `J^T`

简单的任务空间弹簧-阻尼律：

$$
F=K_x(x_d-x)+D_x(\dot x_d-\dot x)
$$

关节力矩映射：

$$
\boxed{\tau=J(q)^T F+g(q)}
$$

| 参数 | 含义 | 单位 / 注意 |
|---|---|---|
| `F` | TCP 6 维广义力（力+力矩，Wrench） | 前 3 N、后 3 N·m（按选定次序） |
| `Kx` | 任务空间刚度矩阵 | 平移和旋转块单位不同 |
| `Dx` | 任务空间阻尼矩阵 | 平移和旋转块单位不同 |
| `J^T` | Jacobian 转置 | 将与 J 一致参考系的 Wrench 转为广义关节力矩 |
| `x_d - x` | 位姿误差的简化写法 | 完整六维姿态应使用一致的 SE(3) 误差定义 |

如果 Wrench 和 Jacobian 不在同一参考系、没有使用一致的参考点，将产生错误力矩。

---

## 13. 参数配置规范与常见误区

### 13.1 推荐配置字段（教学模板，非真实参数）

```yaml
robot:
  model_file: "models/openarm.urdf"   # URDF 路径；必须是实际存在的模型
  base_frame: "base_link"            # 参考基座坐标系；以 URDF 实际 Frame 为准
  tcp_frame: "tool0"                 # 仅为示例名称；必须核对真实 TCP Frame

kinematics:
  ik_method: "damped_least_squares"  # DLS 数值逆运动学
  position_tolerance_m: null         # 位置收敛阈值，m；待测试决定
  orientation_tolerance_rad: null    # 姿态收敛阈值，rad；待测试决定
  max_iterations: null              # 单次 IK 最大迭代次数；待测试决定
  damping: null                     # DLS 阻尼系数；待任务缩放方案确定

planning:
  movej_backend: "ruckig"           # MoveJ 轨迹生成库
  movel_backend: "cartesian_ik_toppra" # MoveL：笛卡尔路径+IK+时间参数化
  sample_period_s: null             # 轨迹采样周期，s；待验证
  collision_check_required: true    # 启用路径碰撞检查的设计要求

hardware:
  enable_real_motor: false          # 默认禁止真实电机运动
  transport: "mock"                 # Mock Driver，仅离线使用
  motor_limits_verified: false      # 未经逐关节实测不得设为 true
```

这里使用 `null` 是刻意标记**参数尚未确定**，而不是建议用 0 或随意填写默认值。实际解析器应明确拒绝缺失但启动时必须提供的配置，并给出可读错误信息。

### 13.2 参数单位应显式编码到名称中

优先：`linear_speed_m_s`、`joint_speed_rad_s`、`position_tolerance_m`。  
避免：`speed: 100`、`angle: 90` 这类无法判断单位的配置。

### 13.3 高频错误排查表

| 现象 | 可能原因 | 排查方法 |
|---|---|---|
| TCP 位置正确、姿态明显错误 | RPY 顺序不一致 | 检查 `Rz Ry Rx` 约定和姿态构造 |
| `FK(q)` 与实机位置不符 | 电机零位、轴方向、减速比、TCP 偏移有误 | 检查映射参数与 URDF |
| IK 发散或朝反方向迭代 | SE(3) 误差、Jacobian 参考系或符号不一致 | 对照 Pinocchio IK 教程 `Jlog6` |
| 靠近某姿态关节速度突然增大 | 奇异或近奇异位形 | 检查奇异值、DLS 阻尼、速度限制 |
| MoveJ 平滑但 TCP 绕弯 | 关节空间轨迹并不要求末端直线 | 使用 MoveL 规划 |
| MoveL 采样点都正确，中间却偏离直线 | 相邻关节点插补改变了路径 | 用 FK 检查连续执行轨迹 |
| 路径合规但电机跟踪误差过大 | 控制频率、带宽、增益或负载问题 | 反馈闭环检查，降低速度并停止危险动作 |
| 重力补偿越开越往下压 | 力矩符号、安装方向或重力模型错误 | 禁止实机继续，离线核对各关节贡献 |

---

## 14. 分阶段实践与验收

以下全部先使用 Docker 或宿主机上的**离线数学测试**，不连接 OpenArm 实体。

| 实验 | 实现内容 | 应记录的结果 | 通过条件 |
|---|---|---|---|
| A：URDF | 加载模型、打印所有关节/Frame | `nq, nv, nframes` | 与真实模型设计一致 |
| B：FK | 输入 `q`，输出 TCP `T` | `p、R` | 旋转正交、符合模型预期 |
| C：Jacobian | 对比解析/数值微分 | 6×7 Jacobian、误差 | 参考系一致且数值误差合理 |
| D：IK | FK → IK → FK | 位姿误差与失败率 | 可达目标收敛、不可达目标报告失败 |
| E：MoveJ | 使用 Ruckig 生成轨迹 | `q/dq/ddq/jerk` 曲线 | 轨迹满足规划约束 |
| F：MoveL | 生成 TCP 直线与连续 IK | 末端路径误差 | FK 验证路径精度达标 |
| G：TOPPRA | 对同一 `q(s)` 时间参数化 | `s(t)、q(t)` | 关节速度/加速度限制通过 |
| H：Executor Mock | 模拟轨迹执行和故障注入 | 时间抖动、状态机、错误码 | 超时能进入预定安全状态 |

### 14.1 正运动学检查公式

验证旋转矩阵：

$$
\|R^TR-I_3\|_F\approx 0,\qquad \det(R)\approx +1
$$

其中 `||·||_F` 是 Frobenius 范数，量化矩阵与合法正交性条件的偏差。

### 14.2 MoveL 路径精度检查

$$
e_p(t)=\|p_{FK}(q(t))-p_{ref}(s(t))\|_2
$$

| 参数 | 说明 | 单位 |
|---|---|---|
| `p_FK(q(t))` | 由规划关节状态通过 FK 计算出的 TCP 位置 | m |
| `p_ref(s(t))` | 目标直线路径在同一进度上的参考 TCP 位置 | m |
| `e_p(t)` | 末端位置相对参考路径的偏差 | m |

这是**模型预测轨迹误差**。真实末端绝对定位精度还需要外部测量（如运动捕捉、激光跟踪或合适的标定测量）。

### 14.3 课程配合 Codex 任务

- **P2**：URDF、SE(3)、FK、Frame、坐标变换。
- **P3**：Jacobian、DLS IK、`Jlog6`、冗余求解和 IK 回归测试。
- **P4**：MoveJ、Ruckig 参数、速度/加速度/jerk 曲线。
- **P5**：MoveL、连续 IK、TOPPRA、几何路径保持。
- **P6**：Executor、Mock Driver、超时/停止和状态机。
- **P7–P8**：独立验收驱动、关节映射与安全门槛后，才进行实机验证。

> 学习过程最重要的习惯：**每个公式理解变量 → 在小模型上测试 → 在 OpenArm URDF 上验证 → 写入可重复的自动化测试。**

---

## 15. 学习路线与参考资料

### 15.1 推荐顺序

1. **旋转与位姿**：重点弄懂 `SO(3)、SE(3)、RPY、四元数、坐标系上/下标`。
2. **正运动学**：URDF → Pinocchio Model/Data → FK → TCP。
3. **Jacobian 与 IK**：速度映射 → DLS → `log6/Jlog6` → 冗余。
4. **MoveJ**：多轴状态 → Ruckig → 速度/加速度/jerk 约束。
5. **MoveL**：TCP 直线 → 姿态插值 → 连续 IK → 时间参数化。
6. **轨迹执行**：固定周期、位置/速度/力矩模式、状态机与反馈。
7. **零重力/阻抗**：动力学、Wrench、重力项、增益、安全门槛。

### 15.2 官方资料

- Pinocchio 官方文档：<https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/>
- Pinocchio 官方逆运动学示例（讲解 `log6`、`Jlog6`、`integrate`）：<https://gepetto.github.io/doc/pinocchio/doxygen-html/md_doc_b-examples_d-inverse-kinematics.html>
- Pinocchio Frame Jacobian 参考系：<https://gepettoweb.laas.fr/doc/stack-of-tasks/pinocchio/master/doxygen-html/namespacepinocchio.html>
- Ruckig 官方教程：<https://docs.ruckig.com/tutorial.html>
- Ruckig 输入参数：<https://docs.ruckig.com/classruckig_1_1InputParameter.html>
- TOPPRA 项目：<https://github.com/hungpham2511/toppra>

**本文中的代码属于教学示例；SDK 具体 API、库版本和硬件参数应以项目锁定的实际版本与验证记录为准。**
