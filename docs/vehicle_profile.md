# 车辆参数档案（Vehicle Profile）与仿真限幅标定

> 用途：PufferDrive 侧的动力学限幅、canonical spec 的 ego 几何、以及后续多车型接入的**单一真值来源**。新增车型时按 §6 复制模板，不要另起一份。
> 来源：`~/Downloads/data_xijiao-master-3799_vehicle_param.pb.txt`（Apollo `vehicle_param` protobuf text，2026-11-04 读取）。
> 关联：`docs/2026-11-04-badcase-as-scenario-seed-constraint-loop.md` §6.1/§6.2（动作语义与执行保真）、`docs/adr/0007-*`。
> 基线：HEAD `9d813755`；PufferDrive 行号按当前分支核实。

---

## 0. 一句话

`vehicle_param` 里**没有**横向加速度限值——真正的约束是**前轮最大转角**与**转角速率**（用户 2026-11-04 确认："由 max-steer-angle rate，用转角+轮胎决定"）。因此仿真侧必须以 `δ_max / δ̇_max` 为硬限，**不能**沿用 PufferDrive 的 `ACCEL_LAT_LIMIT = ±4 m/s²`（那是舒适口径，不是车辆能力）。

---

## 1. 原始参数（vehicle_id 3799 / P04-161X）

| 字段 | 值 | 语义（已解析，见 §2） |
|---|---|---|
| `front_edge_to_center` | 3.0175 m | 后轴中心 → 前缘 |
| `back_edge_to_center` | 0.8475 m | 后轴中心 → 后缘 |
| `length` / `width` / `height` | 3.865 / 1.39 / 2.10 m | 包围盒；高度 = 车顶，**顶部无传感器** |
| `width_without_lateral_sensor` | 1.3 m | 不含侧向传感器；碰撞盒用 **1.39** |
| `min_turn_radius` | 4.9 m | **外前角扫掠半径**，不是后轴半径 |
| `max_acceleration` | 3.0 m/s² | **硬限**（用户确认） |
| `max_deceleration` | −6.0 m/s² | **硬限**（用户确认） |
| `max_steer_angle` | 8.9877 rad | **方向盘**角 |
| `max_steer_angle_rate` | 6.2827 rad/s | **方向盘**角速率（= 360°/s） |
| `steer_ratio` | 14.84 | 方向盘 / 前轮 |
| `wheel_base` | 2.20 m | 轴距 |
| `wheel_rolling_radius` | 0.3003 m | 轮速 → 车速换算（本方案用位姿速度，不需要） |
| `max_abs_speed_when_stopped` | 0.2 m/s | 停稳判定阈值 |
| `vehicle_model` | P04-161X | 车型 |

运营/底盘车速由 owner 补充（不在本文件内）。**三者不可混用**：

| 项 | 值 | 性质 | 仿真落点 | 来源 |
|---|---|---|---|---|
| **运营目标速度** | **12.5 m/s（45 km/h）** | **AD 系统能力上限**（系统延迟 + 感知距离），不是动力学硬限 | **约束 + 指标 + 场景生成边界**，不做动力学夹 | owner 2026-11-04 |
| **底盘设计速度** | **~13.9–16.7 m/s（50–60 km/h）** | 底盘/动力学能力上限（友商已跑到 50+ km/h） | 动力学速度夹（建议 16.7） | owner 2026-11-04 |
| 倒车限速 | **−2.0 m/s** | 硬限 | 动力学夹 | owner 2026-11-04 |

> `12.5 m/s` 不是车跑不快，而是**当前 AD 软件只能在这个速度以下安全运行**（延迟、感知距离）。把它写成动力学夹会同时犯两个错：训练时物理上限制了车做不到的事，且策略永远学不到“超速后怎么收”——而约束式写法两个问题都没有。

> `vehicle_id 3799` 不是单车，是**一类主流车型**的标识（owner 确认）。车队多车型 ⇒ canonical spec 必须带 `vehicle_param_id`，见 §6。

---

## 2. 语义解析：为什么这些数这么读

三个字段的"中心"和"角"容易读反。下面这条推导同时确定三件事：

```
δ_max = max_steer_angle / steer_ratio = 8.9877 / 14.84 = 0.60564 rad (34.70°)
R_axle = wheel_base / tan(δ_max)     = 2.20 / 0.69242    = 3.177 m
外前角半径 = √((R_axle + width/2)² + front_edge_to_center²)
           = √((3.177+0.695)² + 3.0175²) = 4.909 m  ≈  min_turn_radius = 4.9 ✓
```

由此同时确定：

1. `max_steer_angle` / `max_steer_angle_rate` 是**方向盘**量（不是前轮）；
2. 参数里的 "center" 是**后轴中心**（`front_edge_to_center + back_edge_to_center == length` 也对得上）；
3. `min_turn_radius` 是**外前角**半径，不是后轴半径。

⚠️ **不要把 `1/min_turn_radius = 0.204` 当曲率上限**。后轴曲率上限是 `0.3147 1/m`（R = 3.177 m）。用错会把转弯能力低估 35%。

---

## 3. 派生量（仿真与模型契约真正需要的）

| 量 | 值 | 用途 |
|---|---|---|
| 前轮最大转角 `δ_max` | **0.60564 rad** | 替换 `STEERING_LIMIT`（classic）与 jerk 分支的 `0.55` |
| 前轮最大角速率 `δ̇_max` | **0.42336 rad/s** | 替换 `0.6 rad/s` |
| 后轴最小半径 `R_min` | 3.177 m | 曲率上限 |
| 曲率上限 `κ_max` | **0.3147 1/m** | 模型侧可行性检查 |
| 后轴 → 包围盒中心偏移 `d` | **1.085 m** | `d = (front_edge_to_center − back_edge_to_center) / 2` |
| 直线 → 满舵时间 | **1.43 s** | `δ_max / δ̇_max`；左右满舵 2.86 s |
| 包围盒 | 3.865 × **1.39** × 2.10 m | 碰撞/观测；宽度含侧向传感器 |
| 后轴到前缘 / 后缘 | 3.0175 / 0.8475 m | 加塞、停止线、跟车时距判定 |

`1.43 s 到满舵` 这个数很重要：它决定一次紧急避让**最早什么时候能开始转**。模型如果在 1 s 内要求满舵，实车做不到，而现在的仿真允许（见 §4 的 0.92 s / 1.11 s）。

---

## 4. 映射到 PufferDrive

| PufferDrive | 现值 | 本车型 | 处理 |
|---|---|---|---|
| `STEERING_LIMIT`（`constants.h:125`，classic） | 0.667 rad | 0.60564 | 改为可配置，值 0.60564 |
| jerk 分支 `\|new_steering_angle\| ≤ 0.55`（`drive.h:4387`） | 0.55 rad | 0.60564 | 同上（两处必须同源） |
| 转角速率（`drive.h:4241`、`drive.h:4384` 两处 `0.6f * dt`） | 0.6 rad/s | 0.42336 | 改为可配置 |
| `ACCEL_LONG_LIMIT`（`constants.h:127`） | {−5.0, 2.5} | **{−6.0, 3.0}** | 改；注意 `drive.h:4335` 还乘了 `c_acc`，control 模式须置 1 |
| `ACCEL_LAT_LIMIT`（`constants.h:128`） | {−4.0, 4.0} | **不适用**；实测代理 ≈ 8.0（见 §5） | control 模式下不夹；见 §5 |
| `wheelbase`（`drive.h:2487/2655/2907`：`0.6 × sim_length`） | 2.319（对 3.865 m） | **2.20** | ego 用真值；背景车可保留启发式 |
| `MAX_BACKWARD_SPEED`（`constants.h:124`） | −2.0 | **−2.0** | 一致，不动 |
| `base_max_speed_mps`（`config/puffer_drive.yaml:56`） | 20.0 | **底盘设计上限 16.7**（不是 12.5） | 改；12.5 另立为约束 |
| 超速约束阈值 | 无 | **12.5 m/s**（AD 系统能力） | 新增：约束集 + 指标，**不进动力学**（见 §4.1） |
| `AGENT_STOPPED_SPEED_THRESHOLD`（`constants.h:297`） | 0.2 | 0.2 | 一致，不动 |
| `COMFORT_ACCEL_THRESHOLD`（`drive.h:3512`，函数内 `const float`） | 3.0 m/s² | 3.0 = 硬加速限 | 一致；但按 AGENTS.md 应上提到 `constants.h`（此处只标记，不修改） |
| `COMFORT_JERK_THRESHOLD`（`drive.h:3513`） | 5.0 m/s³ | 无车辆参数 | 待定（约束口径，见 ADR-0007 §5） |

三个已知的不一致（**标记，不顺手改**）：

1. classic 用 `0.667`、jerk 用 `0.55`——同一辆车两个物理上限，且都不等于 0.6056；
2. `wheelbase = 0.6 × sim_length` 对本车型偏大 5.4%，而 wheelbase 是 `κ → δ → δ̇` 链条的分母；
3. `ACCEL_LONG_LIMIT[1]` 在 jerk 分支被 `c_acc` 乘，`c_throttle/c_steer` 也参与加速度积分（`drive.h:4331/4341`）——奖励条件化会让**同一动作在不同 agent 上执行出不同轨迹**，control 模式必须跳过。

### 4.1 限值有四种性质，仿真落点不同

这张表是本文件最重要的约束：每一种限值都要标“性质”，否则就会把“系统允许”当成“物理能”，或反之。

| 性质 | 本车型的例 | 仿真落点 |
|---|---|---|
| **执行器硬限** | δ_max 0.6056、δ̇_max 0.4234、accel +3.0 / −6.0 | 动力学夹（不夹就是假车） |
| **底盘/结构限** | 底盘设计速度 50–60 km/h | 动力学夹（速度上限） |
| **AD 系统能力上限** | 运营 12.5 m/s（延迟 + 感知距离） | **约束/指标/场景边界**；不夹动力学 |
| **舒适/合规** | jerk 5.0、comfort accel 3.0 | 指标 + 软奖励 / 动作空间构造（Comfort by Construction） |

推论：**12.5 m/s 的真正约束不在动力学层，而在观测层**（感知距离 + 延迟）。仿真如果不建模这两项，策略就会用比实车更新、更远的信息做决策——在 45 km/h 附近表现得过于激进，而你在仿真里看不到。

### 4.2 观测包络（实测，2026-11-04）

工具：`data_utils/measure_obs_latency.py`。两个包：`rx-2714`（simple，不含传感器，20 s）与 `rx-2688`（bev_bag，含传感器，1.4 GB / 10 s）。

| 量 | rx-2714 simple | rx-2688 bev_bag |
|---|---|---|
| 感知距离：每条消息的最大障碍物距离 p50 / p99 / max | **66.0 / 69.5 / 72.3 m** | **66.8 / 78.3 / 78.8 m** |
| 距离 > 100 m 的障碍物 | **0** | **0** |
| perception 发布延迟（record − header）p50 / p99 | 0.126 / 0.172 s | 0.371 / 0.660 s |
| planning 数据年龄（header − lidar_timestamp）p50 / p99 | **0.459 / 0.689 s** | 0.930 / 1.194 s |
| control 数据年龄 p50 / p99 | **0.526 / 0.761 s** | 1.038 / 1.482 s |
| planning 计算耗时（`latency_stats.total`）p50 / p99 | 72 / 129 ms | 168 / 322 ms |
| planning 自报 perception_latency p50 | 150 ms | 475 ms |
| 12.5 m/s 下的"盲开距离"（control 口径） | **6.6 m** | 13.0 m |

**结论（进入仿真参数）：**

1. **`agent_range_m ≈ 70 m`**（不是 200）。分布高度集中在 66–72 m（200 帧里每帧最大值都落在 66–72），更像**配置门限**而非环境偶然（推断，待确认配置）。
2. **`observation_delay_s ≈ 0.53 s`**（以 simple 包为准）。bev_bag 的 1.04 s **不能当基准**——它同时录 1.4 GB/10 s 的传感器，录包开销会放大延迟；只能当上界。
3. **地图窗口与感知距离是两件事**：地图取数半径 200 m（HD map，不看传感器），障碍物只能看到 ~70 m。canonical spec 必须写成两个字段（`map_range_m=200` / `agent_range_m≈70`）。混用会让模型看见实车看不见的 agent——这是最隐蔽的一种 train-serve skew。
4. 仿真侧现在：`obs_range_partner_m: 200` + 零延迟。两项都要改，且必须**同时**改——只改一项会得到一个更难解释的分布。
5. `perception` 的 `sensor_age` 恒为 0：这个 fork 把 `header.timestamp_sec` 设成了 `lidar_timestamp`，感知链路的真实延迟全在 `publish_lag`（126–371 ms）里。读 bag 时别把 0 当成"无延迟"。
6. 静态物（occ 流）**不稀疏**：这两个包里 fence 5263/1333、cone 820/3947、barriergate 21/78。“真实数据 occ 极稀疏”（来自 rinova adapter 注释）在这两个包上不成立——按包统计，不要按印象写在 spec 里。

### 4.3 两个参数的语义（桥接层实现规格，未落代码）

```
agent_range_m      = 70.0      # 障碍物候选集截断半径（欧氏距离，ego 中心为原点）
map_range_m      = 200.0      # 地图元素取数半径（HD map，与感知无关）
observation_delay_s = 0.53    # 决策时刻能看到的最新世界状态年龄
```

实现位置：**语义外挂/特征装配层**，不改 C 内核。

- **`agent_range_m`**：在组装 agent 槽位前按 `‖p_agent − p_ego‖ ≤ agent_range_m` 过滤，再按距离升序填槽。不得用地图窗口代偿。
- **`observation_delay_s`**：环形缓冲持有最近 `ceil(delay/dt)+1` 帧场景快照；决策步 `t` 的输入取 `t − round(delay/dt)` 帧（否则取最近邻并在 `BuildStats` 上报偏差）。缓冲未满时**不得**用零帧垫——记一次“warm-up 跳过”，与“无链占位帧”同口径。
- **两者必须同时开**：只缩范围→“看得近但反应快”；只加延迟→“看得远但慢半拍”，两者都不是车上那个系统。

验收（按顺序，全部可自动化）：

1. `delay=0, range=∞` 时，新链路与现有实现逐张量一致（回归护栏）。
2. `delay=ceil(0.53/0.1)=5 帧` 时，第 `t` 步的输入张量等于 `t−5` 帧的输入张量（用固定种子回放断言）。
3. E1a（背景 replay、无延迟）vs E1a+delay+range：指标变化量单独记账——这就是“观测包络值多少分”的第一个数据点。

---

## 5. 横向量：唯一一个没有车辆参数的物理量

**事实**：`vehicle_param` 无横向加速度字段；用户口径是"由转角 + 轮胎决定"。转角/角速率约束的是**控制量**，不是**轮胎抓地**。PufferDrive 没有摩擦圆模型，`a_lat` 完全由 `κ·v²` 产生。

现 `ACCEL_LAT_LIMIT = ±4 m/s²` 的实际后果（`drive.h:4345` 硬夹）：

| 车速 | 被夹后的最小半径 |
|---|---|
| 12.5 m/s（本车最高） | 39.1 m |
| 8 m/s | 16 m |
| 5 m/s | 6.25 m |
| < 3.57 m/s | 夹子不生效（受 δ_max 约束） |

也就是说，**现在的仿真在 45 km/h 下不允许车转弯半径小于 39 m**，而真车受转角约束可以到 3.18 m（受轮胎约束大致 20–25 m）。这是一阶保真损失，并且方向固定——RL 会学进"高速不能躲"。

处理口径：

- **control 模式（执行模型）**：撤掉 `a_lat` 硬夹，只保留 `δ_max` 与 `δ̇_max`。这样 sim 的可行集与车辆描述一致。
- **RL 训练**：撤掉夹子后，12.5 m/s 满舵 = 49 m/s²，物理不可能，策略会用它躲避碰撞——必须补一个**轮胎代理上限**。
- 该上限同时应进**约束集**（ADR-0007 §5），而不只进动力学夹——否则它和舒适约束会互相打架。

### 5.1 实测（2026-11-04）

工具：`data_utils/measure_lateral_accel.py`（`a_lat = v · ω_z`，`κ = ω_z / v`，`δ = atan(κ · wheelbase)`；对定位跳变做物理可行性门禁并计数）。

```bash
python data_utils/measure_lateral_accel.py \
    --record '/media/.../white-rhino-*/*.bag' --json-out lateral_envelope.json
```

样本：70 条 record（`/media/.../white-rhino-*`，2024-05 ~ 2024-12），74,526 帧，总时长 1,043 s，**行驶 596 s**。

| 量 | p50 | p90 | p99 | p99.9 | p99.99 | max |
|---|---|---|---|---|---|---|
| speed (m/s) | 1.698 | 7.699 | 9.691 | 12.930 | 14.740 | 14.993 |
| \|a_lat\| (v≥1) (m/s²) | 0.048 | 0.314 | 0.792 | 2.356 | 5.370 | **7.923** |
| \|κ\| (v≥2) (1/m) | 0.002 | 0.022 | 0.088 | 0.176 | 0.309 | **0.315** |
| \|δ\| (v≥2) (rad) | 0.005 | 0.048 | 0.192 | 0.370 | 0.598 | **0.606** |
| δ/δ_max | 0.007 | 0.079 | 0.317 | 0.610 | 0.987 | 1.000 |

| 计数 | 值 |
|---|---|
| \|a_lat\| > 4 / 6 / 8 m/s² | 8 / 4 / 0 |
| δ ≥ 0.9·δ_max | 14 / 44,690 |
| 物理不可行：speed > 15 m/s | 90 |
| 物理不可行：\|κ\| > κ_max（v≥2） | 30 |

**结论**：

- **暂定上限 = 8.0 m/s²**（取实测 max 7.923 向上取整到 0.5；硬限不能低于车辆已经做到的事）。p99.99 = 5.37 作为稳健估计一并保留。
- **596 s 远不够**（脚本阈值 1800 s）：这 8.0 是“不阻碍已知能力”的地板值，不是标定值。需在数据湖上重跑后冻结。
- 转向包络基本没用到：δ/δ_max 的 p99.9 只有 0.61，只有 14 帧触及 90% 满舵。这意味着**当前数据几乎不含极限转向场景**，也解释了为什么 `a_lat` 分布那么轻尾。
- 数据质量：原始湖里存在**物理不可行帧**（最高 38.09 m/s = 137 km/h、30 帧 κ > κ_max），几乎确定是 RTK 重定位跳变。**场景抽取器必须在入口做物理可行性门禁**，否则这些帧会污染训练集与“超速/越界”违规挖掘。
- 纵向加速度：定位的 `linear_acceleration` 噪声大（p100 48 m/s²），`dv/dt` 更差；30% 帧越过 3 m/s²、22% 帧越过 −6 m/s²，**不能据此推翻“硬限”**——需要用 `/apollo/canbus/chassis` 或 `/apollo/control` 做第二源交叉验证。
- 观察到的速度 p99.9 = 12.93、max = 14.99 m/s，**高于运营目标 12.5 m/s**（已剔除 38 m/s 的跳变）——这与“12.5 是 AD 系统能力而非底盘硬限”一致：车能跑更快，只是当前软件不允许。

---

## 6. 新车型接入模板

新增一个 `vehicle_param_id` 时，按顺序做四步，**不要跳过推导**：

1. **读三个比值**：`δ_max = max_steer_angle / steer_ratio`，`δ̇_max = max_steer_angle_rate / steer_ratio`，`R_axle = wheel_base / tan(δ_max)`。
2. **验一致性**：`√((R_axle + width/2)² + front_edge_to_center²)` 是否 ≈ `min_turn_radius`。差得远说明字段语义或单位有问题，停。
3. **算偏移**：`d = (front_edge_to_center − back_edge_to_center) / 2`（后轴 → 包围盒中心）。
4. **填表**（本文件 §3 的格式）并登记 provenance：原始文件路径、读取日期、`vehicle_id`、`vehicle_model`。

| 字段 | 3799 / P04-161X | 车型 B | 车型 C |
|---|---|---|---|
| `vehicle_param_id` | 3799 | | |
| `vehicle_model` | P04-161X | | |
| L × W × H | 3.865 × 1.39 × 2.10 | | |
| 后轴→前/后缘 | 3.0175 / 0.8475 | | |
| `wheel_base` | 2.20 | | |
| `steer_ratio` | 14.84 | | |
| `δ_max` / `δ̇_max` | 0.60564 rad / 0.42336 rad/s | | |
| `κ_max` | 0.3147 1/m | | |
| accel / decel 硬限 | 3.0 / −6.0 m/s² | | |
| 运营目标速度（AD 能力） | 12.5 m/s | | |
| 底盘设计速度 | 13.9–16.7 m/s（50–60 km/h） | | |
| 倒车限速 | −2.0 m/s | | |
| `a_lat` 轮胎代理 | 8.0 m/s²（暂定，见 §5.1） | | |

canonical spec 必须带 `vehicle_param_id`；scene 里每个 agent 的几何与限幅都由它派生，否则换车型就是重训。

---

## 7. 坑（写死，别再踩）

1. `1/min_turn_radius` **不是**曲率上限（差 35%）。
2. `max_steer_angle` / `max_steer_angle_rate` 是**方向盘**量；用了 `steer_ratio` 才是前轮。
3. `wheelbase = 0.6 × length` 只是启发式；对 3.865 m 车偏大 5.4%。
4. 车辆参数里的 "center" 是**后轴**；PufferDrive 的 `sim_x/sim_y` 是**包围盒中心**（`datatypes.h:55`）。两者差 `d`（本车 1.085 m），必须先转换再写 bin。
5. `ACCEL_LAT_LIMIT` 不是车辆参数，是舒适口径；拿它当物理限会把高速转弯砍掉一半以上。
6. `c_throttle / c_steer / c_acc` 是奖励条件化系数，参与加速度积分；control 模式必须置 1。
7. `ACCEL_LONG_LIMIT[1]`、`ACCEL_LAT_LIMIT[1]` 还兼作观测归一化分母（`drive.h:3765-3766`）；改限幅会改观测尺度——若模型侧走 canonical spec，这一点无影响，但别在 PufferDrive 原生 obs 上做对照实验时忘了。
8. **运营目标 ≠ 动力学硬限**：12.5 m/s 是 AD 系统能力（延迟 + 感知距离），写成速度夹会同时错两处——限制车做不到的事，且让策略学不到超速后的收敛。
9. **三层速度不可混用**：运营 12.5 / 底盘 50–60 km/h / 实测尾部 12.93–14.99。混用会让“超速违规挖掘”与“动力学可行域”同时失真。

---

## 8. 口径声明

- §1 全部字段与 §3 全部派生量均来自上述 pb.txt 与 §2 的解析式，**非估计**；一致性检验（4.909 vs 4.9）是独立的交叉验证。
- 最高车速：**运营目标 12.5 m/s（AD 系统能力：延迟 + 感知距离）**，**底盘设计 50–60 km/h（13.9–16.7 m/s）**，倒车 **−2.0 m/s（硬限）**；`max_acceleration/deceleration` 为硬限；宽度用 1.39；顶部无传感器；3799 为车型——均来自 owner 2026-11-04 确认，不在 pb.txt 内。
- `a_lat` 轮胎代理：实测值见 §5.1（暂定 **8.0 m/s²**，样本仅 596 s 行驶，**未冻结**）。
- 实测样本来自 `/media/.../white-rhino-*`（70 条 record，2024-05 ~ 2024-12），**不是全量数据湖**；p99.9 之后的尾部随样本量会变。
- 本文不改代码；§4 的"处理"列是待执行的标定项。

---

## 9. 工具

| 工具 | 用途 |
|---|---|
| `data_utils/apollo_record.py` | cyber record 容器解析 + 最小 protobuf 解码（不需要 `cyber_py3`、不需要 `*_pb2.py`）。bag → 场景管线的入口 |
| `data_utils/measure_lateral_accel.py` | 从 `/apollo/localization/pose` 统计 `a_lat / κ / δ / a_lon`，含物理可行性门禁与计数；`--list-channels` 可查任意 record 的通道 |
| `data_utils/measure_obs_latency.py` | 统计感知距离、各通道数据年龄（`header − lidar_timestamp`）、发布延迟、planning 计算耗时；输出给仿真的 `agent_range_m` 与 `observation_delay_s` |

两个脚本都是“只读不改”；容器解析对 message 总数与文件头 `message_number` 做一致性校验，不一致直接抛错。
