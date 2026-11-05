# 会话交接：从"能不能借用 PufferDrive"到"可验证的执行链"

> 日期：2026-11-04
> 基线：本次改动基于 HEAD `9d813755`（提交后见 `git log`）
> 上一会话：2026-11-03（Terra 系列全景）
> 配套阅读：`docs/2026-11-04-session-causal-chain.md`（本次全部结论的因果链与流程图，HTML 版在 `docs/html/`）

---

## 0. 30 秒恢复上下文

- **目标**：用 PufferDrive 当世界，训一个吃**矢量化输入**（agents / occ polygons / HD map / traffic lights）的 planner；**RL 是核心手段**；bad case（`bag + 时间戳` + 可下载 HD map）是 1→100 的燃料；最终要上车。
- **已定**：canonical spec 模型无关、Nova 降级为基线；occ 是物理实体；走廊锚定 + 全图漫游两个合法性域；反馈契约 A-only（只有约束）；违违规/体感 80/20。
- **已做**：车辆参数档案 + 三个测量工具（a_lat / 观测包络 / bag 读取）+ ADR-0007 + `ACTION_TYPE_CONTROL`（C 实现）+ 4 个 C 测试 + 7 个数值对拍测试。
- **卡在**：① canonical spec 未冻结（阻塞延迟缓冲/range 截断/桥接层）；② `pufferlib._C` 因本机 g++ 9.4 无 C++20 编不了（与本次改动无关，只阻塞 `puffer train`）。

---

## 1. 本次会话的裁决（含证据）

| # | 裁决 | 证据 | 产物 |
|---|---|---|---|
| D1 | **地图一源两投影**：仿真视图（几何/连通/交通控制）与编码视图（Apollo 原生语义）分开；模型侧不经过 Core | `RoadMapElement` 只有 `type/entry/exit/speed_limit`，没有 `boundary_type/turn_type/overlap` | `docs/adr/0007`、`docs/vehicle_profile.md` |
| D2 | **canonical spec 模型无关**；Nova 是基线 + 参考实现，不是契约 | owner 明示"不必拘泥 nova" | `docs/adr/0007` |
| D3 | **RL 是核心手段**，IL 降为可选 anchor | TerraZero：val14 94.19 / InterPlan 70.87，零人类演示 | `docs/2026-11-04-*` §1 |
| D4 | **bad case = 场景种子 + 约束束**（A-only） | owner："only A"；违规 80% / 体感 20% | `docs/2026-11-04-*` §2–3 |
| D5 | **occ 是物理实体**，进碰撞/offroad 热路径 | owner 裁决 | `docs/adr/0007` |
| D6 | **A/B 在执行层等价**：Nova 轨迹自带 `(acc, κ)` | `_integrate_control` 输出 `(x,y,yaw,v,acc,kappa)` | `docs/2026-11-04-*` §6.1 |
| D7 | **动作语义必须直连**：新增 `ACTION_TYPE_CONTROL` | jerk 反解会撞限幅、被 `c_throttle/c_steer` 缩放、被零穿越吸附抹平 | `pufferlib/ocean/drive/drive.h` |
| D8 | **执行保真已量化**：`model_euler` 与 Nova 积分器一致到 float32 噪声 | 60 步 / 6 s：`max |Δxy|` 1.7e-5–3.4e-5 m | `tests/unit_tests/test_control_mode_parity.py` |

---

## 2. 已完成产物

| 文件 | 内容 | 状态 |
|---|---|---|
| `docs/vehicle_profile.md` | 3799 / P04-161X 参数档案：字段语义解析（方向盘 vs 前轮、后轴 vs 包围盒中心）、派生量（δ_max / δ̇_max / κ_max / 满舵 1.43 s）、限幅映射、限值四种性质、a_lat 实测、观测包络实测、新车型接入模板 | 现行，可复用 |
| `docs/2026-11-04-badcase-as-scenario-seed-constraint-loop.md` | A-only 反馈下的 1→100 回路、20 条约束模板、两个 KPI、三个硬技术发现（积分器/执行误差/PPO 似然） | 现行 |
| `docs/adr/0007-*.md` | 七条裁决 + 被否选项 + 后果 | accepted |
| `docs/2026-11-04-session-causal-chain.md`（+ `docs/html/`） | 本次全部结论的因果链、流程图、数字表 | 本会话 |
| `data_utils/apollo_record.py` | cyber record 容器解析 + 最小 protobuf 解码（无 `cyber_py3` / 无 pb2 / 无 protoc） | 可用，自校验 message_number |
| `data_utils/measure_lateral_accel.py` | `a_lat / κ / δ / a_lon` 统计 + 物理可行性门禁 | 可用 |
| `data_utils/measure_obs_latency.py` | 感知距离、各通道数据年龄、发布延迟、planning 计算耗时 | 可用 |
| `pufferlib/ocean/drive/constants.h` | `ACTION_TYPE_CONTROL` / `INTEGRATION_SCHEME_*` / `STEER_RATE_LIMIT_RADPS` | 已实现 |
| `pufferlib/ocean/drive/drive.h` | `move_dynamics_control()` + 分流 + 两个新字段 | 已实现 |
| `pufferlib/ocean/drive/binding.c` / `env_binding.h` | 解包两个新参数 + 导出常量 | 已实现 |
| `pufferlib/ocean/drive/drive.py` / `config_schema.py` | `action_type="control"`、`integration_scheme`、`skip_actuator_limits` | 已实现 |
| `tests/drive/test_drive_dynamics.c` | +4 例（直控、忽略奖励系数、限幅、三步 Euler 参考） | 全绿 |
| `tests/unit_tests/test_control_mode_parity.py` | 7 例数值对拍（含 wash-in 两个方向的断言） | 全绿 |

---

## 3. 关键实测数字（全部本次测出）

| 量 | 值 | 来源 |
|---|---|---|
| 前轮最大转角 δ_max | 0.60564 rad（34.70°） | 8.9877 / 14.84 |
| 前轮最大角速率 δ̇_max | 0.42336 rad/s | 6.2827 / 14.84 |
| 后轴最小半径 / 曲率上限 | 3.177 m / 0.3147 1/m | `wheel_base / tan(δ_max)` |
| 直线→满舵 | **1.43 s**（仿真现为 0.92–1.11 s） | δ_max / δ̇_max |
| 后轴→包围盒中心 | 1.085 m | (3.0175 − 0.8475)/2 |
| 运营目标速度 | 12.5 m/s（AD 能力，非底盘硬限） | owner |
| 底盘设计速度 | 50–60 km/h | owner |
| 加速 / 减速硬限 | +3.0 / −6.0 m/s² | owner |
| 横向加速度代理 | **8.0 m/s²**（实测 max 7.923，596 s 样本，未冻结） | `measure_lateral_accel.py` |
| 感知最远距离 | **≈70 m**（p50 66.0，max 72.3；>100 m 为 0） | `measure_obs_latency.py` |
| 控制侧数据年龄 | **0.53 s**（bev 包 1.04 s，含录包开销） | 同上 |
| 12.5 m/s 下的盲开距离 | **6.6 m** | 0.53 × 12.5 |
| 积分器偏差（60 步） | 1.7e-5–3.4e-5 m | 对拍测试 |
| 不做 acc wash-in 的偏差 | **0.39–0.92 m / 6 s** | 对拍测试 |

---

## 4. 阻塞项（owner 决策 / 环境）

| # | 问题 | 为什么阻塞 | 建议 |
|---|---|---|---|
| B1 | canonical spec 字段清单未冻结 | 延迟缓冲、range 截断、桥接层、三源 adapter 全部依赖它 | 先出 L0 一张表（ego/agents/occ/map+topology/TL/route/provenance） |
| B2 | `pufferlib._C` 编不了 | g++ 9.4 无 `-std=c++20`，libstdc++ 缺 `<compare>`；`pufferl.py` 硬依赖该 kernel | 装 g++-11/13 或换工具链；与本次改动无关 |
| B3 | acc wash-in 归谁 | 模型 inference 会洗前 4 个加速度；不洗则 6 s 偏 0.4–0.9 m | 归**桥接层**（不改 C）；已有测试钉住 |
| B4 | 多条约束的仲裁（范围 vs 动力学） | 12.5 m/s、a_lat 8.0 都是"约束"而非"动力学夹" | 按 `vehicle_profile.md` §4.1 四类性质落位 |

---

## 5. 下一步（依赖顺序）

1. **冻结 canonical spec**（B1）——它是 L0，其他都排在它后面。
2. **桥接层 acc wash-in**（B3）——`(acc, κ)` 先洗再加 wash-in 喂仿真；对拍测试已就绪。
3. **观测包络落地**（range 70 m + delay 0.53 s）——规格在 `vehicle_profile.md` §4.3，验收三步已写。
4. **约束模板库 v0（20 条）+ 可行性裁判**——`docs/2026-11-04-*` §4。
5. **g++ 工具链**（B2）——只有它能解锁 `puffer train`。

---

## 6. 复现命令

```bash
cd ~/labs-y/30-simulation-and-generation/Self-Play/PufferDrive

# C 仿真测试（不需要 torch）
make -C tests/drive unit

# 数值对拍（需要 .venv 里的 numpy/gymnasium；binding*.so 已就位）
source .venv/bin/activate
PYTHONPATH=. python -m pytest tests/unit_tests/test_control_mode_parity.py -q

# 车辆横向包络（自带物理可行性门禁）
python data_utils/measure_lateral_accel.py \
  --record '/media/.../white-rhino-*/*.bag' --json-out lateral_envelope.json

# 观测包络
python data_utils/measure_obs_latency.py \
  --record '/media/.../rx-2714/*.bag' \
  --record '/media/.../rx-2688/*.bag' --json-out obs_envelope.json
```

环境现状：`.venv` 已用 uv + 清华镜像建好（torch 2.15.0+cu132 / numpy / gymnasium / hydra / omegaconf / pytest）。`pufferlib/ocean/drive/binding*.so` 由 gcc 直编；`pufferlib._C` 待 C++20 工具链。

---

## 7. 口径与已知偏差

- 80/20（违规/体感）来自 owner 自陈，**未用数据湖统计验证**；自动挖掘不依赖该比例成立。
- a_lat 代理 8.0 来自 596 s 行驶样本（70 条 record），**未冻结**；需在数据湖上重跑。
- bev_bag 的 1.04 s 延迟含同机录包开销（1.4 GB / 10 s），**只作上界**，基准取 simple 包的 0.53 s。
- `perception` 的 `sensor_age` 恒为 0 是 fork 约定（`header.timestamp_sec == lidar_timestamp`），真实延迟在 `publish_lag`。
- 原始数据湖存在物理不可行帧（最高 38.09 m/s、30 帧 `|κ| > κ_max`），**入口必须做可行性门禁**。
