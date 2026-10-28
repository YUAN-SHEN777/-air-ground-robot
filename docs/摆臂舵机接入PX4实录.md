# 摆臂舵机接入 PX4 实录

> 副标题：从"纹丝不动"到"52.00° 精确到位"
> 时间：2026-10-12 ~ 10-13
> 前置文档：`PX4移植规格.md`（调研）、`PX4移植实录.md`（主体移植：STALE / 翻覆 / 悬停）

---

## 目录

- [第 0 章 · 目标与最终结果](#第-0-章--目标与最终结果)
- [第 1 章 · 先把通路搞清楚：PX4 是怎么控制舵机的](#第-1-章--先把通路搞清楚px4-是怎么控制舵机的)
- [第 2 章 · 坑一：改了代码却看不到效果](#第-2-章--坑一改了代码却看不到效果)
- [第 3 章 · 坑二：ARM 回了 ACCEPTED，飞机却没解锁](#第-3-章--坑二arm-回了-accepted飞机却没解锁)
- [第 4 章 · 坑三：摆臂纹丝不动（最深的坑）](#第-4-章--坑三摆臂纹丝不动最深的坑)
- [第 5 章 · 概念辨析：仿真步长 vs 真机速率](#第-5-章--概念辨析仿真步长-vs-真机速率)
- [第 6 章 · 可复用的排查手段](#第-6-章--可复用的排查手段)
- [第 7 章 · 最终成果与改动清单](#第-7-章--最终成果与改动清单)
- [第 8 章 · 经验提炼](#第-8-章--经验提炼)
- [第 9 章 · 人在回路：怎么舒服地控制摆臂](#第-9-章--人在回路怎么舒服地控制摆臂)
- [附录 · 命令速查](#附录--命令速查)
- [附：还没做完的事](#附还没做完的事)

---

## 第 0 章 · 目标与最终结果

### 目标

让 PX4 能控制这台空-地两栖机器人的**两个摆臂舵机**（`servo_0` / `servo_1`），
并且**两个臂必须严格同步**。

### 最终结果

| DO_SET_ACTUATOR 值 | PX4 指令角 | servo_0 实测 | servo_1 实测 | 两臂差 |
|---|---|---|---|---|
| −1.0 | 0° | −0.00° | −0.00° | 0.000° |
| 0.0 | 26° | +24.60° | +24.63° | 0.030° |
| +1.0 | 52° | **+52.00°** | **+52.00°** | **0.000°** |

- ✅ **用的是 URDF 里定义的真实舵机规格 0.7 N·m，没有放宽任何参数**
- ✅ 未解锁时自动停在 0°（飞行安全位）
- ✅ 命令通路全链路打通，两臂物理级同步
- ✅ 全程 STALE 0 次，RTF ≈ 0.996

（26° 那 1.4° 误差是负载造成的静态误差，来源是控制器 `i_max=0.3` 的积分限幅；
52° 因为顶到机械限位所以精确。）

---

## 第 1 章 · 先把通路搞清楚：PX4 是怎么控制舵机的

### 1.1 全链路

```
MAVLink  DO_SET_ACTUATOR(187)
   ↓
vehicle_command (uORB 话题)
   ↓
FunctionActuatorSet._data[0]        ← 功能号 301 对应的函数提供者
   ↓
MixingOutput::output_limit_calc_single()   ← 混控 / 限幅
   ↓
GZMixingInterfaceServo  →  gz topic /model/airground_0/servo_0  （单位：弧度）
   ↓
两个 JointPositionController（共用同一话题 → 物理级同步）
   ↓
servo_0 / servo_1 关节角度
```

### 1.2 三层映射（最容易搞错的地方）

| 层 | 在哪 | 干什么 |
|---|---|---|
| **第 1 层** | `mixer_module` | 把 MAVLink 的 `[-1, 1]` 映射到 `[0, 1000]` |
| **第 2 层** | `GZMixingInterfaceServo` | 把 `[0, 1000]` 映射成关节角度 |
| **第 3 层** | `JointPositionController`（Gazebo） | 实际出力把关节转到目标角 |

**第 1 层的源码**（`mixer_module.cpp`）：

```cpp
float MixingOutput::output_limit_calc_single(int i, float value) const
{
    if (!PX4_ISFINITE(value)) return _disarmed_value[i];   // ← 没收到命令 → 用 DIS 值
    ...
    // 301 = Peripheral_via_Actuator_Set，不属于 Servo1..ServoMax → 走这条
    output = math::interpolate(value, -1.f, 1.f, min, max);
}
```

**第 2 层的源码**（`GZMixingInterfaceServo.cpp`）：

```cpp
double output_range = _mixing_output.maxValue(i) - _mixing_output.minValue(i);
double output = _angle_min_rad[i]
              + _angular_range_rad[i] * (outputs[i] - _mixing_output.minValue(i)) / output_range;
```

**定案取值**（写进 airframe）：

```
SIM_GZ_SV_FUNC1 = 301      功能号（Peripheral_via_Actuator_Set 1）
SIM_GZ_SV_FUNC2 = 0        只分配一路！两个臂共用话题，物理级同步
SIM_GZ_SV_MIN1  = 0        第 1 层输入下限
SIM_GZ_SV_MAX1  = 1000     第 1 层输入上限
SIM_GZ_SV_MINA1 = 0        角度下限（度）
SIM_GZ_SV_MAXA1 = 52       角度上限（度）
SIM_GZ_SV_DIS1  = 0        未解锁 → 0°（飞行安全位）
SIM_GZ_SV_FAIL1 = 0        失效保护 → 0°
```

对应关系：

```
param1 = -1  →  0°        param1 = 0  →  26°        param1 = +1  →  52°
未收到命令 / 未解锁  →  DIS1 = 0  →  0°
```

### 1.3 陷阱：`SIM_GZ_SV_MIN/MAX/DIS/FAIL` 的值域是 `[0, 1000]`

**不是 `[-1, 1]`！** 源码 `gz_bridge/module.yaml` 的 `standard_params`：

```yaml
- param_prefix: SIM_GZ_SV
  standard_params:
    disarmed: { min: 0, max: 1000, default: 500 }
    min:      { min: 0, max: 1000, default: 0 }
    max:      { min: 0, max: 1000, default: 1000 }
    failsafe: { min: 0, max: 1000 }
```

而 `mixer_module.cpp` 的 `updateParams()` 是用 **`param_get(..., &int32_t)`** 读的：

```cpp
if (_param_handles[i].disarmed != PARAM_INVALID && param_get(_param_handles[i].disarmed, &val) == 0) {
    _disarmed_value[i] = val;
}
```

**⇒ 写成 `-1` 是越界值，会被拒绝**，然后回落成默认值（`disarmed` 默认 500）
—— 这正是"问题 A：未解锁时不是 0° 而是 52°"的根因。

---

## 第 2 章 · 坑一：改了代码却看不到效果

### 症状

airframe 里明明写着 `SIM_GZ_SV_DIS1 = 0`，跑起来却还是上次调试时改的值。
**"代码改了，效果没变"。**

### 根因：`param set-default` 的语义 + 参数持久化

```
param set-default  =「只在参数【还没有值】时才写入」
PX4 SITL 会把参数持久化到 build/px4_sitl_default/rootfs/parameters.bson
   ↓
只要 bson 里已经有这个参数（比如某次调试时运行时改过），
airframe 里的 set-default 就【永远不再生效】
```

### 修法：按参数用途分两类

| 参数类别 | 用什么 | 理由 |
|---|---|---|
| **机体接口类**<br>`SIM_GZ_*` / `CA_ROTOR*` / `CA_AIRFRAME` / `SENS_EN_MAGSIM` | **`param set`（强制覆盖）** | 错了模型**静默失效**，必须保证每次启动都对 |
| **性能调参类**<br>`MPC_THR_HOVER` | `param set-default` | 保留用户在 QGC 覆盖微调的能力 |

改后 airframe 共 **28 行 `param set`**，只剩 `MPC_THR_HOVER` 是 `set-default`。

### 定下的规矩

1. **诊断时绝不要随手 `param set`** —— 会被持久化并污染后续所有实验
2. 做 A/B 实验后**必须显式恢复**，或者做完直接删 `parameters.bson`
3. **凡是"错了就静默失效"的接口参数，一律用 `param set`**
4. 排查手法：`strings parameters.bson | grep -i '<前缀>'` 看有没有非预期的持久化项
   （出厂基线 **312 字节**，膨胀了就说明有污染）

---

## 第 3 章 · 坑二：ARM 回了 ACCEPTED，飞机却没解锁

### 症状

脚本发解锁命令，PX4 回 `ACCEPTED`，电机指令也确实变成非零 ——
**看起来一切正常，但舵机就是不动。**

### 为什么极难发现

因为 `ACCEPTED` 只是"命令被受理"的**初回执**，不代表真的解锁成功。
必须读 **HEARTBEAT 的 `base_mode & MAV_MODE_FLAG_SAFETY_ARMED`** 才算数。

```
ARM 命令回执     : ACCEPTED
【真实解锁位】   : False   ❌ 其实没解锁！
```

**没解锁的后果极其隐蔽**：`gz_bridge` 里 `_armed.armed = false`
→ 所有输出走 `outputs[i] = NAN` → 舵机永远停在 DIS 值
→ **看起来像"机构卡死"**，会把人引向完全错误的方向（去查机械/力矩）。

### 让解锁真正成功的三个条件

#### ① 持续发 `MANUAL_CONTROL`（模拟 QGC 虚拟摇杆）

根因在 commander 的 `modeCheck`：

```cpp
if (manual_control_signal_lost && mode_req_manual_control != 0) {
    const bool rc_disabled = (_param_com_rc_in_mode.get() == 4);
    NavModes nav_modes = rc_disabled ? mode_req_manual_control : NavModes::None;
    ...
    reporter.clearArmingBits(...);      // ← 无条件执行！
    reporter.clearCanRunBits(...);
}
```

**`clearArmingBits` 是无条件调用的** —— 即使只报警告，它仍然把"需要摇杆的模式"
移出可解锁集合。开机默认是 **Position 模式（需要摇杆）**，而仿真里既没遥控器也没摇杆
→ **解锁被拒**。

#### ② airframe 里加 `param set COM_RC_IN_MODE 1`

`1` = "MAVLink only，禁用 RC 及相关解锁前检查"。
QGC 的虚拟摇杆走的就是 `MANUAL_CONTROL`（MAVLink），所以这个值不冲突。

#### ③ 关掉自动上锁

```
COM_DISARM_LAND  = 2.0 s    落地后自动上锁
COM_DISARM_PRFLT = 10.0 s   解锁后不起飞自动上锁   ← 主犯
```

PX4 日志原话：`INFO [commander] Disarmed by auto preflight disarming`

**这是测试专用设置**，故意放在脚本里而不写进 airframe（真实飞行需要自动上锁）。
脚本每次启动都会删 `parameters.bson`，所以运行时改的参数不会污染下一次运行。

### 附带发现 A：`param_set_send` 的【类型】必须和 PX4 里一致

否则 PX4 报 `ERROR [mavlink] param types mismatch` 并**静默丢弃**：

| 参数 | 类型 |
|---|---|
| `COM_DISARM_LAND` / `COM_DISARM_PRFLT` | **FLOAT** → `MAV_PARAM_TYPE_REAL32` |
| `SIM_GZ_EC_*` / `SIM_GZ_SV_*` | **INT32** → `MAV_PARAM_TYPE_INT32` |

**这个错误只写在日志里，不看日志根本发现不了。**
`COM_DISARM_*` 被静默丢弃 → 自动上锁没关掉 → 解锁后几秒掉锁 → 一切看似无解。

### 附带发现 B：脚本必须【周期性】发 GCS 心跳

`mavutil` 不会自动发心跳。不发的话 PX4 几秒后报
`INFO [commander] Connection to ground station lost`。

### 附带发现 C：`COM_DISARM_LAND` 会打断"串行位置控制"

现实中舵机是**串行总线**（如 LX-16A / STS 系列），位置命令必须一条条排队。
所以在真机上"降落 → 停桨 → 折臂 → 走轮"这个时序不能被自动上锁打断。

---

## 第 4 章 · 坑三：摆臂纹丝不动（最深的坑）

### 4.1 症状

解锁确认成功、命令确认送达（探针显示 PX4 确实在往 `/model/airground_0/servo_0`
发 `+52.00°`）、两臂也确实同步 —— 但**关节角纹丝不动（0.00°）**。

而**不接 PX4** 的独立仿真里，同样的 `robot.urdf`、同样的 0.7 N·m，
**3 秒内折到 +52.00°，零稳态误差**。

### 4.2 排查走过的弯路（逐一排除清单）

| # | 假设 | 怎么排除的 |
|---|---|---|
| 1 | 舵机力矩不足 | 3.0 N·m 能折到 51° → 但 1 ms 下 **0.7 N·m 也完全正常** |
| 2 | 机械死点 | 1 ms 下 0→52° 全行程平滑 |
| 3 | 地面摩擦 | 把 PX4 世界地面改成 0.4（与独立模式一致）后仍不行 |
| 4 | 质量差异 | 两边总质量逐 link 比对，完全一致（0.679861 kg） |
| 5 | 刷进地面 | 两边 base/link9/link10/link1/link5 的 z **逐位相同** |
| 6 | 物理引擎 | 换 dartsim 后仍不行 |
| 7 | 自碰撞 | 模型级 `<self_collide>false</self_collide>` 加上仍不行 |
| 8 | PID 增益 | 纯 P 控制（i=0, d=0）在 4 ms 下**同样失败** |
| 9 | 关节 `<dynamics><damping>` | 置 0 无效 |
| 10 | 关节 `<effort>` 的实现方式 | 改用 `cmd_max=0.7` 走另一条路径，**同样失败** |
| 11 | ODE 接触 `<min_depth>` | 0.0005/0.001/0.002 **全部无效** |
| 12 | 控制频率与步长不匹配 | **零负载时 4 ms 完全正常** → 排除 |

### 4.3 ⚠️ 我自己踩的两个实验陷阱（都会导致错误结论）

#### 陷阱一：用"把 spawn 高度调高"来模拟零负载 —— **无效！**

从 z=2.0 自由落体只需 **0.64 秒**，而脚本要等 12 秒才开始发指令
→ **机器人早就落地了**，测的还是"有负载"。

**正确做法：把重力设为 0，让机器人永久悬浮。**

```bash
GRAVITY="0 0 0" python3 build_px4_model.py
```

> 我因此得出过"与负载无关"的错误结论，后来推翻。**这是一个会让整个推理链跑偏的坑。**

#### 陷阱二：脚本里的话题名硬编码要跟着世界名走

世界从 `default` 改名 `airground` 后，IMU 话题路径变成
`/world/airground/...`，而 `px4_check.py` 没跟着改
→ **静默读到 0 条 → RTF 误报成 0.000**。

### 4.4 真正的根因：物理步长 × 地面接触

```
零重力（真正零负载，无任何接触）：
    step=4ms  0.7N·m  →  52.00°  ✅
    step=1ms  0.7N·m  →  52.00°  ✅      ← 4 ms 本身完全没问题！

正常重力（摆臂要抬机身）：
    step=4ms  0.7N·m  →   0.00°  ❌
    step=1ms  0.7N·m  →  52.00°  ✅
```

**机理**：

1. 粗步长下**地面接触的穿透更深**
2. 引擎为消解穿透施加的**恢复力更大**
3. → 摆臂要克服的**等效阻力被抬高**
4. → 0.7 N·m 的舵机刚好推不动

**这是纯粹的数值伪影，与真机无关。**

### 4.5 力矩阈值扫描（量化）

| 步长 | 从 0° 折到 52° 所需力矩 |
|---|---|
| **1 ms** | 0.40 ❌ → **0.50 ✅** 　⇒ 真实所需 **≈ 0.48 N·m** |
| **4 ms** | 0.70 ❌ → **0.85 ✅** 　⇒ 等效所需 ≈ 0.80 N·m |

**⇒ 步长把等效所需力矩抬高了 1.65 倍。**

**⇒ 真机余量**：所需 ≈ 0.48 N·m vs 舵机 0.7 N·m → **余量 46%**。
（这与最初"折叠要抬起整机，余量仅 5%"的估算**不同** —— 后者是错的。）

### 4.6 定案

**保持 0.7 N·m 真实规格 + 世界步长 `0.001 s`。**

理由：
- 1 ms 下 RTF = **0.996**（几乎零代价）
- 保真：0.7 N·m 就是 0.7 N·m，不会误导后续设计判断
- 精确：52.00° 到位，两臂差 0.000°

**不采用"把关节 effort 虚报成 1.0 来迁就 4 ms"的做法** ——
那会让仿真里的舵机比真机强 43%，后续如果拿仿真校核力矩余量会得出错误结论。

---

## 第 5 章 · 概念辨析：仿真步长 vs 真机速率

**这一节是整份文档里最容易反复踩的概念坑，单独讲清楚。**

### 5.1 真机上没有"步长"这个东西

"步长"是**仿真专有概念** —— 仿真把连续的时间切成小块，一块一块地推算。

```
真机：时间连续流动，物理是"真"的
仿真：每 1 ms 或 4 ms 算一步 → 这叫"步长"
```

**真机上不存在"每 4 ms 算一次物理"这个动作。**

### 5.2 `max_step_size` 到底是什么

世界文件里：

```xml
<physics type="ode">
  <max_step_size>0.001</max_step_size>          <!-- 每步走多少【仿真时间】 -->
  <real_time_update_rate>1000</real_time_update_rate>  <!-- 每秒【真实时间】想走多少步 -->
</physics>
```

Gazebo 内部就是一个循环：

```
仿真时间 t = 0
┌─ 循环 ──────────────────────────────────────────┐
│  ① 所有插件系统的 OnUpdate(当前时间, dt)          │
│     ← 舵机控制器在这里读角度差、算出该出多大力      │
│     ← 接触检测也在这里：轮子和地面重叠了多少        │
│  ② 物理引擎推进 dt                                │
│     ← 拿上一步算出的力，把刚体往前推 dt             │
│  ③ 发布传感器数据 / 话题                          │
│  t += dt                                         │
└─────────────────────────────────────────────────┘
```

**`max_step_size = 0.001` 的意思：这个循环每转一圈，仿真时间前进 0.001 秒。**

### 5.3 两个后果

#### 后果一：力是"算一次、用一整段"（零阶保持）

物理引擎在整个 dt 期间用的都是第①步算出的**那一个**力矩，中途不再问：

```
dt = 4 ms：  算 → ─────用这个力 4ms─────→ 算 → ─────4ms─────→ 算
dt = 1 ms：  算 → ─1ms─→ 算 → ─1ms─→ 算 → ─1ms─→ 算 → ─1ms─→ 算
```

**dt 越大，控制器两次"睁眼"之间的盲区越长。**

#### 后果二：接触穿透更深

穿透是在两次检测之间累积起来的 → 步长越大，穿透越深 → 恢复力越大
→ **表现就是"地面变硬了"**。

### 5.4 两个"时间"千万别混

| | 含义 |
|---|---|
| **仿真时间** | `t += dt` 累加出来的，给物理用 |
| **真实时间** | 你手腕上手表的时间 |

**`RTF`（实时率）= `real_time_update_rate` × `max_step_size`**

```
1000 × 0.001 = 1.0   ← 仿真速度和现实一样快 ✓（实测 0.996）
 250 × 0.004 = 1.0   ← 同样也是 1.0
```

⚠️ **改步长时必须两个数一起改**：如果在 4 ms 基础上只把 `max_step_size`
改成 0.001、`real_time_update_rate` 保持 250，那 `RTF = 250 × 0.001 = 0.25`
→ **仿真会变成 1/4 速度**。

### 5.5 和 PX4、和真机的对应关系

```
真机：
   飞行器物理 ──连续──→ 舵机（内部 1~3 kHz 闭环，出力 ≤0.7 N·m）
                          ↑
   PX4 每 4 ms 送一个【目标角度】

仿真：
   Gazebo 每 dt 算一次物理
      ↑
   Gazebo 的 JointPositionController ← 充当"舵机内部控制器"的替身
      ↑
   PX4 每 4 ms 送一个【目标角度】（和真机一样）
```

**三个要点：**

1. **PX4 的 4 ms 在真机和仿真里都存在**（源码 `mixer_module.cpp:325` 写死：
   `fixed_rate_scheduling_interval = 4_ms; // schedule at 250Hz`），
   那是"送目标角度"的间隔，**不受仿真步长影响**
2. **仿真步长决定的是"舵机内部控制器"多快调一次**：

   | | 这个"虚拟舵机"每秒调节次数 |
   |---|---|
   | 仿真 4 ms | 250 次 |
   | 仿真 1 ms | 1000 次 |
   | **真舵机** | **1000~3000 次** |

3. **⇒ 单从"像不像真舵机"来看，1 ms 明显比 4 ms 更接近真实**

### 5.6 为什么不干脆用 0.0001 s

**代价正比于步数：**

| 步长 | 每秒步数 | CPU | 实测 RTF |
|---|---|---|---|
| 4 ms | 250 | 1× | 1.0 |
| **1 ms** | **1000** | **4×** | **0.996** ← 还跑得动 |
| 0.1 ms | 10000 | 40× | 会掉出实时 |

**1 ms 是"精度够用 + 还跑得动"的甜点。**

### 5.7 一句话总结

> **`max_step_size` = Gazebo 把连续的物理切成多细的碎片来推算。**
> 碎片越粗（4 ms），控制器"睁眼"的间隔越长、接触穿透越深，
> 于是等效阻力被人为抬高 1.65 倍。

⚠️ **反向陷阱**：不要因为"仿真里 4 ms 折不动"就得出"真机需要更大舵机"的结论
—— 那是被伪影误导。真机甲实际所需 ≈0.48 N·m，0.7 N·m 余量 46%。

---

## 第 6 章 · 可复用的排查手段

### 6.1 先读 PX4 日志！

最容易犯的错就是**一直跑测试却不读日志**。这次的三个关键线索全在日志里：

```
ERROR [mavlink] param types mismatch param: COM_DISARM_LAND
INFO  [commander] Disarmed by auto preflight disarming
INFO  [commander] Connection to ground station lost
```

### 6.2 PX4 SITL 自带 shell，可以从 stdin 发命令

PX4 SITL 默认开启 pxh shell 并读 stdin（`-d` 才关闭）。用 `Popen(stdin=PIPE)`
然后写命令，就能**直接问 PX4 内部状态**：

```python
sh("gz_bridge status")            # 打印每个输出通道的
                                  # func / value / failsafe / disarmed / min / max / center
sh("listener vehicle_command -n 3")   # 看命令是否真的进了 uORB
```

**排查时先看这个，比瞎猜快得多。**（这次就是靠它确认"命令确实进了 uORB
且参数完全正确"，从而把范围缩到输出环节。）

### 6.3 二分法：把生成的模型放进普通世界跑

`servo_px4model_only.py` —— 用 PX4 的世界 + 我们生成的 `model.sdf`，
**完全不启动 PX4**，直接往舵机话题发角度。

一次就把范围劈成两半：
- 能折 → 模型没问题，是 PX4 运行时引入的
- 不能折 → `model.sdf` 与 `robot.urdf` 有差异

**这次正是靠它确认"问题在模型/引擎侧，与 PX4 运行时无关"。**

### 6.4 零重力测试

判断"负载是否参与"的唯一正确方式：

```bash
GRAVITY="0 0 0" python3 build_px4_model.py    # 永久悬浮，无任何接触
```

**不要用"把 spawn 高度调高" —— 那是无效的（机器人会落地）。**

### 6.5 环境变量形式的诊断开关

`build_px4_model.py` 里所有诊断开关都是环境变量，默认关闭，用完即弃：

| 开关 | 用途 |
|---|---|
| `SERVO_EFFORT=<N>` | 临时改舵机关节 effort，验证"是否力矩不足" |
| `SERVO_GAINS=d_gain=0,p_gain=1` | 覆盖舵机 PID 增益（调参用） |
| `STEP_SIZE=<s>` | 覆盖物理步长（**主开关，默认已是 0.001**） |
| `GRAVITY="0 0 0"` | 覆盖重力（**零负载测试的唯一正确做法**） |

**曾用过但实测无效、已移除的（记录以免重走）：**

| 试过的 | 结果 |
|---|---|
| 换物理引擎 `dartsim` | 仍失败 |
| 关节 `<dynamics><damping>` 置 0 | 仍失败 |
| ODE 接触 `<contact><ode><min_depth>` = 0.0005/0.001/0.002 | **完全无效**（都是 0.00°） |
| 覆盖 PID 增益 `d_gain=0,i_gain=0` | 仍失败 |

### 6.6 顺便修掉的伪装成"我们的问题"的陷阱

- 关节角话题是 **`/airground/joint_state`**（在 `model.sdf` 的
  `JointStatePublisher` 里定义），**不是 `/joint_state`**
  → 订阅错话题会一条数据都收不到
- `axis1.force` 在 dartsim 下**恒为 0**（引擎不填这个字段），别指望用它量力矩
- 世界改名后，所有 `/world/<名字>/...` 的话题路径都要跟着改

---

## 第 7 章 · 最终成果与改动清单

### 7.1 新增文件

| 文件 | 作用 |
|---|---|
| `servo_test.py` | **舵机端到端测试**：清参数 → 启动 → 只读校验参数 → 测未解锁位 → 解锁 → 发命令 → 记录关节角 + 多重探针 |
| `servo_probe.py` | **深度探针**：通过 stdin 给 PX4 发 `gz_bridge status`，直接读内部通道状态 |
| `servo_px4model_only.py` | **二分实验**：只跑生成的 `model.sdf`，不接 PX4 |
| `servo_standalone_test.py` | **对照组**：用 `robot.urdf` 跑独立世界 |

### 7.2 `build_px4_model.py` 的改动

| # | 改动 | 原因 |
|---|---|---|
| 1 | 接口参数改 `param set`（原来 `set-default`） | 避免被持久化参数顶掉 |
| 2 | 舵机参数改为正确的 `[0,1000]` 值域 | 原来写成 `[-1,1]` 越界被拒 |
| 3 | 补上 `reference="base"` 改名 | **真 Bug**：漏了这种引用形式 → `<self_collide>false</self_collide>` 被静默丢弃 |
| 4 | 新增自定义世界 `worlds/airground.sdf` | 地面摩擦 0.4（与轮子配套）+ 1 ms 步长 |
| 5 | 模型根节点补 `<pose>` | 避免刷进地面 |
| 6 | 模型级 `<self_collide>false</self_collide>` | 补 URDF 的意图 |
| 7 | airframe 加 `COM_RC_IN_MODE 1` / `NAV_DLL_ACT 0` | 解锁必需 |
| 8 | 加一组环境变量诊断开关 | 便于 A/B 排查 |

### 7.3 对 PX4 仓库的改动（保持干净）

- 新增 `Tools/simulation/gz/models/airground/`（模型）
- 新增 `Tools/simulation/gz/worlds/airground.sdf`（世界）
- 新增 `ROMFS/px4fmu_common/init.d-posix/airframes/4030_gz_airground`（airframe）
- 修改 `ROMFS/.../airframes/CMakeLists.txt` **仅 +1 行**（登记机架名）
- **`src/` `platforms/` `msg/` `cmake/` `boards/` 零改动**

---

## 第 8 章 · 经验提炼

### 8.1 把"我改了 A，但看到的是 B"当成第一类信号

只要出现"改了没效果"，**先怀疑两件事**：

1. **参数被顶掉了**（`set-default` vs 持久化）
2. **我的改动根本没生效**（改名漏了、正则没匹配、话题名不对）

这次的坑一（参数持久化）和"`reference="base"` 没改名"都属于这一类。

### 8.2 "返回成功"不等于"真的成功"

- `ARM` 回 `ACCEPTED` ≠ 解锁了 → 要查 HEARTBEAT 的解锁位
- `param_set_send` 没报错 ≠ 参数写进去了 → 类型不匹配会**静默丢弃**
- `gz_bridge` 收到命令 ≠ 舵机会动 → 没解锁时输出全走 DIS 值

**凡是关键状态，都要找一条独立的路径去确认。**

### 8.3 先二分，再深挖

不要一上来就盯着最可疑的代码看。
**先用二分实验把范围劈成两半**（这次是"脱离 PX4 跑模型"），
能省掉大量瞎猜。

### 8.4 检查自己的实验方法本身

这是这次最大的教训：

- ❌ "把机器人放到高处"来模拟零负载 → 它会落地 → 测的还是"有负载"
- ❌ 世界改了名但脚本没改 → 静默读到 0 条 → RTF 误报

**在得出"物理结论"之前，先确认"测量方法本身是有效的"。**

### 8.5 区分"仿真伪影"和"真实物理"

| | 该不该改设计？ |
|---|---|
| 仿真里 4 ms 折不动 | ❌ **是伪影**，不该动机构 |
| 仿真里 1 ms 需要 0.48 N·m | ✅ 是真实需求，可以参考 |

**判据**：把同一件事在**不同数值设置下**做一遍（步长、引擎、零重力）。
结论一致 → 可信；结论漂移 → 说明是数值问题。

### 8.6 保真优先于"能跑就行"

把关节 effort 虚报成 1.0 能让 4 ms 跑通，但仿真里的舵机就比真机强 43%。
**一旦用这个仿真去校核力矩余量，就会得出错误结论。**
既然 1 ms 的代价只有 RTF 0.996，就没必要在保真上妥协。

---

## 第 9 章 · 人在回路：怎么舒服地控制摆臂

> 前面 8 章解决的是「能不能动」。
> 这一章解决的是「人怎么操作它」—— 以及**操作错了会有什么后果**。
>
> 时间：2026-10-14。本章发现全部有源码级证据 + 实测验证。

### 9.1 先把「功能号」和「命令」分清楚

很多人一开始会绕晕：为什么会有 301、407、310、187 这么多数字？
它们其实是**两个维度**的东西：

| 维度 | 是什么 | 例子 |
|---|---|---|
| **功能号**（`SIM_GZ_SV_FUNC1`） | 「这个输出通道去**哪里取值**」的地址 | 301 / 407 |
| **MAVLink 命令** | 「发送什么消息」 | 310 / 187 |

`mixer_module.cpp` 里的分发表是理解一切的钥匙：

```cpp
static constexpr FunctionProvider all_function_providers[] = {
    {Motor1,          MotorMax,                  &FunctionMotors::allocate},      // 101~112
    {Servo1,          ServoMax,                  &FunctionServos::allocate},      // 201~215
    {Peripheral_via_Actuator_Set1, Peripheral_via_Actuator_Set6,
                                                 &FunctionActuatorSet::allocate}, // 301~306 ← 我们在用
    {RC_Roll,         RC_AUXMax,                 &FunctionManualRC::allocate},    // 402~412（407=aux1）
    ...
};
```

**⇒ 填 301 → 通道接到 `FunctionActuatorSet`；填 407 → 接到 `FunctionManualRC`。
一个 `int` 装不下两个地址，这就是「互斥」的本质。**

> 类比：`SIM_GZ_SV_FUNC1` 像**一个换台旋钮**，不是「多插一根线」。

### 9.2 四条路总览

| | ① QGC<br/>Actuators 滑条 | ② **QGC 自定义动作**<br/>**按钮 ×9** | ③ `servo_slider.py` | ④ 407 + aux1 |
|---|---|---|---|---|
| **在 QGC 界面里** | ✅ | ✅ **Fly View** | ❌ 独立窗口 | ❌ |
| 形态 | 连续滑条 | 9 个按钮 | 连续滑条 | 手柄轴 |
| 命令 | `310` | **`187`** | 310/187 自动切 | aux1 |
| 功能号 | 301 | 301 | 301 | **407** |
| 未解锁 | ✅ | ❌ | ✅ | ❌ |
| **已解锁** | ❌ | ✅ | ✅ | ✅ |
| 触地保护 | ❌ | ❌ | ✅ | ❌ |
| 改 airframe | ❌ | ❌ | ❌ | ✅ |

**⇒ 日常用法：未解锁用①，已解锁用②。**

### 9.3 路① QGC Actuators 滑条（310，只在未解锁时有效）

**位置**：`Vehicle Setup → Actuators →` 右栏"Actuator Testing"区

**操作**：先拨开关 `Propellers are removed - Enable sliders` → 滑动条

**滑条标题不是「Servo 1」！** 是 **`Peripheral via Actuator Set 1`**
（因为功能号 301 的名字叫这个，见 metadata 的 `functions_v1`）：

```json
"functions_v1": {
    "101": {"label": "Motor 1"},
    "201": {"label": "Servo 1"},                      ← 不是我们
    "301": {"label": "Peripheral via Actuator Set 1"}, ← ★ 我们
    "407": {"label": "RC AUX 1"}
}
```

**⚠️ 一共会出现 5 个滑条**：`Motor 1`~`Motor 4`（桨，范围 0~1）+ `Peripheral via Actuator Set 1`（我们，范围 −1~+1）。**只拖第 5 个。**

**为什么能看到**（证据链）：

1. PX4 生成 `build/px4_sitl_default/actuators.json`
   （`Tools/module_config/generate_actuators_metadata.py` 从 `output_functions.yaml` 生成），
   通过 `COMPONENT_METADATA` 上报。其中：
   ```json
   {"label": "SIM_GZ", "show-subgroups-if": "SIM_GZ_EN>=1",
    "subgroups": [{"label":"ESCs",  "param_prefix":"SIM_GZ_EC"},
                  {"label":"Servos","param_prefix":"SIM_GZ_SV"},
                  {"label":"Wheels","param_prefix":"SIM_GZ_WH"}]}
   ```
   → 所以 airframe 里那句 `param set SIM_GZ_EN 1` 是**必须的**。
2. QGC `Actuators::parametersChanged()` 读每个通道的 `*_FUNC*` 参数值，
   为每个功能号建一个 `ActuatorTesting::Actuator`
   （功能号不匹配任何 actuator 类型时用 `actuator-types.DEFAULT` 兜底）。
3. `ActuatorComponent.qml` 的 `sliderRepeater` → `ActuatorSlider`。

**它发什么**：`MAV_CMD_ACTUATOR_TEST`(310)，`param5 = 1000 + 301 = 1301`
```cpp
// Commander::handleCommandActuatorTest
if (function < 1000) { /* 1..12→电机, 33..47→舵机, ELSE→UNSUPPORTED（301 被拒） */ }
else { actuator_test.function -= 1000; }   // ★ QGC 发的正是 1000+301
```

**✅ 免解锁也能动** —— 靠 `mixer_module.cpp:481` 的后门：
```cpp
// mixer_module.cpp:461  正常路径：未解锁 → NAN
if (_armed.armed || (_armed.prearmed && _functions[i]->allowPrearmControl())) {
    outputs[i] = _functions[i]->value(...);
} else {
    outputs[i] = NAN;
}

// mixer_module.cpp:481  actuator_test 的专门后门，覆盖掉上面的 NAN
if (!_armed.armed && !_armed.kill) {
    _actuator_test.overrideValues(outputs, _max_num_outputs);
}
```

**⇒ 这是唯一一条「未解锁也能驱动舵机」的路。副产品：电机不转，安全。**

**⚠️ 行为**：QGC 的 `sendTimer`（50 ms、`repeat:true`）会持续重发 → **松手后保持**；
一旦解锁，PX4 会回 `DENIED`（`if (isArmed()) return DENIED`），滑条失效。

**✅ 实测（2026-10-14，全程未解锁）**：
```
MAV_CMD_ACTUATOR_TEST(310), param1=value, param2=1.0, param5=1301, 20 Hz 重发

−1 → servo_0 = -0.00°  servo_1 = -0.00°  两臂差 -0.000°
 0 → servo_0 = 24.58°  servo_1 = 24.63°  两臂差 -0.041°
+1 → servo_0 = 52.00°  servo_1 = 52.00°  两臂差 +0.000°
```

### 9.4 路② QGC 自定义动作按钮（187，**已解锁也能用**）

**为什么需要它**：310 需要未解锁，187 需要已解锁（`mixer_module.cpp:461`）—— **两者互补**。
想在「已解锁但还在地面」时折摆臂，只能用 187，而 QGC 的 Actuators 滑条给不了。

**QGC 的机制**（`src/QmlControls/MavlinkActionManager.cc`）：从 JSON 文件读自定义命令，
显示在 **Fly View → Actions 面板**。

**⚠️ 目录跟随 locale！** 由 `SettingsManager::appSettings()->mavlinkActionsSavePath()` 决定，
本机是中文 locale → 实际路径是
```
/home/yuan/文档/QGroundControl/MavlinkActions/      ← 不是 ~/Documents/！
```

**文件格式**（必填 `label` / `description` / `mavCmd`，其余可选）：
```json
{
  "version": 1,
  "fileType": "MavlinkActions",
  "actions": [
    {"label": "摆臂 26° (中间)", "description": "...",
     "mavCmd": 187, "compId": 1, "param1": 0.0, "param7": 0}
  ]
}
```

**已部署**：`qgc_actions/AirArmActions.json`（工作区副本）
→ `/home/yuan/文档/QGroundControl/MavlinkActions/`

9 档**单调递增**（`param1 = angle/26 − 1`）：

| # | param1 | 关节角 | 按钮 |
|---|---|---|---|
| **1** | −1.00 | **0.0°** | **★ 摆臂 0° (飞行安全位)** ← 永远在最上 |
| 2 | −0.75 | 6.5° | 摆臂 6.5° |
| 3 | −0.50 | 13.0° | 摆臂 13° |
| 4 | −0.25 | 19.5° | 摆臂 19.5° |
| 5 | 0.00 | 26.0° | 摆臂 26° |
| 6 | +0.25 | 32.5° | 摆臂 32.5° |
| 7 | +0.50 | 39.0° | 摆臂 39° |
| 8 | +0.75 | 45.5° | 摆臂 45.5° |
| **9** | +1.00 | **52.0°** | **◆ 摆臂 52° (地面行驶位)** ← 永远在最下 |

> **为什么单调递增**：手滑点到 `0°` 的下一档，最多是 6.5°（而不是原来的 13° 或 52°）。
> **风险按距离线性递减。**

**启用步骤**：
```
QGC → Application Settings → Fly View → MAVLink Actions
      → "Fly View Actions" 下拉框选 AirArmActions.json
      → 回 Fly View → 点工具栏 "Actions"（≡）按钮
```

**为什么 187 免检**？`Commander.cpp` 里它**一行条件都没有**：
```cpp
case vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR:
    answer_command(cmd, vehicle_command_ack_s::VEHICLE_CMD_RESULT_ACCEPTED);
    return true;
```
**真正的门控在 `mixer_module.cpp:461`（需要 armed）→ 正好和 310 互补。**

### 9.5 路③ `servo_slider.py`（备选，自带触地保护）

**是独立 tkinter 窗口，不在 QGC 里。** 按 arm 状态自动选命令：

| 状态 | 命令 | 发送策略 |
|---|---|---|
| 未解锁 | `310` | 20 Hz 持续重发（timeout 上限 3 s） |
| 已解锁 | `187` | 值变化时发一次 + 1 Hz 补发 |

**额外功能**：角度精确输入、预设按钮、实时显示 arm/高度/拒绝计数、
**触地保护**（高度 > 0.20 m → 强制回 0°）、退出时自动归零。

```bash
python3 servo_slider.py            # 默认连 udpout:127.0.0.1:18570
python3 servo_slider.py --no-guard # 关掉高度保护
```

### 9.6 路④ 407 + `MANUAL_CONTROL.aux1`（已实现但不推荐）

**什么时候考虑**：你想用**真实手柄**做实时遥操作（比如降落时手动调摆臂角度）。

**启用方式**：
```bash
SERVO_SRC=aux1 python3 build_px4_model.py && cd /home/yuan/PX4-Autopilot && make px4_sitl gz_airground
```

**⚠️ 四个必要条件（缺一不通）**：

| # | 条件 | 源码依据 |
|---|---|---|
| 1 | `SIM_GZ_SV_FUNC1 = 407` **写进 airframe** | 运行时改不可靠：`updateSubscriptions()` 里 `if (!_need_function_update \|\| _armed.armed) return false;` |
| 2 | `COM_RC_IN_MODE = 1` | 默认 **0 = RcOnly**，`ManualControlSelector::isInputValid()` 会直接丢弃 MAVLink 输入 |
| 3 | **必须解锁** | 同 `mixer_module.cpp:461` |
| 4 | **必须持续发**（≥2 Hz） | `COM_RC_LOSS_T = 0.5 s`，超时后 `FunctionManualRC::resetAllToDisarmedValue()` → `_data[5] = NAN` → 回 disarmed 位 |

**⚠️ 硬伤：0.5 秒后弹回 0°。** 因为它是**跟随器**不是锁存器：
```cpp
// FunctionManualRC::update()  —— 每帧都从 topic 重新取值
if (_topic.update(&mcsp)) {
    if (mcsp.valid) { _data[5] = mcsp.aux1; }
    else           { resetAllToDisarmedValue(); }   // ← 全刷 NAN
}
```
对比 301 通路是**锁存器**（`FunctionActuatorSet::update()` 只写不清）：
```cpp
while (_topic.update(&vc)) {
    if (vc.command == VEHICLE_CMD_DO_SET_ACTUATOR) { _data[0] = vc.param1; }
}
// ⚠️ 没有 else 分支！收不到命令就什么都不做 → 值一直留着
```

**⚠️ 与 301 互斥**：`SIM_GZ_SV_FUNC1` 是一个 `int`，填 407 就没 301 —— 连 `DO_SET_ACTUATOR` 也一起失效。
「开两个通道分别接」也走不通：话题名 `/model/<实例>/servo_<i>` 是 gz_bridge 硬编码，
而且**一个 joint 只能挂一个位置控制器**，两个闭环会互相打架。

**⚠️ 不要用 MAVLink 批量 `param_set`**（尤其 `SIM_GZ_EC_FUNC*`）——
实测会破坏 gz_bridge 的函数映射，之后函数输出全变 NAN。
需要改参数就用 **pxh shell**（见 6.2）。

**排查工具**：`aux_diag.py` —— 一边发 aux1、一边用 pxh shell 逐段查 uORB
（`listener manual_control_input` / `listener manual_control_setpoint` / `gz_bridge status`）。

> **⚠️ pymavlink 的方言陷阱**：`ardupilotmega` 方言的 `MANUAL_CONTROL` **只有**
> `target/x/y/z/r/buttons`，**没有 aux 字段**。必须
> `from pymavlink.dialects.v20 import common as mav_common`
> （`v10/common` 也没有，只有 **v20** 有 `buttons2/enabled_extensions/s/t/aux1..aux6`）。
> 另：`MAV_CMD_DO_SET_ACTUATOR`/`MAV_CMD_ACTUATOR_TEST` 在 ardupilotmega 里**没有名字常量**，
> 只能用数字 **187 / 310**。

### 9.7 安全设计：触地保护

#### 为什么需要它

**先看机构关系**（`robot.urdf` 的关节树）：

```
base (机身)
 ├─ link9  ← joint1 (轴 +X)  ← 左摆臂
 │    ├─ link1 / link3   螺旋桨 ✈️
 │    └─ link5 / link7   轮子  ⚙️
 └─ link10 ← joint2 (轴 −X)  ← 右摆臂
      ├─ link2 / link4   螺旋桨 ✈️
      └─ link6 / link8   轮子  ⚙️
```

**⇒ 四个螺旋桨和四个轮子【全部挂在摆臂上】。摆臂角 = 桨盘倾角。**

#### 飞行中折摆臂的后果（已量化）

TWR = 3.0（满油门总推力 = 3G）：

| 摆臂角 θ | 垂直分量 cos θ | **水平分量 sin θ** |
|---|---|---|
| 0° | 3.00 G | 0 |
| 26° | 2.70 G | 1.31 G |
| **52°** | **1.85 G** | **2.36 G ≈ 23 m/s²** |

三件事同时发生：

1. **水平被猛甩** —— 2.36 G ≈ 23 m/s² → 1 秒内 83 km/h
2. **俯仰力矩爆炸** —— 桨在 YZ 平面相对转轴半径 **0.1229 m**，摆到 52° 时
   **z 位移 ≈ +9.7 cm**；推力方向又偏 52° → 倒立摆效应放大 2~3 倍
3. **⚠️ 最致命：PX4 根本不知道摆臂转了**
   ```bash
   param set CA_ROTOR0_PX +0.1008     # 写死的常量，而且没有 PZ
   param set CA_ROTOR0_PY +0.1100
   ```
   → 控制分配矩阵认为「桨在固定位置、推力永远朝上」→ 补偿量完全错 → **姿控发散**

#### PX4 有没有现成的门控？—— 没有（已逐一查证）

| 查什么 | 结果 |
|---|---|
| `DO_SET_ACTUATOR` 在 Commander 里的门控 | ❌ 一行条件都没有 |
| `FunctionActuatorSet` 的 `param7` 有没有别的分支 | ❌ 只有 `index == 0` 一个分支 |
| `vehicle_land_detected` 参与执行器门控吗 | ❌ 只用于自动上锁 / home 位置 / 起飞检测 |
| 本版本有没有 `landing_gear` 模块 | ❌ 没有 |
| 有没有「按高度门控执行器」的机制 | ❌ **完全没有** |

#### 三条路

| 路 | 做法 | ✅ / ❌ |
|---|---|---|
| **A. 改 PX4 源码** | `Commander.cpp` 里加 `if (!_vehicle_land_detected.landed) DENIED;` | ✅ 真硬拒绝、永久生效<br/>❌ 改源码、每次升级要重打补丁、**全局生效**（会同时禁掉 `servo_slider.py` 的 187） |
| **B. 外部守护脚本** | 20 Hz 检测高度，空中强制发 `187 param1=-1.0` 拉回 | ✅ PX4 零改动、只影响摆臂<br/>❌ 必须一直跑着（忘了启动 = 没保护） |
| **C. 靠纪律** | 按钮命名带语义 + 单调递增排序 + 操作清单 | ✅ 零成本<br/>❌ 手滑就是手滑，**无软件兜底** |

**⚠️ 「拉回」到底够不够快？** —— 够：

| 项 | 数值 |
|---|---|
| 摆臂 0° → 52° 响应时间 | ≈ 1 s → 角速度 ≈ **52 °/s** |
| 守护脚本检测延迟（20 Hz） | 50 ms |
| **→ 被拉回前最多转了** | **≈ 2.6°** |
| 推力偏 2.6° 的水平分量 | sin 2.6° = 4.5% → **0.13 G**（几乎无感） |

**⇒ 物理上「拉回」和「拒绝」几乎没有区别。**

#### 定案：**当前选 C**

```
现在（纯地面调试）      →  C
开始带桨测试            →  换 B（桨转起来后误触代价变大）
真机试飞                →  A 或 B，必须有软件保护
```

#### 两种模式的按钮风险矩阵

| 控制方式 | 命令 | 飞行中能发吗 | 风险 |
|---|---|---|---|
| QGC Actuators 滑条 | 310 | ❌ PX4 直接拒绝 | ✅ **天然安全** |
| **QGC 自定义动作按钮** | **187** | ✅ 能发 | 🔴 **点一下 = 炸机** |
| `servo_slider.py` | 310/187 | ✅ 能发 | 🟢 有触地保护 |
| 407 + aux1 | aux1 | ✅ 能发 | 🔴 手柄误碰 |

### 9.8 配套工具

#### `view_joints.py`（已修好，两种模式通用）

**⚠️ 两种模式的话题名和关节名【都不一样】**，这是最容易搞错的地方：

| 模式 | 世界名 | 话题 | 舵机关节名 |
|---|---|---|---|
| 独立 Gazebo | `default` | `/joint_state` | `joint1` / `joint2` |
| **PX4 仿真** | `airground` | **`/airground/joint_state`** | **`servo_0` / `servo_1`** |

现在脚本自动都试，并显示数据源 + 两臂差值：
```bash
python3 view_joints.py                 # 自动找话题
python3 view_joints.py 0.3             # 更快的刷新
python3 view_joints.py 0.5 /airground/joint_state   # 指定话题
```

#### ⚠️ 别用 `grep -A2` 看关节角

```bash
gz topic -e -t /airground/joint_state | grep -A2 'servo_'   # ❌ 看不到角度
```
因为真正的角度在第 18 行：
```
joint {                              
  name: "servo_0"        ← grep 命中      (行 1)
  id: 48                                 (行 2)  ← -A1
  parent: "base_link"                    (行 3)  ← -A2  ← 停在这
  child: "link9"
  pose { ... }
  axis1 { xyz { x: 1 } limit_upper: 0.9076 damping: 0.2
    position: -7.32e-13                  (行 18) ← ★ 角度在这
    velocity: -4.4e-16 }
}
```
**用 `-A20`，或者干脆用 `view_joints.py`。**

---

## 附录 · 命令速查

```bash
cd "/home/yuan/robot air-ground/robot_urdf (4)"     # ⚠️ 路径含空格，必须加引号

# ── 生成 PX4 模型 + 世界 + airframe ──
python3 build_px4_model.py

# ── 启动仿真 ──
cd /home/yuan/PX4-Autopilot && make px4_sitl gz_airground            # 无界面
cd /home/yuan/PX4-Autopilot && PX4_GZ_HEADLESS=0 make px4_sitl gz_airground   # 带界面（配 QGC 用）

# ── 舵机测试 / 排查 ──
python3 servo_test.py                 # 端到端（约 100 秒，自动清理进程）
python3 servo_probe.py                # 深度探针（读 PX4 内部状态）
python3 servo_px4model_only.py        # 二分实验（不接 PX4）
python3 servo_standalone_test.py      # 对照：独立世界跑 robot.urdf
python3 aux_diag.py                   # 逐段排查 aux(407) 通路（pxh shell 查 uORB）
python3 servo_aux_test.py             # 407/aux1 通路验证（需先 SERVO_SRC=aux1 重新生成）

# ── 实时看关节角（⭐ 两种模式通用）──
python3 view_joints.py                # 自动找话题（/airground/joint_state 或 /joint_state）
python3 view_joints.py 0.3            # 自定义刷新间隔
python3 view_joints.py 0.5 /airground/joint_state   # 指定话题

# ── 人在回路控制（第 9 章）──
python3 servo_slider.py               # 独立滑条窗口（未解锁 310 / 已解锁 187，自带触地保护）
python3 servo_slider.py --no-guard    # 关掉高度保护

# ── 诊断开关（A/B 用，默认全关）──
STEP_SIZE=0.004 python3 build_px4_model.py
SERVO_EFFORT=3.0 python3 build_px4_model.py
GRAVITY="0 0 0" python3 build_px4_model.py
SERVO_GAINS=d_gain=0 python3 build_px4_model.py
SERVO_SRC=aux1 python3 build_px4_model.py   # 把 SIM_GZ_SV_FUNC1 从 301 改成 407(RC_AUX1)

# ── 看参数持久化有没有被污染（出厂基线 312 字节）──
ls -l /home/yuan/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson
strings /home/yuan/PX4-Autopilot/build/px4_sitl_default/rootfs/parameters.bson | grep -i SIM_GZ

# ── 清理 ──
pkill -9 -f 'bin/px4'; pkill -9 -f 'gz sim'
```

**舵机命令（pymavlink 的方言是 ardupilotmega，**没有** `MAV_CMD_DO_SET_ACTUATOR` /
`MAV_CMD_ACTUATOR_TEST` 这两个名字常量，必须用数字 187 / 310）：**

```python
# ① 187 = DO_SET_ACTUATOR —— 锁存型，【必须已解锁】（mixer_module.cpp:461）
conn.mav.command_long_send(sysid, comp, 187, 0, value, 0, 0, 0, 0, 0, 0)
#                              ↑    ↑   ↑
#                          命令号 确认 param1（-1 → 0°，0 → 26°，+1 → 52°）
#                                     param7=0 表示"用 param1~6 对应执行器 1~6"

# ② 310 = ACTUATOR_TEST —— 必须【未解锁】，但免解锁即可驱动舵机
#    参数5 = 1000 + 功能号 = 1000 + 301 = 1301；必须 20 Hz 持续重发（timeout 上限 3 s）
conn.mav.command_long_send(sysid, comp, 310, 0, value, 1.0, 0, 0, 1301, 0, 0)
#                                              ↑     ↑            ↑
#                                          param1 timeout     param5
```

**角度换算（已由 `SIM_GZ_SV_MINA1=0` / `MAXA1=52` 决定）：**
$$\text{angle} = 26 \times (value + 1), \qquad value = \frac{\text{angle}}{26} - 1$$

### QGC 里的两种控制入口

```
① 未解锁（地面调机构）
   QGC → Vehicle Setup → Actuators → 拨开安全开关
       → 拖 “Peripheral via Actuator Set 1”（范围 −1~+1）
       ⚠️ 前 4 个滑条是 Motor 1~4（螺旋桨），不要碰

② 已解锁（地面调机构）
   QGC → Application Settings → Fly View → MAVLink Actions
       → "Fly View Actions" 选 AirArmActions.json
       → 回 Fly View → 点工具栏 "Actions"（≡）
       → 9 个按钮（0° 在最上，52° 在最下）
```

**QGC 自定义动作文件位置（⚠️ 路径跟随 locale）：**
```
/home/yuan/文档/QGroundControl/MavlinkActions/AirArmActions.json
           ↑ 中文 locale 是“文档”，不是 ~/Documents/
```
源文件（工作区副本）：`qgc_actions/AirArmActions.json`

---

## 附：还没做完的事

- [ ] **带载折叠时序**：目前舵机测试是**停桨**做的（怕推力掀翻机器人触发自动上锁）。
      真机时序上要确认 **降落触地 → 停桨 → 再折臂**，否则带推力折叠很危险
- [ ] **触地保护（第 9.7 节）**：当前选**路线 C（靠纪律）**。
      开始带桨测试前应改成 **路线 B（外部守护脚本）**：
      20 Hz 检测高度，空中强制发 `187 param1=-1.0` 拉回 0°
- [ ] **地面行驶**：轮子接口 `SIM_GZ_WH_*` 发的是 `Actuators`，
      而 Gazebo 的 `JointController` 只吃 `Double` → 需要桥接
- [ ] **空-地状态机接入 MAVLink**：把 `air_ground_ctrl.py` 的
      模式互斥 / 触地联锁搬到 PX4 侧（= 路线 B 的完整版）
- [ ] **QGC 虚拟摇杆端到端验证**：脚本已验证 `MANUAL_CONTROL` 通路可用，
      但还没用真实 QGC 手动飞一遍
- [ ] 起飞/降落时的**摆臂时序**与实际包线测试
- [x] ~~QGC 里能不能用一个滑条控制摆臂~~ → **✅ 已解决（第 9 章）**
      未解锁 → Actuators 滑条（310）；已解锁 → 自定义动作按钮 ×9（187）
