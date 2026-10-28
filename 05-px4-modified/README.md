# 修改后的 PX4 源码

本目录收录本项目对 **PX4-Autopilot v1.18.0-beta1** 的全部改动。

> 基于 commit：`PX4-Autopilot` v1.18.0-beta1（`/home/yuan/PX4-Autopilot`）
> 目标硬件：MicoAir H743 V2（STM32H743VIT6）

---

## 一、改动总览

| 类型 | 文件 | 说明 |
|:---|:---|:---|
| **新增模块** | `src/modules/ground_drive/` | 地面驱动外部模式（ground_drive.cpp 885 行 / hpp 210 行 / module.yaml 206 行 / Kconfig / CMakeLists） |
| **新增机架** | `ROMFS/px4fmu_common/init.d/airframes/4030_airground` | 实机空气帧 |
| **新增机架** | `ROMFS/px4fmu_common/init.d-posix/airframes/4030_gz_airground` | SITL 空气帧 |
| **修改** | `ROMFS/px4fmu_common/init.d/airframes/CMakeLists.txt` | +1 行（登记机架名，该文件是显式清单非 glob） |
| **修改** | `ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` | +1 行（同上） |
| **修改** | `boards/micoair/h743-v2/default.px4board` | +1 行 `CONFIG_MODULES_GROUND_DRIVE=y` |
| **修改** | `boards/px4/sitl/default.px4board` | +1 行（同上，供 SITL） |
| **新增资源** | `Tools/simulation/gz/models/airground/` | Gazebo 模型（model.sdf / model.config / meshes）※ 位于 git 子模块内 |
| **新增资源** | `Tools/simulation/gz/worlds/airground.sdf` | 自定义世界（1 ms 步长，地面 μ=0.4） |

**核心 `src/` 改动只有一处**：新增 `src/modules/ground_drive/`。
**未修改任何 PX4 原有源文件**（`src/` / `platforms/` / `msg/` / `cmake/` 零改动）。

---

## 二、目录说明

```
05-px4-modified/
├── px4-modifications.patch     ← 4 个被改文件的 unified diff（可直接 git apply）
├── files/                      ← 完整文件，按 PX4 原始相对路径组织
│   ├── src/modules/ground_drive/
│   ├── ROMFS/px4fmu_common/init.d/airframes/
│   ├── ROMFS/px4fmu_common/init.d-posix/airframes/
│   ├── boards/micoair/h743-v2/
│   ├── boards/px4/sitl/
│   └── Tools/simulation/gz/{models/airground, worlds/airground.sdf}
└── README.md
```

`files/` 下可直接整体覆盖到 PX4 源码树同名位置（新增文件 + 修改后的文件都是完整副本）。

---

## 三、应用方法

### 方法 A · 直接覆盖（推荐）

```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout v1.18.0-beta1

# 把 files/ 下的内容整体覆盖过去（保持相对路径）
cp -r /path/to/05-px4-modified/files/* .

# 编译
make micoair_h743-v2_default -j16
```

### 方法 B · 打补丁（只应用 4 个修改，新增文件仍需复制）

```bash
git apply /path/to/px4-modifications.patch
cp -r /path/to/05-px4-modified/files/src/modules/ground_drive src/modules/
cp /path/to/05-px4-modified/files/ROMFS/px4fmu_common/init.d/airframes/4030_airground \
   ROMFS/px4fmu_common/init.d/airframes/
cp /path/to/05-px4-modified/files/ROMFS/px4fmu_common/init.d-posix/airframes/4030_gz_airground \
   ROMFS/px4fmu_common/init.d-posix/airframes/
```

### 仿真资源（gz 是 git 子模块）

`Tools/simulation/gz` 在 PX4 中是 **git 子模块**，需单独处理：

```bash
cd PX4-Autopilot/Tools/simulation/gz
# 把 models/airground/ 与 worlds/airground.sdf 复制进来
```

或者用本仓库的生成器自动完成（推荐）：

```bash
cd /path/to/air-ground-robot
python3 03-tools/build/build_px4_model.py
```

该脚本会从 `01-simulation/world/robot.urdf` 生成 PX4 侧的模型 + airframe + 世界，
并内置 **8 项自校验**（尺寸、质量、桨位置、KM 符号、mesh 路径、步长等）。

---

## 四、`ground_drive` 模块概要

**作用**：把 PX4 当"地面车飞控"用 —— 注册一个外部导航模式（`ext1`），
通过 `setpoint_config{TYPE_DIRECT_ACTUATORS}` 让 `control_allocator` 让位，
然后直接向 `actuator_motors` 写入轮速指令。

**关键设计**（详见 `docs/PX4地面驱动方案.md` 第 4 章）：

| 机制 | 说明 |
|:---|:---|
| 外部模式注册 | 用 `register_ext_component_request` 自动注册，**不改 commander** |
| 让位开关 | `SetpointType::DirectActuators` → `control_allocator` 不参与混控 |
| 三段式门控 | 摆臂角度决定 `can_arm_and_run`，防止摆臂在飞行位时误入地面模式 |
| land detector 补偿 | 直驱模式下 `mc_rate_control` 不跑 → 补发 `vehicle_thrust_setpoint{0}`，否则 `landed` 恒 false |
| 摆臂角外推 | 无角度传感器，按 `GD_SERVO_RATE` 时间外推（保守取值） |
| 饱和保转向 | 差速超限时两侧等比缩放，保持转弯半径 |

**参数**（`module.yaml`）：

| 参数 | 默认 | 说明 |
|:---|:---:|:---|
| `GD_SERVO_RATE` | 40 | 摆臂角外推速率 [deg/s] |
| `GD_STICK_DZ` | 0.15 | 摇杆死区（抑制回中偏置）|
| `GD_SERVO_GATE` | 1 | 摆臂门控总开关（0=台架调试，危险）|
| `GD_ARM_MIN_DEG` | 48 | 允许 arm 的最小摆臂角 [deg] |
| `GD_MAX_SPEED` | 1.2 | 最大地面速度 [m/s] |
| `GD_WHEEL_TRACK` | 0.165 | 轮距 [m] |
| `GD_YAW_GAIN` | 1.5 | 转向增益 [rad/s] |
| `GD_MAX_OMEGA` | 2400 | 电机满输出角速度 [rad/s] |
| `GD_GEAR_RATIO` | 64 | 减速比 |
| `GD_WHEEL_R` | 0.04 | 轮半径 [m] |
| `GD_TURN_SLOW` | 1.0 | 满舵时纵向指令降速系数 |

---

## 五、airframe 关键参数（`4030_airground`）

```sh
# 构型
param set CA_AIRFRAME 0                    # 多旋翼
param set CA_ROTOR_COUNT 4
param set CA_ROTOR0_PX +0.1008 / PY +0.1100 / KM +0.05   # 前左
param set CA_ROTOR1_PX +0.1008 / PY -0.1100 / KM -0.05   # 前右
param set CA_ROTOR2_PX -0.1092 / PY +0.1100 / KM -0.05   # 后左
param set CA_ROTOR3_PX -0.1092 / PY -0.1100 / KM +0.05   # 后右
param set CA_R_REV 15                      # 4 路电机全部可逆（地面倒车必需）

# 输出映射
param set PWM_MAIN_FUNC1..4  101..104      # Motor1..4 → 输出 1~4
param set PWM_MAIN_TIM0      -4            # DShot300
param set PWM_MAIN_FUNC9/10  301           # 摆臂舵机（同一功能号 = 物理级同步）
param set PWM_MAIN_TIM3      50            # 舵机 PWM 50 Hz

# DShot 3D（双向）
param set DSHOT_3D_ENABLE    1
param set DSHOT_MIN          0
param set DSHOT_3D_DEAD_L    985           # ⚠️ 必须远小于转向指令 ±0.0825
param set DSHOT_3D_DEAD_H    1015

# 地面驱动
param set GD_STICK_DZ        0.15
param set GD_MAX_OMEGA       2400
param set GD_MAX_SPEED       1.2
param set GD_TURN_SLOW       1.0
```

> ⚠️ **`DSHOT_3D_DEAD_L/H` 是最容易踩的坑**：原地转向指令只有 `±0.0825`
> （`w_cmd=1.5 rad/s`，`Δv=w·W/2=0.124 m/s`，`control=Δv·64/(0.04×2400)`）。
> 死区一旦宽于该值，**前后能动但左右打到底完全无输出**。

---

## 六、编译与烧录

```bash
# 编译
make micoair_h743-v2_default -j16
# 产物：build/micoair_h743-v2_default/micoair_h743-v2_default.px4
# FLASH 占用约 96.6%（1898664 B / 1920 KB）

# 烧录（需先断开 QGroundControl，它独占 /dev/ttyACM0）
python3 Tools/px4_uploader.py build/micoair_h743-v2_default/micoair_h743-v2_default.px4
```
