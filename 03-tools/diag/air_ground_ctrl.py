#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 air-ground 机器人 · 控制状态机（第 1 阶段：舵机 + 轮子 + 安全联锁）
================================================================================

 实现的规则：
   1. 两舵机【强制同步】—— 靠仿真器实现：joint1/joint2 订阅同一个话题
   2. 舵机角度 0° ~ 52°（URDF 里也写了硬限位）
   3. 模式分界 48°：
        0° ~ 48°  → 🛫 飞行区：螺旋桨可用，轮子【强制停】
        > 48°     → 🛞 地面区：轮子可用，  螺旋桨【强制停】
   4. 轮子与螺旋桨【互斥】—— 代码结构上保证不可能同时非零
   5. 安全联锁：机身未落地时，舵机【禁止】超过 48°
      （判据 = 机身高度 z。不用 contact 话题 —— 后者在这个 gz 版本里不发布）

 运行前：先启动仿真
     cd "/home/yuan/robot air-ground/robot_urdf (4)"
     gz sim -v 4 -r robot_world.sdf

 运行：
     python3 air_ground_ctrl.py
================================================================================
"""

import math
import threading
import time

from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model
from gz.msgs10.pose_v_pb2 import Pose_V

# ==================== 可调参数 ====================

MAX_ARM_DEG = 52.0      # 舵机硬上限（与 URDF 的 0.9076 rad 对应）
MODE_THRESH = 48.0      # 飞行 / 地面的模式分界角
ARM_STEP = 2.0          # 每次输入的转动步长（度）
ARM_RATE_DEG_S = 16.0   # 舵机指令斜坡速率（度/秒）：48° ÷ 16 = 3.0 秒走完
ARM_RAMP_HZ = 50.0      # 斜坡刷新频率（Hz）
WHEEL_SPEED = 50.0      # 轮子目标转速 rad/s
PROP_SPEED = 1963.0     # 螺旋桨默认转速 rad/s
                        #   1963 = 悬停转速：w = sqrt(6.67 / (4 * 4.325e-7))
                        #          （4 桨推力恰好等于整机重力 6.67 N）
                        #   3400 = 满油门，推重比 3，会猛冲升空
GROUND_HOLD_DEG = 0.0   # 触发联锁时回退到的角度
ON_GROUND_Z = 0.20      # 机身高度低于此值算"已落地"（安全联锁判据）
                        #   ⚠️⚠️ 阈值不能取小！摆臂折叠【本身就会把机身撑高】：
                        #        0° → z = 0.021 m      52° → z = 0.101 m（实测）
                        #   若阈值卡在 0.08，摆臂转到 33° 时高度就超过阈值，
                        #   会被误判为"空中"而拽回 0°，高度降回来又被允许
                        #   → 死循环，永远折不到位（踩过）。
                        #   0.20 m = 机构最大高度的 2 倍，飞行时（≥1 m）也能正确判为空中。

# 你之前提到"从0到48时螺旋桨自动停下"有两种可能的理解，两个开关都留给你试：
PROP_STOP_ON_ANGLE = True    # ① 角度 > 48° → 桨强制停（与"48度后只能用轮子"一致）
PROP_STOP_ON_TRANSIT = False # ② 舵机一离开 0° 就停桨（收臂落地更保守）

# ==================== 话题名 ====================

MODEL_NAME = 'robot'                      # robot_world.sdf 里 <include><name>robot</name>
JOINT_TOPIC = '/joint_state'              # 由 JointStatePublisher 发布
POSE_TOPIC = '/world/default/pose/info'   # 由 SceneBroadcaster 发布（读机身高度判落地）
ARM_TOPIC = '/cmd/arm_angle'              # 两舵机共同订阅（强制同步）
WHEEL_TOPIC = '/cmd/wheel_speed'      # 四轮共同订阅（天然同步）
MOTOR_TOPIC = '/cmd/motor_speed'      # 四桨推力模型共同订阅（Actuators 数组）


def deg(rad):
    return rad * 180.0 / math.pi


class AirGroundController:

    def __init__(self):
        self.node = Node()
        self.pub_arm = self.node.advertise(ARM_TOPIC, Double)
        self.pub_wheel = self.node.advertise(WHEEL_TOPIC, Double)
        self.pub_motor = self.node.advertise(MOTOR_TOPIC, Actuators)

        self.lock = threading.Lock()
        self.arm_deg = 0.0          # 目标角度（度）—— 用户设定的
        self.arm_cmd = 0.0          # 实际下发角度（度）—— 按斜坡逼近目标
        self._ramp_t = time.time()  # 斜坡用的上一拍时刻
        self._warned = False        # 联锁警告只在状态翻转时打印一次
        self.actual_deg = 0.0       # 实测角度（度）
        self.on_ground = False      # 是否已落地（由机身高度判定）
        self.body_z = 0.0           # 实测机身高度（m）
        self.flight_on = False      # 螺旋桨手动开关
        self.wheel_on = False       # 轮子手动开关
        self.prop_speed = PROP_SPEED  # 螺旋桨目标转速 rad/s（可命令行调）
        self._joint_seen = False
        self._pose_seen = False

        self.node.subscribe(Model, JOINT_TOPIC, self._on_joint)
        self.node.subscribe(Pose_V, POSE_TOPIC, self._on_pose)

        # 斜坡发生器：以固定频率推进指令，让舵机 3 秒平滑走完
        #   ⚠️ 关键：阶跃指令会让积分瞬间饱和，造成超调/俯冲和"积分寻的"抖动
        threading.Thread(target=self._ramp_loop, daemon=True).start()

    # ---------------- 回调（在后台线程执行） ----------------

    def _on_joint(self, msg):
        # ⚠️ 关节角度在 joint.axis1.position（rad），不是 joint.axis1
        #    （joint.axis1 是 Axis 子消息；axis1.velocity / force 实测未被填充，不可用作反馈）
        with self.lock:
            self._joint_seen = True
            joint = None
            for j in msg.joint:
                if j.name == 'joint1':
                    joint = j
                    break
            if joint is None and len(msg.joint) > 0:
                joint = msg.joint[0]
            if joint is not None:
                self.actual_deg = deg(joint.axis1.position)

    def _on_pose(self, msg):
        # ⚠️ 落地判据用【机身高度】，不用 contact 话题：
        #    robot_world.sdf 里虽然声明了 gz-sim-contact-system，
        #    但 /world/default/contact 话题实测【不发布】（订阅成功、9 秒 0 条消息）。
        #    静止时机身 z = 0.021 m，起飞后迅速增大，用高度判据天然分离。
        z = None
        for p in msg.pose:
            if p.name == MODEL_NAME:
                z = p.position.z
                break
        if z is None:
            return
        with self.lock:
            self._pose_seen = True
            self.body_z = z
            self.on_ground = z < ON_GROUND_Z

    # ---------------- 指令斜坡（把"运动时间"和 PID 增益解耦） ----------------

    def _ramp_loop(self):
        """以 ARM_RAMP_HZ 的频率持续推进指令，避免阶跃激起积分饱和。"""
        dt = 1.0 / ARM_RAMP_HZ
        while True:
            time.sleep(dt)
            try:
                self.apply(verbose=False)
            except Exception:
                pass

    # ---------------- 核心：计算并下发指令 ----------------

    def apply(self, verbose=True):
        with self.lock:
            arm = self.arm_deg
            ground = self.on_ground
            flight = self.flight_on
            wheel = self.wheel_on
            pspeed = self.prop_speed

        # ---- 规则 5：安全联锁 ----
        if arm > MODE_THRESH and not ground:
            if verbose and not self._warned:
                print(f"\n⛔ 安全联锁触发：机身离地中，禁止超过 {MODE_THRESH:.0f}°"
                      f"（当前指令 {arm:.1f}°，机身 z = {self.body_z:.3f} m）"
                      f" → 已回退到 {GROUND_HOLD_DEG:.0f}°")
            self._warned = True
            arm = GROUND_HOLD_DEG
            with self.lock:
                self.arm_deg = arm
        else:
            self._warned = False

        # ---- 螺旋桨 ----
        prop_allowed = arm <= MODE_THRESH
        if PROP_STOP_ON_ANGLE and arm > MODE_THRESH:
            prop_allowed = False
        if PROP_STOP_ON_TRANSIT and arm > 0.5:
            prop_allowed = False
        prop_cmd = pspeed if (prop_allowed and flight) else 0.0

        # ---- 轮子 ----
        wheel_allowed = arm > MODE_THRESH
        wheel_cmd = WHEEL_SPEED if (wheel_allowed and wheel) else 0.0

        # （互斥是结构保证的：prop_allowed 与 wheel_allowed 不可能同时为真）

        # ---- 下发：角度走斜坡，避免阶跃指令激起积分饱和 ----
        now = time.time()
        dt = min(max(now - self._ramp_t, 0.0), 0.2)
        self._ramp_t = now
        step = ARM_RATE_DEG_S * dt
        with self.lock:
            if self.arm_cmd < arm:
                self.arm_cmd = min(self.arm_cmd + step, arm)
            elif self.arm_cmd > arm:
                self.arm_cmd = max(self.arm_cmd - step, arm)
            cmd = self.arm_cmd

        self.pub_arm.publish(Double(data=math.radians(cmd)))
        self.pub_wheel.publish(Double(data=wheel_cmd))

        # 螺旋桨：4 个推力模型共用一条话题，靠 velocity 数组下标寻址
        #   （下标在 URDF 里由 actuator_number 指定：0=前左 1=前右 2=后左 3=后右）
        mk = Actuators()
        mk.velocity.extend([prop_cmd] * 4)
        self.pub_motor.publish(mk)

        return cmd, prop_cmd, wheel_cmd, prop_allowed, wheel_allowed

    # ---------------- 状态显示 ----------------

    def status(self, arm, prop_cmd, wheel_cmd, prop_allowed, wheel_allowed):
        with self.lock:
            actual = self.actual_deg
            ground = self.on_ground
            body_z = self.body_z
            flight = self.flight_on
            wheel = self.wheel_on
            jseen = self._joint_seen
            pseen = self._pose_seen

        mode = '🛫 飞行区' if arm <= MODE_THRESH else '🛞 地面区'
        print("\n" + "-" * 62)
        print(f"  模式        : {mode}   (分界 {MODE_THRESH:.0f}°)")
        print(f"  舵机指令    : {arm:6.2f}°     实测: {actual:6.2f}°"
              f"{'' if jseen else '   ⚠️ 未收到关节状态'}")
        print(f"  落地检测    : {'✅ 已落地' if ground else '❌ 空中'}"
              f"   (机身 z = {body_z:.3f} m，判据 < {ON_GROUND_Z:.2f})"
              f"{'' if pseen else '   ⚠️ 未收到位姿消息'}")
        print(f"  螺旋桨      : {'开' if flight else '关'}（手动）   "
              f"本周期指令 = {prop_cmd:7.1f} rad/s"
              f"   [{'允许' if prop_allowed else '禁止'}]")
        print(f"  轮子        : {'开' if wheel else '关'}（手动）   "
              f"本周期指令 = {wheel_cmd:7.1f} rad/s"
              f"   [{'允许' if wheel_allowed else '禁止'}]")
        print("-" * 62)

    # ---------------- 命令帮助 ----------------

    @staticmethod
    def help():
        print("""
================= 命令 =================
  +  /  -        舵机角度 +2° / -2°（两舵机同步转动）
  <数字>         直接设定舵机角度，例如 48 或 52
  f              切换【螺旋桨】开关（飞行区才生效）
  w              切换【轮子】开关（地面区才生效）
  p <转速>       设定螺旋桨转速 rad/s（悬停 1963，满油 3400）
  s              显示当前状态
  h              显示本帮助
  q              退出
========================================
说明：轮子与螺旋桨互斥，代码结构上不可能同时工作。
      机身离地时把角度设到 48° 以上会被自动拦下。
      ⚠ 目前还没接姿态控制器，桨一开机器人会离地后翻覆；
        稳定悬停需要 MulticopterControl（姿态/速度控制器）。
""")


def main():
    print("正在连接 Gazebo ...")
    ctrl = AirGroundController()
    time.sleep(0.5)
    print("已连接。话题：")
    print(f"  舵机指令 → {ARM_TOPIC}   (Gazebo 侧：joint1 + joint2 同时订阅)")
    print(f"  轮子指令 → {WHEEL_TOPIC} (Gazebo 侧：joint7~joint10 同时订阅)")
    print(f"  桨指令   → {MOTOR_TOPIC} (Gazebo 侧：4 个推力模型按 actuator_number 取值)")
    ctrl.help()

    # 初始指令
    ctrl.apply()

    while True:
        try:
            raw = input("命令> ").strip()
        except (EOFError, KeyboardInterrupt):
            print("\n退出。")
            break

        if raw == '':
            continue

        if raw == 'q':
            with ctrl.lock:
                ctrl.arm_deg = 0.0
                ctrl.flight_on = False
                ctrl.wheel_on = False
            ctrl.apply()
            print("已停止所有执行器并退出。")
            break

        elif raw == 'h':
            ctrl.help()
            continue

        elif raw == '+':
            with ctrl.lock:
                ctrl.arm_deg = min(ctrl.arm_deg + ARM_STEP, MAX_ARM_DEG)

        elif raw == '-':
            with ctrl.lock:
                ctrl.arm_deg = max(ctrl.arm_deg - ARM_STEP, 0.0)

        elif raw == 'f':
            with ctrl.lock:
                ctrl.flight_on = not ctrl.flight_on

        elif raw == 'w':
            with ctrl.lock:
                ctrl.wheel_on = not ctrl.wheel_on

        elif raw.startswith('p '):
            try:
                v = float(raw[2:])
                with ctrl.lock:
                    ctrl.prop_speed = max(0.0, min(v, 3400.0))
                print(f"螺旋桨转速 → {ctrl.prop_speed:.0f} rad/s"
                      f"   (悬停 1963 / 满油 3400)")
            except ValueError:
                print("用法：p <rad/s>   例如 p 1963")

        elif raw == 's':
            pass  # 下面统一显示状态

        else:
            try:
                v = float(raw)
                with ctrl.lock:
                    ctrl.arm_deg = max(0.0, min(v, MAX_ARM_DEG))
            except ValueError:
                print(f"未知命令: {raw}   （输入 h 查看帮助）")
                continue

        result = ctrl.apply()
        ctrl.status(*result)


if __name__ == '__main__':
    main()
