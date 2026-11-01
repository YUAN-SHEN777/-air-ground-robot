#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
test_mode_switch.py —— 9/10 路舵机分离 + 模式切换 SITL 测试

验证：
  9 路（SIM_GZ_SV_FUNC1=301，/servo_0）只改摆臂角；
  10 路（SIM_GZ_SV_FUNC2=302，/servo_1）只改模式切换：
      45° = 空中，135° = 地面；
  AirGroundMux 按 10 路角度带滞回切 AIR/GROUND。

优先用 187 + param7=1/2 验证 FunctionActuatorSet 的单通道寻址；
若未 prearmed，再用 310（param5=1301/1302）做同样的通道验证。
"""

import atexit
import math
import os
import subprocess
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")

from pymavlink import mavutil
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node

import tilt_yaw_px4 as T

ARM_TOPIC = "/model/airground_0/servo_0"
MODE_TOPIC = "/model/airground_0/servo_1"
JOINT_TOPIC = "/airground/joint_state"


class GzState:
    def __init__(self):
        self.arm_cmd = None
        self.mode_cmd = None
        self.joints = {}
        self.node = Node()
        self.node.subscribe(Double, ARM_TOPIC, self._on_arm)
        self.node.subscribe(Double, MODE_TOPIC, self._on_mode)
        self.node.subscribe(Model, JOINT_TOPIC, self._on_joint)

    def _on_arm(self, msg):
        self.arm_cmd = math.degrees(msg.data)

    def _on_mode(self, msg):
        self.mode_cmd = math.degrees(msg.data)

    def _on_joint(self, msg):
        for j in msg.joint:
            if j.name in ("servo_0", "servo_1"):
                self.joints[j.name] = math.degrees(j.axis1.position)

    def arm_joint(self):
        a, b = self.joints.get("servo_0"), self.joints.get("servo_1")
        if a is None or b is None:
            return float("nan")
        return 0.5 * (a + b)


def send_loop(conn, secs, arm=None, mode=None, use_310=False):
    """周期发心跳/中立摇杆 + 187(param7=1/2) 或 310(param5=1301/1302)。"""
    t0 = time.time()
    while time.time() - t0 < secs:
        conn.mav.heartbeat_send(
            mavutil.mavlink.MAV_TYPE_GCS,
            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        # 中立摇杆：避免 commander 认为 manual control 丢失
        conn.mav.manual_control_send(1, 0, 0, 500, 0, 0)

        if arm is not None:
            if use_310:
                conn.mav.command_long_send(
                    1, 1, 310, 0, arm, 1.0, 0, 0, 1301, 0, 0)
            else:
                conn.mav.command_long_send(
                    1, 1, 187, 0, arm, 0, 0, 0, 0, 0, 1)

        if mode is not None:
            if use_310:
                conn.mav.command_long_send(
                    1, 1, 310, 0, mode, 1.0, 0, 0, 1302, 0, 0)
            else:
                conn.mav.command_long_send(
                    1, 1, 187, 0, mode, 0, 0, 0, 0, 0, 2)

        time.sleep(0.05)


def show(tag, gz, logs):
    print(f"\n[{tag}]")
    print(f"  servo_0 指令 = {gz.arm_cmd!s:>8}°   "
          f"servo_1 指令 = {gz.mode_cmd!s:>8}°   "
          f"两摆臂关节 = {gz.arm_joint():.2f}°")
    if logs:
        lines = open(T.LOG, errors="ignore").read().splitlines()
        mode_lines = [ln for ln in lines if "AirGroundMux] mode ->" in ln]
        if mode_lines:
            print("  mux:", mode_lines[-1].strip())


def main():
    T.cleanup()
    if not T.boot():
        return 1

    atexit.register(T.cleanup)

    conn = mavutil.mavlink_connection(
        "udpout:127.0.0.1:18570", source_system=255, source_component=0)
    gz = GzState()

    # 让 187 在未解锁时也能生效（mixer_module 需要 armed || prearmed）
    conn.param_set_send("COM_PREARM_MODE", 2,
                        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(7)
    send_loop(conn, 1.0, arm=-1.0)

    # ① 先建立空中位：9 路 0°，10 路 45°
    send_loop(conn, 3.0, arm=-1.0, mode=-1.0)
    show("初始：arm 0° / mode 45°（空中）", gz, True)

    # ② 10 路 → 135°，应切地面；9 路保持 0°
    send_loop(conn, 3.0, arm=-1.0, mode=1.0)
    show("mode 135°（地面）", gz, True)

    # ③ 10 路 → 45°，应切回空中
    send_loop(conn, 3.0, arm=-1.0, mode=-1.0)
    show("mode 45°（空中）", gz, True)

    # ④ 只动 9 路：52°，10 路应保持 45°
    send_loop(conn, 3.0, arm=1.0, mode=-1.0)
    show("arm 52° / mode 45°（验证 9 路独立）", gz, True)

    # ⑤ 9 路回 0°，10 路仍 45°
    send_loop(conn, 2.0, arm=-1.0, mode=-1.0)
    show("arm 0° / mode 45°", gz, True)

    # ⑥ 再用 310 验证通道（若 187 未生效，这一步一定生效）
    send_loop(conn, 3.0, arm=-1.0, mode=1.0, use_310=True)
    show("310: mode 135°（地面）", gz, True)
    send_loop(conn, 3.0, arm=-1.0, mode=-1.0, use_310=True)
    show("310: mode 45°（空中）", gz, True)

    # 收尾：打印 mux 的模式切换日志
    lines = open(T.LOG, errors="ignore").read().splitlines()
    print("\n=== AirGroundMux 模式日志 ===")
    for ln in lines:
        if "AirGroundMux] mode ->" in ln:
            print(ln.strip())

    T.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
