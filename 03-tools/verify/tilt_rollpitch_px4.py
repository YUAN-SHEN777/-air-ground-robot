#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""tilt_rollpitch_px4.py —— 摆臂角 0° vs 20° 的横滚/俯仰权限对比

目的：给报告 §5.5.5 的“20° 时横滚/俯仰差动效率下降”提供实测依据。

方法：
  · 复用 tilt_yaw_px4 的 SITL 启动、起飞、Position 悬停；
  · 用 Gazebo 位姿真值记录 roll/pitch，施加 ±0.8 的横滚/俯仰杆量 1.2 s；
  · 以峰值角速度作为权限指标，交替测 0° 与 20°（0,20,0,20,0），
    每档左右各一次，最后取均值。

用法：
    python3 tilt_rollpitch_px4.py
"""

import math
import os
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")
os.environ.setdefault("HEADLESS", "1")

import tilt_yaw_px4 as T

from gz.msgs10.actuators_pb2 import Actuators
from gz.transport13 import Node

AB = [0, 20, 0, 20, 0]
SETTLE = 4.0
PULSE = 1.2
RECOVER = 1.5
STICK = 0.8


class Motors:
    """记录 AirGroundMux 输出的 4 路 prop_speed。"""

    def __init__(self):
        self.rec = []              # (t, [m0,m1,m2,m3])
        self.node = Node()
        self.node.subscribe(Actuators, "/airground_0/command/prop_speed", self._on)

    def _on(self, msg):
        v = list(msg.velocity)
        if len(v) >= 4:
            self.rec.append((time.time(), v[:4]))


class Attitude(T.Gimbal):
    """在 Gimbal 的 yaw 记录之外，额外记录 roll/pitch。"""

    def __init__(self):
        self.att = []              # (t, roll_deg, pitch_deg)
        super().__init__()

    def _on_pose(self, msg):
        for p in msg.pose:
            if p.name != T.MODEL:
                continue
            q = p.orientation
            # ZYX Euler
            roll = math.degrees(math.atan2(
                2.0 * (q.w * q.x + q.y * q.z),
                1.0 - 2.0 * (q.x * q.x + q.y * q.y)))
            pitch = math.degrees(math.asin(max(-1.0, min(1.0,
                2.0 * (q.w * q.y - q.z * q.x)))))
            yaw = math.degrees(math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
            t = time.time()
            self.rec.append((t, yaw))
            self.att.append((t, roll, pitch))


def run(link, secs, label="", arm_deg=0.0, **kw):
    """与 tilt_yaw_px4.Link.run 相同，但 187 使用 param7=1（只写 9 路）。"""
    t0 = time.time()
    while time.time() - t0 < secs:
        link.send(T.mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
        link.cmd(187, T.arm_param(arm_deg), 0, 0, 0, 0, 0, 1)
        link.manual(**kw)
        link.drain()
        time.sleep(0.02)
    if label:
        print("   %-20s %4.1fs  roll=%+.2f pitch=%+.2f nav=%s"
              % (label, secs, kw.get("roll", 0.0), kw.get("pitch", 0.0), link.nav))


def peak_abs_rate(samples, axis):
    """samples: [(t, roll, pitch)]；axis: 1=roll, 2=pitch；返回峰值 °/s。"""
    if len(samples) < 8:
        return 0.0
    vals = [(s[0], s[axis]) for s in samples]
    vals.sort(key=lambda x: x[0])
    rates = []
    for i in range(1, len(vals)):
        dt = vals[i][0] - vals[i - 1][0]
        if dt > 1e-6:
            rates.append(abs((vals[i][1] - vals[i - 1][1]) / dt))
    return max(rates) if rates else 0.0


def diff_metrics(samples, key):
    """samples: [(t, [m0..m3])]；返回 (峰值差动, 公共推力均值)。

    prop_speed 是 mux 解码后的 0..1000；m0/m1 为前左/前右，
    m2/m3 为后左/后右。
      roll  D = (m0+m2) - (m1+m3)
      pitch D = (m0+m1) - (m2+m3)
    20° 时悬停推力更大，因此最终比较 D / mean（归一化差动）。
    """
    if not samples:
        return 0.0, 0.0
    ds = []
    means = []
    for _, v in samples:
        if key == "roll":
            d = (v[0] + v[2]) - (v[1] + v[3])
        else:
            d = (v[0] + v[1]) - (v[2] + v[3])
        ds.append(abs(d))
        means.append(sum(v) / 4.0)
    return (max(ds) if ds else 0.0,
            sum(means) / len(means) if means else 0.0)


def main():
    T.cleanup()
    if not T.boot():
        print("❌ PX4 SITL 未就绪")
        return 1

    g = Attitude()
    motors = Motors()
    link = T.Link()
    t0 = time.time()
    while time.time() - t0 < 12 and link.armed is None:
        link.send(T.mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
        link.drain()
        time.sleep(0.2)
    if link.armed is None:
        print("❌ 收不到 PX4 心跳")
        return 1

    print("\n④ 起飞（摆臂 0°）")
    run(link, 2.0, "预置 0°", arm_deg=0.0)
    link.cmd(176, 1.0, 4.0, 2.0)          # AUTO_TAKEOFF
    run(link, 1.0, "切 Takeoff", arm_deg=0.0)
    link.cmd(400, 1.0)                    # ARM
    run(link, 1.5, "解锁", arm_deg=0.0)
    run(link, 11.0, "自动起飞", arm_deg=0.0)

    print("\n⑤ 切 Position 并稳定")
    for _ in range(10):
        link.cmd(176, 1.0, 3.0, 0.0)      # POSCTL
        if link.wait_nav(T.NAV_POSCTL, timeout=6.0):
            print("   ✓ nav_state = %d (POSCTL)" % link.nav)
            break
    run(link, 3.0, "稳定", arm_deg=0.0)

    data = {0: {"roll": [], "pitch": [], "roll_D": [], "pitch_D": [],
               "roll_M": [], "pitch_M": []},
            20: {"roll": [], "pitch": [], "roll_D": [], "pitch_D": [],
                "roll_M": [], "pitch_M": []}}

    print("\n⑥ 0° / 20° 交替；每个角度测 roll±、pitch±")
    for rep, deg in enumerate(AB, 1):
        run(link, SETTLE, "摆臂→%d°" % deg, arm_deg=deg, roll=0.0, pitch=0.0)
        real = g.arm_deg()
        print("   ── 第 %d 轮  θ=%d°（实测 %+.2f°）" % (rep, deg, real))

        for axis, key in ((1, "roll"), (2, "pitch")):
            for sgn in (+1, -1):
                run(link, RECOVER, "回中", arm_deg=deg, roll=0.0, pitch=0.0)
                n0 = len(g.att)
                n0m = len(motors.rec)
                kw = {key: sgn * STICK}
                run(link, PULSE, "%s%+d" % (key, sgn), arm_deg=deg, **kw)
                run(link, RECOVER, "回收", arm_deg=deg, roll=0.0, pitch=0.0)
                pr = peak_abs_rate(g.att[n0:], axis)
                d, common = diff_metrics(motors.rec[n0m:], key)
                data[deg][key].append(pr)
                data[deg][key + "_D"].append(d)
                data[deg][key + "_M"].append(common)
                print("      %s %+d  峰值 %6.1f °/s  差动 %7.1f  mean %6.1f  归一 %6.3f"
                      % (key, sgn, pr, d, common, d / max(common, 1e-9)))

    print("\n⑦ 回 0° 并降落")
    run(link, 3.0, "回 0°", arm_deg=0.0, roll=0.0, pitch=0.0)
    link.cmd(176, 1.0, 4.0, 6.0)          # AUTO_LAND
    run(link, 6.0, "降落", arm_deg=0.0, roll=0.0, pitch=0.0)
    link.cmd(400, 0.0)                    # DISARM
    run(link, 1.0, "上锁", arm_deg=0.0, roll=0.0, pitch=0.0)

    print("\n" + "=" * 64)
    for key in ("roll", "pitch"):
        a = data[0][key]
        b = data[20][key]
        ma = sum(a) / len(a) if a else float("nan")
        mb = sum(b) / len(b) if b else float("nan")
        drop = (1.0 - mb / ma) * 100.0 if ma > 1e-6 else float("nan")
        print("%-5s  0° 均值 %6.1f °/s  20° 均值 %6.1f °/s  下降 %5.1f%%"
              % (key, ma, mb, drop))
        print("       0° 样本: %s" % " ".join("%.1f" % x for x in a))
        print("      20° 样本: %s" % " ".join("%.1f" % x for x in b))

        da = data[0][key + "_D"]
        db = data[20][key + "_D"]
        ma = data[0][key + "_M"]
        mb = data[20][key + "_M"]
        mda = sum(da) / len(da) if da else float("nan")
        mdb = sum(db) / len(db) if db else float("nan")
        mma = sum(ma) / len(ma) if ma else float("nan")
        mmb = sum(mb) / len(mb) if mb else float("nan")
        n0 = mda / mma if mma > 1e-9 else float("nan")
        n20 = mdb / mmb if mmb > 1e-9 else float("nan")
        d_drop = (1.0 - n0 / n20) * 100.0 if n20 > 1e-9 else float("nan")
        print("       差动：0° %.1f (mean %.1f, norm %.3f)  20° %.1f (mean %.1f, norm %.3f)"
              % (mda, mma, n0, mdb, mmb, n20))
        print("       归一化差动变化：效率下降 %5.1f%%" % d_drop)
    print("=" * 64)
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    finally:
        print("\n⑧ 清理")
        T.cleanup()
