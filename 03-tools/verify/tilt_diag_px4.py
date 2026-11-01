#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tilt_diag_px4.py —— 诊断：为什么 0° 的角加速度测出来是 0
=========================================================
只做一件事：把 0° 和 15° 偏航脉冲的【原始偏航角时间曲线】打印出来，
每 0.1 s 一个点，看 0° 到底是"根本没有力矩"还是"响应延迟"。

同时打印：
  · PX4 的偏航角速度指令（vehicle_rates_setpoint）—— 从 ulog 取不到，
    所以这里用 gz 位姿差分做参考
  · 每 0.1 s 的瞬时角速度

用法： python3 tilt_diag_px4.py
"""
import time

import tilt_yaw_px4 as T

CASES = [0, 15, 0, 15]


def dump(samples, t0, label):
    s = [(t - t0, y) for t, y in samples if t >= t0]
    ys = T.unwrap([y for _, y in s])
    print("   %-10s  t(s) :" % label, end="")
    ts = []
    t = 0.0
    while t <= 2.4:
        ts.append(t)
        t += 0.2
    for t in ts:
        print(" %5.1f" % t, end="")
    print()
    print("   %-10s  yaw°:" % "", end="")
    for t in ts:
        # 取最接近的样本
        if not s:
            print("   ---", end="")
            continue
        j = min(range(len(s)), key=lambda i: abs(s[i][0] - t))
        print(" %5.1f" % ys[j], end="")
    print()
    print("   %-10s  ω°/s:" % "", end="")
    for t in ts:
        if len(s) < 3:
            print("   ---", end="")
            continue
        a = max(0, min(len(ys) - 1, int(t / max(s[-1][0], 1e-6) * (len(ys) - 1))))
        b = min(len(ys) - 1, a + 2)
        dt = s[b][0] - s[a][0]
        print(" %5.0f" % ((ys[b] - ys[a]) / dt if dt > 1e-6 else 0.0), end="")
    print("\n")


def main():
    T.cleanup()
    if not T.boot():
        return 1
    g = T.Gimbal()
    link = T.Link()
    t0 = time.time()
    while time.time() - t0 < 12 and link.armed is None:
        link.send(T.mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
        link.drain()
        time.sleep(0.2)
    if link.armed is None:
        print("❌ 无心跳")
        return 1

    print("\n起飞")
    link.run(2.0, "预置 0°", arm_deg=0.0)
    link.cmd(176, 1.0, 4.0, 2.0)
    link.run(1.0, "Takeoff", arm_deg=0.0)
    link.cmd(400, 1.0)
    link.run(1.5, "ARM", arm_deg=0.0)
    link.run(11.0, "爬升", arm_deg=0.0)
    for _ in range(10):
        link.cmd(176, 1.0, 3.0, 0.0)
        if link.wait_nav(T.NAV_POSCTL, timeout=6.0):
            break
    link.run(3.0, "稳定", arm_deg=0.0)

    for k, deg in enumerate(CASES, 1):
        link.run(3.5, "", arm_deg=deg, yaw=0.0)
        link.run(2.5, "", arm_deg=deg, yaw=0.0)
        print("=== 第%d次  θ=%d°（实测 %.2f°）===" % (k, deg, g.arm_deg()))
        n0 = len(g.rec)
        t_start = time.time()
        link.run(1.2, "", arm_deg=deg, yaw=+T.STICK)
        link.run(1.2, "", arm_deg=deg, yaw=0.0)
        dump(g.rec[n0:], t_start, "θ=%d°" % deg)

    print("\n回 0° 并降落")
    link.run(3.0, "", arm_deg=0.0, yaw=0.0)
    link.cmd(176, 1.0, 4.0, 6.0)
    link.run(6.0, "", arm_deg=0.0, yaw=0.0)
    link.cmd(400, 0.0)
    link.run(1.0, "", arm_deg=0.0, yaw=0.0)
    return 0


if __name__ == "__main__":
    try:
        main()
    finally:
        print("\n清理")
        T.cleanup()
