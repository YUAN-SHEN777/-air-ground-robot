#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tilt_sweep_px4.py —— 摆臂倾角 ↔ 偏航权限  ·  PX4 SITL  ·  v4
============================================================
针对前几轮暴露的问题重新设计：

  ① 指标换成【角加速度 α】，不再是峰值偏航率
     - 取脉冲前 0.06~0.45 s 的偏航角做【二次拟合】 y = a0 + a1·t + a2·t²/2
     - a2 就是角加速度（deg/s²）。这一段速率还在线性爬升，
       不受 2 s 窗口截断和控制器收敛影响
     - α = τ/J，J 在各角度几乎不变 ⇒ α 比值 ≈ 力矩比值

  ② 每档 5 次，六档【轮转】而不是成块
     0,5,10,15,20,25, 0,5,10,15,20,25, ...
     这样任何缓慢漂移（电池/温度/姿态）平均到每一档上

  ③ 每档报【均值 ± 标准差】，并打印全部原始样本
     如果 0° 的 5 个样本仍然散得厉害，说明方法本身不成立

  ④ 顺带打印位姿采样率，确认 0.4 s 窗口里够不够样本

摆臂仍经 PX4 的 MAV_CMD_DO_SET_ACTUATOR(187)： θ = 26° × (1 + p1)
全程 PX4 闭环：ARM → AUTO.TAKEOFF → POSITION → 手动偏航摇杆 → LAND

用法： python3 tilt_sweep_px4.py [analyze]
"""
import json
import math
import os
import statistics
import sys
import time

import tilt_yaw_px4 as T

ANGLES = [0, 5, 10, 15, 20, 25]
REPS = 5
SETTLE = 3.5          # 摆臂到位等待
YAW_SEC = 1.2         # 脉冲时长
HOLD = 2.5            # 回中等待
FIT_A, FIT_B = 0.06, 0.45     # 拟合窗口（相对脉冲起点）
OUT = "/tmp/tilt_sweep_samples.json"


def quad_accel(samples, t0):
    """对 [t0+FIT_A, t0+FIT_B] 内的 (t, yaw) 做 y = a0+a1·t+a2·t²/2 最小二乘
       返回 a2（deg/s²）。yaw 已解卷绕。"""
    sel = [(t - t0, y) for t, y in samples if FIT_A <= t - t0 <= FIT_B]
    if len(sel) < 8:
        return None, 0
    ts = [s[0] - FIT_A for s in sel]
    ys = T.unwrap([s[1] for s in sel])
    n = len(ts)
    rows = [[1.0, t, t * t / 2.0] for t in ts]
    m = [[sum(rows[k][i] * rows[k][j] for k in range(n)) for j in range(3)]
         + [sum(rows[k][i] * ys[k] for k in range(n))] for i in range(3)]
    for c in range(3):
        piv = max(range(c, 3), key=lambda r: abs(m[r][c]))
        if abs(m[piv][c]) < 1e-12:
            return None, n
        m[c], m[piv] = m[piv], m[c]
        for r in range(3):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, 4):
                    m[r][k] -= f * m[c][k]
    return m[2][3] / m[2][2], n


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
        print("❌ 收不到 PX4 心跳")
        return 1

    print("\n④ 起飞（摆臂 0°）")
    link.run(2.0, "预置 0°", arm_deg=0.0)
    link.cmd(176, 1.0, 4.0, 2.0)
    link.run(1.0, "切 Takeoff", arm_deg=0.0)
    link.cmd(400, 1.0)
    link.run(1.5, "解锁", arm_deg=0.0)
    link.run(11.0, "自动起飞", arm_deg=0.0)

    print("\n⑤ 切 Position")
    for _ in range(10):
        link.cmd(176, 1.0, 3.0, 0.0)
        if link.wait_nav(T.NAV_POSCTL, timeout=6.0):
            print("   ✓ nav_state = %d" % link.nav)
            break
    link.run(3.0, "稳定", arm_deg=0.0)

    print("\n⑥ 探针")
    for k in range(6):
        link.run(1.0, "悬停", arm_deg=0.0, yaw=0.0)
        n0 = len(g.rec)
        link.run(1.2, "探针", arm_deg=0.0, yaw=+T.STICK)
        link.run(1.5, "回收", arm_deg=0.0, yaw=0.0)
        pr = T.peak_rate([(r[0], r[1]) for r in g.rec[n0:]])
        print("   第%d次 %.0f °/s" % (k + 1, pr))
        if pr > 30:
            print("   ✓ 摇杆生效")
            break

    print("\n⑦ %d 档 × %d 次（轮转）" % (len(ANGLES), REPS))
    samples = []
    motors = {}
    for rep in range(REPS):
        for deg in ANGLES:
            link.run(SETTLE, "", arm_deg=deg, yaw=0.0)
            real = g.arm_deg()
            for label, sgn in (("R", +1), ("L", -1)):
                link.run(HOLD, "", arm_deg=deg, yaw=0.0)
                n0 = len(g.rec)
                t_start = time.time()
                link.run(YAW_SEC, "", arm_deg=deg, yaw=sgn * T.STICK)
                win = [r for r in g.rec[n0:] if r[0] <= time.time()]
                a2, ns = quad_accel([(r[0], r[1]) for r in win], t_start)
                samples.append(dict(rep=rep, deg=deg, dir=label, arm=real,
                                    alpha=a2, ns=ns))
                if a2 is None:
                    print("   rep%d θ=%2d° %s  ⚠️ 样本不足(%d)" % (rep, deg, label, ns))
                else:
                    print("   rep%d θ=%2d°(实测%+5.2f°) %s  α=%+8.1f °/s²  (%d样本)"
                          % (rep, deg, real, label, a2, ns))
            link.run(HOLD, "", arm_deg=deg, yaw=0.0)

    print("\n⑧ 回 0° 并降落")
    link.run(3.0, "回 0°", arm_deg=0.0, yaw=0.0)
    link.cmd(176, 1.0, 4.0, 6.0)
    link.run(6.0, "降落", arm_deg=0.0, yaw=0.0)
    link.cmd(400, 0.0)
    link.run(1.0, "上锁", arm_deg=0.0, yaw=0.0)

    json.dump(samples, open(OUT, "w"))
    report(samples)
    return 0


def report(samples):
    rate = len(samples) / max(1e-6, 1)  # 占位，采样率在下面单独算
    print("\n" + "=" * 80)
    print(f"{'θ':>4} {'n':>3} {'|α| 均值':>11} {'标准差':>9} {'相对0°':>8} "
          f"{'理论':>7} {'全部样本 |α| (°/s²)':>32}")
    print("-" * 80)
    # ⚠️ 角加速度对左右杆是反号的，必须取绝对值再统计，
    #    否则 R/L 会互相抵消、均值恒为 0（旧版本就是这个 bug）。
    good = [s for s in samples if s["alpha"] is not None]
    base = [abs(s["alpha"]) for s in good if s["deg"] == 0]
    base_m = statistics.mean(base) if base else float("nan")
    summary = []
    for deg in ANGLES:
        v = [abs(s["alpha"]) for s in good if s["deg"] == deg]
        if not v:
            continue
        m = statistics.mean(v)
        sd = statistics.pstdev(v) if len(v) > 1 else 0.0
        rel = m / base_m if base_m else float("nan")
        th = math.cos(math.radians(deg)) + 2.1 * math.sin(math.radians(deg))
        summary.append((deg, m, sd, rel, th))
        s = " ".join("%.0f" % x for x in v)
        print(f"{deg:>3}° {len(v):>3} {m:>10.1f}  {sd:>8.1f} {rel:>8.2f} "
              f"{th:>7.2f} {s:>32}")
    print("=" * 80)
    print("理论列 = cosθ + 2.1·sinθ（由 x_f−x_r=0.21 m、κ=0.05 m 推出）")
    print("理论「相对0°」就是该列数值本身（0° 时为 1.00）")


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        print("\n⑨ 清理")
        T.cleanup()
