#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tilt_ab_px4.py —— 0° vs 25° 交叉 A/B 验证
==========================================
目的：确认上一轮"θ=0° 没有偏航权限"到底是真实现象，还是脉冲对齐错位
      造成的假象。

设计要点（怎样都不可能搞错）：
  · 只测两个角度：0° 与 25°，【交替】重复 3 轮
      0° → 25° → 0° → 25° → 0° → 25°
  · 判定量直接取【Gazebo 位姿真值】的峰值偏航角速度，
    墙钟时间与指令窗口完全同一时基，不涉及 ulog 对齐
  · 每轮之间留 5 s 稳态，避免上一档的余速串进来

用法： python3 tilt_ab_px4.py
"""
import math
import time

import tilt_yaw_px4 as T

AB = [0, 25, 0, 25, 0, 25]
SETTLE = 5.0
YAW_SEC = 2.0
HOLD = 2.0


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

    print("\n⑤ 切 Position（等 nav_state=22）")
    for _ in range(10):
        link.cmd(176, 1.0, 3.0, 0.0)
        if link.wait_nav(T.NAV_POSCTL, timeout=6.0):
            print("   ✓ nav_state = %d" % link.nav)
            break
    link.run(3.0, "稳定", arm_deg=0.0)

    print("\n⑥ 探针：确认摇杆生效")
    for k in range(6):
        link.run(1.0, "悬停", arm_deg=0.0, yaw=0.0)
        n0 = len(g.rec)
        link.run(1.5, "探针右偏", arm_deg=0.0, yaw=+T.STICK)
        link.run(1.5, "回收", arm_deg=0.0, yaw=0.0)
        pr = T.peak_rate([(r[0], r[1]) for r in g.rec[n0:]])
        print("   第 %d 次：%.0f °/s" % (k + 1, pr))
        if pr > 40:
            print("   ✓ 生效")
            break
    else:
        print("   ⚠️ 探针一直无响应")

    print("\n⑦ 0° / 25° 交替 A/B（各 3 次）")
    data = {0: [], 25: []}
    for rep, deg in enumerate(AB, 1):
        link.run(SETTLE, f"摆臂→{deg}°", arm_deg=deg, yaw=0.0)
        real = g.arm_deg()
        vals = []
        for label, sgn in (("右", +1), ("左", -1)):
            link.run(HOLD, "稳定", arm_deg=deg, yaw=0.0)
            n0 = len(g.rec)
            link.run(YAW_SEC, f"{deg}°-{label}", arm_deg=deg, yaw=sgn * T.STICK)
            vals.append(T.peak_rate([(r[0], r[1]) for r in g.rec[n0:]]))
        data[deg].extend(vals)
        print("   [第%d轮] θ=%2d°(实测%+5.2f°)  右 %5.0f °/s   左 %5.0f °/s"
              % (rep, deg, real, vals[0], vals[1]))

    print("\n⑧ 回 0° 并降落")
    link.run(3.0, "回 0°", arm_deg=0.0, yaw=0.0)
    link.cmd(176, 1.0, 4.0, 6.0)
    link.run(6.0, "降落", arm_deg=0.0, yaw=0.0)
    link.cmd(400, 0.0)
    link.run(1.0, "上锁", arm_deg=0.0, yaw=0.0)

    print("\n" + "=" * 62)
    for deg in (0, 25):
        v = data[deg]
        if v:
            print("%3d°  样本 %s   均值 %5.0f °/s   最大 %5.0f °/s"
                  % (deg, " ".join("%.0f" % x for x in v),
                     sum(v) / len(v), max(v)))
    print("=" * 62)
    a0 = sum(data[0]) / len(data[0]) if data[0] else 0.0
    a25 = sum(data[25]) / len(data[25]) if data[25] else 0.0
    print("25°/0° 比值 = %.2f" % (a25 / a0 if a0 > 1e-6 else float("nan")))
    return 0


if __name__ == "__main__":
    try:
        rc = main()
    finally:
        print("\n⑨ 清理")
        T.cleanup()
