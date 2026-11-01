#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
rev_probe.py —— 对拍「PX4 电机指令 → mux 解码」这条链路
================================================================
目的：查清 CA_R_REV 为什么会改变摆臂 0° 的偏航权限。

做法：同时订阅两个话题，看同一条指令在两端的数值
    /airground_0/command/motor_speed   ← PX4 控制分配的输出
    /airground_0/command/prop_speed    ← AirGroundMux 解出的桨转速

分别在「悬停静止」和「打满偏航」两个工况取样，
把 4 路指令的原值、零推力点、可用的上下行程都打出来。

用法：
    python3 rev_probe.py
"""

import os
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")

import tilt_yaw_px4 as T
from pymavlink.dialects.v20 import common as mav_common  # noqa: F401

from gz.transport13 import Node
from gz.msgs10.actuators_pb2 import Actuators

TOPIC_PX4 = "/airground_0/command/motor_speed"
TOPIC_MUX = "/airground_0/command/prop_speed"

STICK = 0.8


class Tap:
    """同时盯住 PX4 输出与 mux 解码结果"""

    def __init__(self):
        self.px4 = None
        self.mux = None
        self.node = Node()
        self.node.subscribe(Actuators, TOPIC_PX4, self._on_px4)
        self.node.subscribe(Actuators, TOPIC_MUX, self._on_mux)

    def _on_px4(self, msg):
        self.px4 = list(msg.velocity) if len(msg.velocity) else list(msg.position)

    def _on_mux(self, msg):
        self.mux = list(msg.velocity) if len(msg.velocity) else list(msg.position)

    def sweep(self, link, secs, **kw):
        """一边持续发指令，一边采样 PX4 输出，返回逐通道 (min, mean, max)"""
        rows = []
        t0 = time.time()
        while time.time() - t0 < secs:
            link.send(mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
            link.cmd(187, T.arm_param(0.0), 0, 0, 0, 0, 0, 0)
            link.manual(**kw)
            link.drain()
            if self.px4:
                rows.append(list(self.px4))
            time.sleep(0.02)
        if not rows:
            return None
        out = []
        for i in range(4):
            col = [r[i] for r in rows]
            out.append((min(col), sum(col) / len(col), max(col)))
        return out


def report(tag, stats):
    if stats is None:
        print(f"   {tag}: 没采到数据")
        return
    print(f"   {tag}")
    print("      通道      最低      均值      最高")
    for i, (lo, av, hi) in enumerate(stats):
        print(f"        {i}   {lo:8.1f}  {av:8.1f}  {hi:8.1f}")
    lo_all = min(s[0] for s in stats)
    print(f"      → 全部通道最低值 {lo_all:.1f}"
          f"（负值说明分配器在下达反向推力）")


def main():
    print("① 清理并启动")
    T.cleanup()
    if not T.boot():
        return 1

    tap = Tap()
    link = T.Link()
    t0 = time.time()
    while time.time() - t0 < 10 and link.armed is None:
        link.send(mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
        link.drain()
        time.sleep(0.2)
    if link.armed is None:
        print("❌ 收不到 PX4 心跳")
        return 1

    print("\n② 起飞（摆臂 0°）")
    link.run(2.0, "预置 0°", arm_deg=0.0)
    link.cmd(176, 1.0, 4.0, 2.0)
    link.run(1.0, "切 Takeoff", arm_deg=0.0)
    link.cmd(400, 1.0)
    link.run(1.5, "解锁", arm_deg=0.0)
    link.run(11.0, "自动起飞", arm_deg=0.0)
    link.run(3.0, "稳定", arm_deg=0.0)

    print("\n③ 切 Position 模式（AUTO 下摇杆无效）")
    ok = False
    for _ in range(3):
        link.cmd(176, 1.0, 3.0, 0.0)
        if link.wait_nav(T.NAV_POSCTL, timeout=6.0):
            ok = True
            break
    print(f"   nav={link.nav}  {'✓ 已进入 Position' if ok else '✗ 切换失败'}")

    print("\n④ 悬停静止（3 s）")
    report("悬停", tap.sweep(link, 3.0))

    print("\n⑤ 满偏航 +%.1f（3 s）" % STICK)
    report("右偏航", tap.sweep(link, 3.0, yaw=STICK))

    print("\n⑥ 满偏航 −%.1f（3 s）" % STICK)
    report("左偏航", tap.sweep(link, 3.0, yaw=-STICK))

    print("\n⑦ 回中并降落")
    link.run(1.5, "回中", arm_deg=0.0)
    T.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
