#!/usr/bin/env python3
"""
yaw_test.py —— 受控偏航权限测试（不依赖遥控器）

流程：
  切 Position 模式 → 解锁 → 爬升 3 s → 悬停 2 s
  → 右偏航 3 s → 悬停 1 s → 左偏航 3 s → 悬停 1 s

用途：比较不同 CA_ROTORx_KM 符号配置下的偏航跟踪能力。
跑完用 analyze 子命令检查生成的 .ulg 日志。

用法：
    python3 yaw_test.py            # 跑一遍测试
    python3 yaw_test.py analyze    # 分析最新日志
"""

import socket
import sys
import time
import warnings

import numpy as np
from pymavlink.dialects.v20 import common as mav_common

PORT = 18570
LOCAL = 14553          # 独立端口，不和 QGC(14550) / joystick_drive(14551) 抢


class Link:
    """最小 MAVLink 客户端：心跳 + MANUAL_CONTROL + 命令。"""

    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(("127.0.0.1", LOCAL))
        self.s.setblocking(False)
        self.mav = mav_common.MAVLink(None, srcSystem=255, srcComponent=0)
        self.mav.robust_parsing = True
        self.armed = None
        self.hb = 0

    def send(self, msg):
        try:
            self.s.sendto(msg.pack(self.mav), ("127.0.0.1", PORT))
        except OSError:
            pass

    def drain(self):
        while True:
            try:
                data, _ = self.s.recvfrom(65535)
            except (BlockingIOError, OSError):
                return
            for m in self.mav.parse_buffer(data) or []:
                if m.get_type() == "HEARTBEAT" and m.get_srcSystem() != 255:
                    self.hb += 1
                    self.armed = bool(m.base_mode & 128)

    def cmd(self, c, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        self.send(mav_common.MAVLink_command_long_message(
            1, 1, c, 0, p1, p2, p3, p4, p5, p6, p7))

    def manual(self, pitch=0.0, roll=0.0, thr=0.0, yaw=0.0):
        # x=pitch  y=roll  z=throttle[0..1000]  r=yaw
        self.send(mav_common.MAVLink_manual_control_message(
            1, int(1000 * pitch), int(1000 * roll),
            int(500 + 500 * thr), int(1000 * yaw),
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    def run(self, secs, label="", **kw):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.send(mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
            self.manual(**kw)
            self.drain()
            time.sleep(0.02)
        if label:
            print("   %-14s %4.1fs   thr=%+.2f yaw=%+.2f  armed=%s"
                  % (label, secs, kw.get("thr", 0), kw.get("yaw", 0),
                     {True: "是", False: "否", None: "?"}[self.armed]))


def fly():
    link = Link()
    print("=== 偏航权限测试 ===")
    # ① 先切 AUTO.TAKEOFF（Position 模式下从地面是起不来的）
    link.cmd(176, 1.0, 4.0, 2.0)      # DO_SET_MODE: custom, AUTO(4), TAKEOFF(2)
    link.run(1.0, "切 Takeoff")
    link.cmd(400, 1.0)                # ARM
    link.run(1.5, "解锁")
    if link.armed is False:
        print("   ⚠️ 解锁失败，检查健康检查项")
    link.run(8.0, "自动起飞")          # Takeoff 模式会自己爬到目标高度
    # ② 切回 Position 做偏航测试
    link.cmd(176, 1.0, 3.0, 0.0)      # DO_SET_MODE: custom, POSCTL(3)
    link.run(3.0, "切 Position", thr=0.0)
    link.run(3.0, "右偏航", thr=0.0, yaw=+0.8)
    link.run(1.0, "悬停", thr=0.0)
    link.run(3.0, "左偏航", thr=0.0, yaw=-0.8)
    link.run(1.0, "悬停", thr=0.0)
    # ③ 降落 + 上锁
    link.cmd(176, 1.0, 4.0, 6.0)      # AUTO.LAND
    link.run(3.0, "自动降落")
    link.cmd(400, 0.0)                # DISARM
    link.run(1.0, "上锁")
    print("=== 完成，跑 `python3 yaw_test.py analyze` 看结果 ===")


def analyze():
    warnings.filterwarnings("ignore")
    import glob
    import os
    from pyulog import ULog

    d = os.path.expanduser("~/PX4-Autopilot/build/px4_sitl_default/rootfs/log")
    files = sorted(glob.glob(d + "/*/*.ulg"), key=os.path.getmtime)
    if not files:
        print("没找到日志"); return
    f = files[-1]
    print("日志:", f)
    u = ULog(f)
    sp = u.get_dataset("vehicle_rates_setpoint")
    av = u.get_dataset("vehicle_angular_velocity")
    ts = (sp.data["timestamp"] - sp.data["timestamp"][0]) / 1e6
    ta = (av.data["timestamp"] - av.data["timestamp"][0]) / 1e6
    s = np.asarray(sp.data["yaw"], dtype=float)
    a = np.interp(ts, ta, np.asarray(av.data["xyz[2]"], dtype=float))

    m = np.abs(s) > 0.3
    print("\n偏航指令 |sp|>0.3 rad/s: %d / %d 帧" % (m.sum(), len(s)))
    if m.sum() > 50:
        ratio = a[m].mean() / s[m].mean() if abs(s[m].mean()) > 1e-6 else float("nan")
        print("  指令均值 %+.2f rad/s (%+.0f°/s)"
              % (s[m].mean(), np.degrees(s[m].mean())))
        print("  实际均值 %+.2f rad/s (%+.0f°/s)"
              % (a[m].mean(), np.degrees(a[m].mean())))
        print("  跟踪率   %+.0f%%" % (100 * ratio))
        print("  相关系数 %+.2f" % np.corrcoef(s[m], a[m])[0, 1])
        print("  实际峰值 %.0f°/s" % np.degrees(np.abs(a[m]).max()))
    else:
        print("  有效偏航帧太少，测试可能没飞起来")

    pos = u.get_dataset("vehicle_local_position")
    z = np.asarray(pos.data["z"], dtype=float)
    print("\n高度: 最大 %.2f m  末值 %.2f m" % (-z.min(), -z[-1]))


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "analyze":
        analyze()
    else:
        fly()
