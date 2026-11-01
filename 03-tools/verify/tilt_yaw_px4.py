#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tilt_yaw_px4.py —— 摆臂倾角 ↔ 偏航权限  ·  PX4 SITL 闭环实测  (v3)
==================================================================
摆臂经 PX4 的 MAV_CMD_DO_SET_ACTUATOR(187) 驱动： θ = 26° × (1 + p1)

v3 的三处改进（针对 v1/v2 的问题）：
  1. 0° 基准放到【首尾各测一次】，并等 HEARTBEAT 确认进入 Position(22) 模式
     —— v1 里 0° 的脉冲被 AUTO_TAKEOFF 吞掉了
  2. 主指标 = 【电机差动量】 D = (c0+c3) − (c1+c2)
     PX4 的 CA_ROTOR*_KM = [+,−,−,+]，所以 D 就是"为产生偏航力矩付出的推力差"
     D 越小 ⇒ 单位差动换来的偏航力矩越大 ⇒ 权限越大。不需求导，不受限幅影响
  3. 角加速度取 PX4 IMU 的 vehicle_angular_velocity.xyz_derivative[2]，
     不再用位姿平滑求导（那会把峰值衰减掉）

时间对齐：记录【解锁瞬间】的墙钟时间，在 ulog 里找 arming_state 0→1 的那一帧，
用它的时间戳做锚点，把每个偏航脉冲窗口精确映射到 ulog 时间轴。

用法：
    python3 tilt_yaw_px4.py            # 跑测试（会自动接着分析）
    python3 tilt_yaw_px4.py analyze    # 只分析上一次
"""

import glob
import json
import math
import os
import socket
import subprocess
import sys
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")

from pymavlink.dialects.v20 import common as mav_common

PX4 = os.environ.get("PX4_DIR", os.path.expanduser("~/PX4-Autopilot"))
LOG = "/tmp/px4_tilt_yaw.log"
WIN = "/tmp/tilt_yaw_windows.json"
PORT, LOCAL = 18570, 14553
JOINT_TOPIC = "/airground/joint_state"
POSE_TOPIC = "/world/airground/pose/info"
MODEL = "airground_0"

# 顺序：先 0° 拿基准，再逐档加大，最后再回 0° 复核（两次）
SCHEDULE = [(0, "0a"), (5, "5"), (10, "10"), (15, "15"),
            (20, "20"), (25, "25"), (0, "0b"), (0, "0c")]

ARM_K = 26.0
SETTLE = 5.0
YAW_SEC = 2.0
HOLD_SEC = 1.5
STICK = 0.8          # 指令 ≈ 160 °/s（满杆 200），略低于上限，避免纯撞限幅

NAV_POSCTL = 22


def arm_param(deg):
    return deg / ARM_K - 1.0


# ───────────────────────── 进程管理 ─────────────────────────
def cleanup():
    for pat in ("build/px4_sitl_default/bin/px4", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(2)
    for f in glob.glob("/tmp/px4_lock-*"):
        try:
            os.remove(f)
        except OSError:
            pass


def boot():
    print("② 启动 PX4 SITL + Gazebo（server only）")
    logf = open(LOG, "w")
    subprocess.Popen(["make", "px4_sitl", "gz_airground"], cwd=PX4,
                     stdout=logf, stderr=subprocess.STDOUT,
                     preexec_fn=os.setsid)
    print("③ 等待就绪", end="", flush=True)
    for _ in range(90):
        time.sleep(1)
        print(".", end="", flush=True)
        try:
            if "Ready for takeoff" in open(LOG, errors="ignore").read():
                print(" ✓")
                return True
        except OSError:
            pass
    print(" ✗")
    print("".join(open(LOG, errors="ignore").readlines()[-25:]))
    return False


# ───────────────────────── 遥测（gz 侧）─────────────────────────
class Gimbal:
    def __init__(self):
        from gz.transport13 import Node
        from gz.msgs10.model_pb2 import Model
        from gz.msgs10.pose_v_pb2 import Pose_V
        self.joint = {}
        self.rec = []
        self.node = Node()
        self.node.subscribe(Model, JOINT_TOPIC, self._on_joint)
        self.node.subscribe(Pose_V, POSE_TOPIC, self._on_pose)

    def _on_joint(self, msg):
        for j in msg.joint:
            if j.name in ("servo_0", "servo_1"):
                self.joint[j.name] = math.degrees(j.axis1.position)

    def _on_pose(self, msg):
        for p in msg.pose:
            if p.name != MODEL:
                continue
            q = p.orientation
            yaw = math.degrees(math.atan2(2 * (q.w * q.z + q.x * q.y),
                                          1 - 2 * (q.y * q.y + q.z * q.z)))
            self.rec.append((time.time(), yaw))

    def arm_deg(self):
        a, b = self.joint.get("servo_0"), self.joint.get("servo_1")
        return float("nan") if a is None or b is None else 0.5 * (a + b)


def unwrap(degs):
    out, off = [], 0.0
    for i, d in enumerate(degs):
        if i:
            while d + off - out[-1] > 180:
                off -= 360
            while d + off - out[-1] < -180:
                off += 360
        out.append(d + off)
    return out


def peak_rate(samples):
    """samples: [(t, yaw_deg)] → 峰值偏航角速度 °/s（粗测，仅供实时显示）"""
    if len(samples) < 8:
        return 0.0
    t = [s[0] - samples[0][0] for s in samples]
    y = unwrap([s[1] for s in samples])
    k = max(1, min(3, (len(y) - 1) // 2))
    ys = [sum(y[i - k:i + k + 1]) / (2 * k + 1) for i in range(k, len(y) - k)]
    ts = t[k:len(y) - k]
    if len(ys) < 4:
        return 0.0
    return max(abs((ys[i + 1] - ys[i]) / max(ts[i + 1] - ts[i], 1e-6))
               for i in range(len(ys) - 1))


# ───────────────────────── MAVLink ─────────────────────────
class Link:
    def __init__(self):
        self.s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.s.bind(("127.0.0.1", LOCAL))
        self.s.setblocking(False)
        self.mav = mav_common.MAVLink(None, srcSystem=255, srcComponent=0)
        self.mav.robust_parsing = True
        self.armed = None
        self.nav = None
        self.t_arm = None

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
                    was = self.armed
                    self.armed = bool(m.base_mode & 128)
                    self.nav = (m.custom_mode >> 16) & 0xFF
                    if self.armed and not was:
                        self.t_arm = time.time()

    def cmd(self, c, p1=0, p2=0, p3=0, p4=0, p5=0, p6=0, p7=0):
        self.send(mav_common.MAVLink_command_long_message(
            1, 1, c, 0, p1, p2, p3, p4, p5, p6, p7))

    def manual(self, pitch=0.0, roll=0.0, thr=0.0, yaw=0.0):
        self.send(mav_common.MAVLink_manual_control_message(
            1, int(1000 * pitch), int(1000 * roll),
            int(500 + 500 * thr), int(1000 * yaw),
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0))

    def run(self, secs, label="", arm_deg=0.0, **kw):
        t0 = time.time()
        while time.time() - t0 < secs:
            self.send(mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
            self.cmd(187, arm_param(arm_deg), 0, 0, 0, 0, 0, 0)
            self.manual(**kw)
            self.drain()
            time.sleep(0.02)
        if label:
            print("   %-14s %4.1fs  yaw=%+.2f  nav=%s  armed=%s"
                  % (label, secs, kw.get("yaw", 0.0), self.nav,
                     {True: "是", False: "否", None: "?"}[self.armed]))

    def wait_nav(self, target, timeout=25.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if self.nav == target:
                return True
            self.send(mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
            self.manual()
            self.cmd(187, arm_param(0.0), 0, 0, 0, 0, 0, 0)
            self.drain()
            time.sleep(0.05)
        return False


# ───────────────────────── 主流程 ─────────────────────────
def fly():
    cleanup()
    if not boot():
        return 1

    g = Gimbal()
    link = Link()
    t0 = time.time()
    while time.time() - t0 < 10 and link.armed is None:
        link.send(mav_common.MAVLink_heartbeat_message(6, 8, 0, 0, 0, 3))
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
    if link.t_arm is None:
        print("⚠️ 没捕捉到解锁时刻，时间对齐会退化")
    link.run(11.0, "自动起飞", arm_deg=0.0)

    print("\n⑤ 切 Position 并等待生效")
    for _ in range(10):
        link.cmd(176, 1.0, 3.0, 0.0)
        if link.wait_nav(NAV_POSCTL, timeout=6.0):
            print("   ✓ nav_state = %d (POSCTL)" % link.nav)
            break
        print("   当前 nav_state = %s，重试" % link.nav)
    else:
        print("   ⚠️ 一直没进入 POSCTL，结果可能不可信")
    link.run(3.0, "稳定", arm_deg=0.0)

    print("\n⑥ 逐档偏航脉冲")
    windows = [dict(kind="anchor", t_arm=link.t_arm)]
    for deg, tag in SCHEDULE:
        link.run(SETTLE, f"摆臂→{deg}°", arm_deg=deg, yaw=0.0)
        real = g.arm_deg()
        print("   ── {:<3} θ={:2d}°  摆臂实测 {:+.2f}°".format(tag, deg, real))
        for label, sgn in (("右", +1), ("左", -1)):
            link.run(HOLD_SEC, "稳定", arm_deg=deg, yaw=0.0)
            n0 = len(g.rec)
            t_start = time.time()
            link.run(YAW_SEC, f"{tag}-{label}偏", arm_deg=deg, yaw=sgn * STICK)
            t_end = time.time()
            pr = peak_rate([(r[0], r[1]) for r in g.rec[n0:]])
            windows.append(dict(kind="pulse", tag=tag, deg=deg, dir=label,
                                t0=t_start, t1=t_end, arm=real, peak=pr))
            print("      %s偏  峰值 %6.0f °/s" % (label, pr))
        link.run(HOLD_SEC, "回中", arm_deg=deg, yaw=0.0)

    print("\n⑦ 回 0° 并降落")
    link.run(3.0, "摆臂回 0°", arm_deg=0.0, yaw=0.0)
    link.cmd(176, 1.0, 4.0, 6.0)
    link.run(6.0, "自动降落", arm_deg=0.0, yaw=0.0)
    link.cmd(400, 0.0)
    link.run(1.0, "上锁", arm_deg=0.0, yaw=0.0)
    time.sleep(2.0)

    with open(WIN, "w") as fp:
        json.dump(windows, fp)
    print("   脉冲窗口已存 %s" % WIN)
    return 0


# ───────────────────────── 分析 ─────────────────────────
def analyze():
    import numpy as np
    from pyulog import ULog

    if not os.path.exists(WIN):
        print("没有窗口文件，先跑一次测试")
        return
    windows = json.load(open(WIN))
    anchor = next((w for w in windows if w["kind"] == "anchor"), None)
    pulses = [w for w in windows if w["kind"] == "pulse"]
    if anchor is None or anchor.get("t_arm") is None:
        print("缺少解锁锚点，无法对齐")
        return

    d = os.path.join(PX4, "build/px4_sitl_default/rootfs/log")
    files = sorted(glob.glob(d + "/*/*.ulg"), key=os.path.getmtime)
    if not files:
        print("找不到 ulog")
        return
    print("日志:", files[-1].split("/")[-1])
    u = ULog(files[-1])

    # ① 用 arming_state 0→1 建立墙钟↔ulog 时间偏移
    st = u.get_dataset("vehicle_status")
    ast = np.asarray(st.data["arming_state"])
    at = np.asarray(st.data["timestamp"], float)
    idx = np.argmax(ast > 0) if (ast > 0).any() else None
    if idx is None:
        print("日志里找不到解锁事件")
        return
    off = at[idx] / 1e6 - anchor["t_arm"]        # ulog_s = wall_s + off

    def get(name, cols):
        ds = u.get_dataset(name)
        t = np.asarray(ds.data["timestamp"], float) / 1e6
        return t, {c: np.asarray(ds.data[c], float) for c in cols}

    tm, mot = get("actuator_motors", [f"control[{i}]" for i in range(4)])
    ta, av = get("vehicle_angular_velocity",
                 ["xyz[2]", "xyz_derivative[2]"])
    tsp, sp = get("vehicle_rates_setpoint", ["yaw"])

    rows = []
    for w in pulses:
        a = w["t0"] + off
        b = w["t1"] + off
        sel = (tm >= a + 0.4) & (tm <= b - 0.2)           # 避开指令上升沿
        if sel.sum() < 5:
            continue
        D = float(np.mean((mot["control[0]"][sel] + mot["control[3]"][sel])
                          - (mot["control[1]"][sel] + mot["control[2]"][sel])))
        sa = (ta >= a) & (ta <= b)
        peak_w = float(np.max(np.abs(av["xyz[2]"][sa]))) if sa.any() else 0.0
        peak_a = float(np.max(np.abs(av["xyz_derivative[2]"][sa]))) if sa.any() else 0.0
        ss = (tsp >= a + 0.8) & (tsp <= b)
        tgt = float(np.mean(np.abs(sp["yaw"][ss]))) if ss.any() else 0.0
        rows.append(dict(tag=w["tag"], deg=w["deg"], dir=w["dir"], arm=w["arm"],
                         D=abs(D), rate=math.degrees(peak_w),
                         acc=math.degrees(peak_a), tgt=math.degrees(tgt)))

    if not rows:
        print("没有可用的脉冲窗口（时间对齐失败？）")
        return

    print("\n" + "=" * 84)
    print(f"{'档':>4} {'θ':>4} {'向':>2} {'指令':>7} {'峰值率':>8} "
          f"{'峰值角加速度':>12} {'电机差动D':>10}")
    print("-" * 84)
    for r in rows:
        print(f"{r['tag']:>4} {r['deg']:>3}° {r['dir']:>2} {r['tgt']:>6.0f}° "
              f"{r['rate']:>7.0f}° {r['acc']:>10.0f}°/s² {r['D']:>10.4f}")

    # ③ 汇总：以 0a 为基准（若无效则用 0b/0c 均值）
    def collect(tag):
        g = [r for r in rows if r["tag"] == tag]
        return (sum(r["D"] for r in g) / len(g),
                sum(r["rate"] for r in g) / len(g),
                sum(r["acc"] for r in g) / len(g)) if g else (None, None, None)

    d0, r0, a0 = collect("0a")
    d0b, r0b, a0b = collect("0b")
    d0c, r0c, a0c = collect("0c")
    print(f"\n0° 基准：0a D={d0 and round(d0,4)}  0b D={d0b and round(d0,4)}  "
          f"0c D={d0c and round(d0c,4)}")
    vals = [x for x in (d0, d0b, d0c) if x]
    D0 = sum(vals) / len(vals) if vals else None
    if D0 is None:
        print("无可用 0° 基准")
        return

    print("\n" + "=" * 78)
    print(f"{'θ':>4} {'实测D':>9} {'D/D0':>8} {'理论D/D0':>10} "
          f"{'判定':>8} {'峰值率比':>10} {'理论率比':>10}")
    print("-" * 78)
    for deg in (0, 5, 10, 15, 20, 25):
        g = [r for r in rows if r["deg"] == deg]
        if not g:
            continue
        D = sum(r["D"] for r in g) / len(g)
        R = sum(r["rate"] for r in g) / len(g)
        th = math.cos(math.radians(deg)) + 2.1 * math.sin(math.radians(deg))
        ratio = D / D0
        pred = 1.0 / th
        ok = "✅" if abs(ratio - pred) <= 0.15 * pred + 0.05 else "⚠️"
        r0r = (sum(r["rate"] for r in rows if r["deg"] == 0) /
               max(len([r for r in rows if r["deg"] == 0]), 1))
        print(f"{deg:>3}° {D:>9.4f} {ratio:>8.2f} {pred:>10.2f} {ok:>8} "
              f"{R / r0r if r0r else float('nan'):>10.2f} {th:>10.2f}")
    print("=" * 78)
    print("判据：D/D0 越小 = 单位差动换来的偏航力矩越大；应≈ 1/(cosθ+2.1sinθ)")


if __name__ == "__main__":
    if len(sys.argv) > 1 and sys.argv[1] == "analyze":
        analyze()
    else:
        try:
            rc = fly()
            if rc == 0:
                analyze()
        finally:
            print("\n⑧ 清理")
            cleanup()
        sys.exit(0)
