#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 摆臂滑条控制器 —— 未解锁 / 已解锁 都能用
================================================================================
 为什么需要这个脚本（QGC 自带滑条做不到的事）：

   QGC 的 Actuators 滑条发的是 MAV_CMD_ACTUATOR_TEST(310)，
   而 PX4 规定 310 【必须在未解锁状态】使用：

       Commander::handleCommandActuatorTest()
           if (isArmed()) return MAV_RESULT_DENIED;

   所以「已解锁（但还在地面）时折摆臂」QGC 做不到。

   PX4 有另一条路 —— MAV_CMD_DO_SET_ACTUATOR(187)：
       Commander.cpp:  case VEHICLE_CMD_DO_SET_ACTUATOR:
                           answer_command(..., ACCEPTED);   // 无任何限制
   但它需要 mixer 处于 armed 状态（mixer_module.cpp:461）。

   ⇒ 两条命令【互补】，本脚本按 arm 状态自动切换：
        未解锁  →  MAV_CMD_ACTUATOR_TEST(310)   免解锁，20 Hz 重发
        已解锁  →  MAV_CMD_DO_SET_ACTUATOR(187) 锁存型，变化时才发

 安全保护（默认开启，--no-guard 关）：
   机身高度 > 0.20 m 判为飞行 → 强制摆臂回 0° 并禁止操作。
   ⚠️ 阈值 0.20 不是随便取的：摆臂折叠【本身】会把机身撑高
      （0°→0.021 m，52°→0.1014 m），阈值必须超过机构行程。

 用法：
     python3 servo_slider.py                  # 默认连 udpout:127.0.0.1:18570
     python3 servo_slider.py --port 18570
     python3 servo_slider.py --no-guard       # 关掉高度保护
================================================================================
"""

import argparse
import collections
import sys
import threading
import time

import tkinter as tk
from tkinter import ttk, messagebox

from pymavlink import mavutil

# ---------------------------------------------------------------- 常量

CMD_DO_SET_ACTUATOR = 187          # ⚠️ 数字！ardupilotmega 方言没有名字常量
CMD_ACTUATOR_TEST = 310            # ⚠️ 同上
ACT_ID = 301                       # SIM_GZ_SV_FUNC1 的功能号
ACT_FUNC_PARAM = 1000 + ACT_ID      # 310 的 param5 = 1301

ANGLE_MIN, ANGLE_MAX = 0.0, 52.0    # SIM_GZ_SV_MINA1 / MAXA1
GROUND_Z = 0.20                     # 判"在地面"的高度阈值（m）
SEND_HZ = 20                        # 310 必须持续重发（timeout 上限 3 s）

# ---------------------------------------------------------------- 角度换算
# mixer 两层映射（MIN1=0/MAX1=1000, MINA1=0°/MAXA1=52°）推出：
#     angle = 26 * (value + 1)      value = angle/26 - 1
# 即：value=-1 → 0°   value=0 → 26°   value=+1 → 52°


def ang2val(deg):
    return deg / (ANGLE_MAX / 2.0) - 1.0


def val2ang(v):
    return (ANGLE_MAX / 2.0) * (v + 1.0)


# ---------------------------------------------------------------- MAVLink 层

class Link:
    """负责连接、心跳、遥测接收、命令发送"""

    def __init__(self, port, guard=True, log=None):
        self.port = port
        self.guard = guard
        self.log = log or (lambda s: None)

        self.conn = None
        self.sysid = self.compid = None

        # 共享状态（UI 与后台线程都读）
        self.lock = threading.Lock()
        self.armed = None          # None=未知，True/False
        self.alt = 0.0             # 机身高度（m，向上为正）
        self.mode = 0
        self.blocked = False       # 高度保护是否触发
        self.value = -1.0          # 目标命令值 [-1, +1]（-1 = 0°）
        self.connected = False
        self.ack_denied = 0

        self._last187 = None
        self._last187_t = 0.0
        self._stop = threading.Event()
        self._th = threading.Thread(target=self._run, daemon=True)

    # ---------- 生命周期 ----------

    def start(self):
        self._th.start()

    def stop(self):
        self._stop.set()

    def zero(self):
        """让摆臂回 0°（值 -1）"""
        with self.lock:
            self.value = -1.0

    # ---------- 后台循环 ----------

    def _run(self):
        try:
            self.conn = mavutil.mavlink_connection(
                f"udpout:127.0.0.1:{self.port}",
                source_system=255, source_component=0)
        except Exception as e:                       # noqa: BLE001
            self.log(f"❌ 建立连接失败: {e}")
            return

        # 握手：持续发心跳直到收到 PX4 的
        for _ in range(30):
            self._hb()
            if self.conn.wait_heartbeat(timeout=0.5):
                break
        else:
            self.log("❌ 收不到 PX4 心跳 —— 仿真在跑吗？")
            return

        self.sysid = self.conn.target_system
        self.compid = self.conn.target_component
        with self.lock:
            self.connected = True
        self.log(f"✅ 已连接 PX4 (sys={self.sysid})")

        self._scan_sysid()

        dt = 1.0 / SEND_HZ
        while not self._stop.is_set():
            t0 = time.time()
            self._pump_rx()
            self._send_once()
            self._hb()
            time.sleep(max(0.0, dt - (time.time() - t0)))

    def _hb(self):
        try:
            self.conn.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        except Exception:                            # noqa: BLE001
            pass

    def _scan_sysid(self):
        """额外把 ALIVE 的 system id 记下来（多 sysid 场景保险）"""
        while True:
            m = self.conn.recv_match(type="HEARTBEAT", blocking=False)
            if m is None:
                break

    def _pump_rx(self):
        """⚠️ 必须一次抽干缓冲，否则高频遥测下永远读到最旧的数据"""
        while True:
            m = self.conn.recv_match(
                type=["HEARTBEAT", "LOCAL_POSITION_NED",
                      "COMMAND_ACK", "GLOBAL_POSITION_INT"],
                blocking=False)
            if m is None:
                break
            t = m.get_type()
            if t == "HEARTBEAT":
                if m.get_srcSystem() != self.sysid:
                    continue
                armed = bool(m.base_mode
                             & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                with self.lock:
                    if self.armed is not None and self.armed and not armed:
                        self.log("⚠️ 检测到【自动上锁】—— 摆臂已回 0°。"
                                 "（COM_DISARM_PRFLT 默认 10 s，"
                                 "可用「关闭自动上锁」按钮）")
                    self.armed = armed
                    self.mode = m.custom_mode
            elif t == "LOCAL_POSITION_NED":
                # NED：z 向下为正 ⇒ 高度 = -z
                with self.lock:
                    self.alt = -m.z
            elif t == "COMMAND_ACK":
                if m.result == mavutil.mavlink.MAV_RESULT_DENIED:
                    with self.lock:
                        self.ack_denied += 1

    def _send_once(self):
        with self.lock:
            if not self.connected:
                return
            armed = self.armed
            alt = self.alt
            goal = self.value

        # ---- 高度保护 ----
        blocked = self.guard and alt > GROUND_Z
        with self.lock:
            self.blocked = blocked
        if blocked:
            goal = -1.0                      # 强制 0°（安全位）

        if armed is None:
            return

        if armed:
            # 187：锁存型。只在值变化时发；另外 1 Hz 补发一次防被别的源覆盖
            now = time.time()
            if goal != self._last187 or (now - self._last187_t) > 1.0:
                self._send187(goal)
                self._last187 = goal
                self._last187_t = now
        else:
            # 310：必须持续重发（否则 3 s 超时 → 回 disarmed 位）
            self._send310(goal)

    def _send187(self, value):
        try:
            self.conn.mav.command_long_send(
                self.sysid, self.compid, CMD_DO_SET_ACTUATOR, 0,
                float(value), 0, 0, 0, 0, 0,
                0)          # param7=0 ⇒ param1~6 对应执行器 1~6
        except Exception as e:                       # noqa: BLE001
            self.log(f"⚠️ 187 发送失败: {e}")

    def _send310(self, value):
        try:
            self.conn.mav.command_long_send(
                self.sysid, self.compid, CMD_ACTUATOR_TEST, 0,
                float(value),        # param1 = value
                1.0,                 # param2 = timeout(s)
                0, 0,
                ACT_FUNC_PARAM,      # param5 = 1301
                0, 0)
        except Exception as e:                       # noqa: BLE001
            self.log(f"⚠️ 310 发送失败: {e}")

    # ---------- 解锁 / 上锁 ----------

    def arm(self, want):
        try:
            self.conn.mav.command_long_send(
                self.sysid, self.compid,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                1 if want else 0, 0, 0, 0, 0, 0, 0)
            self.log(f"→ 发送{'解' if want else '上'}锁命令")
        except Exception as e:                       # noqa: BLE001
            self.log(f"⚠️ 发送失败: {e}")

    def set_disarm_params(self, enable):
        """COM_DISARM_PRFLT / COM_DISARM_LAND —— ⚠️ 是 FLOAT 类型！"""
        for name, val in (("COM_DISARM_PRFLT", 0.0 if enable else 10.0),
                          ("COM_DISARM_LAND", 0.0 if enable else 2.0)):
            try:
                self.conn.mav.param_set_send(
                    self.sysid, self.compid, name.encode(), val,
                    mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
            except Exception as e:                   # noqa: BLE001
                self.log(f"⚠️ {name} 设置失败: {e}")
        if enable:
            self.log("✅ 已关闭自动上锁（COM_DISARM_PRFLT=0, COM_DISARM_LAND=0）")
            self.log("   ⚠️ 会持久化到 parameters.bson；"
                     "删除该文件可恢复出厂 10 s / 2 s")
        else:
            self.log("✅ 已恢复自动上锁（10 s / 2 s）")


# ---------------------------------------------------------------- UI

class App:
    def __init__(self, root, link):
        self.root = root
        self.link = link
        root.title("摆臂滑条控制器 —— airground")
        root.geometry("640x520")
        root.protocol("WM_DELETE_WINDOW", self.on_close)

        self._suppress = False
        # ⚠️ tkinter 控件【不能】从后台线程写 → 日志先入队，在主线程 tick() 里落盘
        self._logq = collections.deque()

        # ===== 状态栏 =====
        bar = ttk.Frame(root, padding=(10, 8))
        bar.pack(fill="x")
        self.lbl_conn = ttk.Label(bar, text="● 未连接", foreground="gray")
        self.lbl_conn.pack(side="left")
        self.lbl_arm = ttk.Label(bar, text="arm: ?", foreground="gray")
        self.lbl_arm.pack(side="left", padx=14)
        self.lbl_alt = ttk.Label(bar, text="高度: —")
        self.lbl_alt.pack(side="left", padx=14)
        self.lbl_cmd = ttk.Label(bar, text="命令: —")
        self.lbl_cmd.pack(side="left", padx=14)

        ttk.Separator(root).pack(fill="x")

        # ===== 角度显示 =====
        head = ttk.Frame(root, padding=(10, 10))
        head.pack(fill="x")
        ttk.Label(head, text="摆臂角度", font=("", 11)).pack(side="left")
        self.lbl_angle = ttk.Label(head, text="0.0 °", font=("", 22, "bold"))
        self.lbl_angle.pack(side="left", padx=16)
        self.lbl_warn = ttk.Label(head, text="", foreground="red",
                                  font=("", 11, "bold"))
        self.lbl_warn.pack(side="left")

        # ===== 滑条 =====
        body = ttk.Frame(root, padding=(14, 4))
        body.pack(fill="x")
        self.var_angle = tk.DoubleVar(value=ANGLE_MIN)
        self.scale = ttk.Scale(body, from_=ANGLE_MIN, to=ANGLE_MAX,
                               orient="horizontal", variable=self.var_angle,
                               command=self.on_slide)
        self.scale.pack(fill="x")
        ticks = ttk.Frame(body)
        ticks.pack(fill="x")
        for a, side in ((0, "left"), (13, "left"), (26, "center"),
                        (39, "right"), (52, "right")):
            ttk.Label(ticks, text=f"{a}°").pack(side=side)

        # ===== 精确输入 + 预设 =====
        row = ttk.Frame(root, padding=(14, 8))
        row.pack(fill="x")
        ttk.Label(row, text="精确值:").pack(side="left")
        self.ent = ttk.Entry(row, width=8)
        self.ent.insert(0, "0.0")
        self.ent.pack(side="left", padx=6)
        self.ent.bind("<Return>", lambda e: self.on_apply())
        ttk.Label(row, text="°").pack(side="left")
        ttk.Button(row, text="应用", command=self.on_apply).pack(side="left", padx=6)
        ttk.Separator(row, orient="vertical").pack(side="left", fill="y", padx=10)
        ttk.Label(row, text="预设:").pack(side="left")
        for a in (0, 13, 26, 39, 52):
            ttk.Button(row, text=f"{a}°", width=4,
                       command=lambda x=a: self.set_angle(x)).pack(side="left", padx=2)

        # ===== 操作按钮 =====
        ops = ttk.Frame(root, padding=(14, 4))
        ops.pack(fill="x")
        ttk.Button(ops, text="解锁", command=lambda: self.link.arm(True)).pack(side="left")
        ttk.Button(ops, text="上锁", command=lambda: self.link.arm(False)).pack(side="left", padx=6)
        ttk.Button(ops, text="归零 0°", command=self.on_zero).pack(side="left", padx=6)
        ttk.Separator(ops, orient="vertical").pack(side="left", fill="y", padx=10)
        ttk.Button(ops, text="关闭自动上锁",
                   command=lambda: self.link.set_disarm_params(True)).pack(side="left")
        ttk.Button(ops, text="恢复自动上锁",
                   command=lambda: self.link.set_disarm_params(False)).pack(side="left", padx=6)

        # ===== 日志 =====
        ttk.Separator(root).pack(fill="x", pady=(8, 0))
        lf = ttk.Frame(root, padding=(10, 6))
        lf.pack(fill="both", expand=True)
        self.txt = tk.Text(lf, height=9, state="disabled", wrap="word")
        self.txt.pack(side="left", fill="both", expand=True)
        sb = ttk.Scrollbar(lf, command=self.txt.yview)
        sb.pack(side="right", fill="y")
        self.txt.configure(yscrollcommand=sb.set)

        self.log("提示：滑条拖动即生效；松手保持。")
        self.log("      未解锁 → 用 ACTUATOR_TEST(310)   已解锁 → 用 DO_SET_ACTUATOR(187)")
        if self.link.guard:
            self.log(f"      高度保护已开启：>{GROUND_Z} m 强制回 0°（--no-guard 可关）")

        self.root.after(100, self.tick)

    # ---------- 事件 ----------

    def on_slide(self, _v):
        if self._suppress:
            return
        self.set_angle(self.var_angle.get())

    def on_apply(self):
        try:
            self.set_angle(float(self.ent.get()))
        except ValueError:
            messagebox.showerror("输入错误", "请输入数字")

    def set_angle(self, deg):
        deg = max(ANGLE_MIN, min(ANGLE_MAX, float(deg)))
        with self.link.lock:
            self.link.value = ang2val(deg)
        self._suppress = True
        self.var_angle.set(deg)
        self._suppress = False
        self.ent.delete(0, "end")
        self.ent.insert(0, f"{deg:.1f}")
        self.lbl_angle.configure(text=f"{deg:.1f} °")

    def on_zero(self):
        """归零并同时清掉 187 的锁存值"""
        self.set_angle(0.0)
        self.log("→ 归零 0°")

    def on_close(self):
        self.log("退出：让摆臂回 0° ...")
        self.link.zero()
        time.sleep(0.35)
        self.link.stop()
        time.sleep(0.15)
        self.root.destroy()

    # ---------- 刷新 ----------

    def tick(self):
        self._drain_log()

        with self.link.lock:
            conn = self.link.connected
            armed = self.link.armed
            alt = self.link.alt
            blocked = self.link.blocked
            denied = self.link.ack_denied

        self.lbl_conn.configure(
            text="● 已连接" if conn else "● 未连接",
            foreground="green" if conn else "gray")

        if armed is None:
            self.lbl_arm.configure(text="arm: ?", foreground="gray")
            self.lbl_cmd.configure(text="命令: —")
        elif armed:
            self.lbl_arm.configure(text="arm: 已解锁", foreground="red")
            self.lbl_cmd.configure(text=f"命令: DO_SET_ACTUATOR(187)"
                                        f"   拒绝 {denied}", foreground="black")
        else:
            self.lbl_arm.configure(text="arm: 未解锁", foreground="green")
            self.lbl_cmd.configure(text=f"命令: ACTUATOR_TEST(310)"
                                        f"   拒绝 {denied}", foreground="black")

        self.lbl_alt.configure(text=f"高度: {alt:.3f} m")
        self.lbl_warn.configure(
            text=f"⚠️ 高于 {GROUND_Z} m — 强制 0°" if blocked else "")

        self.root.after(100, self.tick)

    def log(self, s):
        """线程安全：只入队；控件写入由主线程的 _drain_log() 做"""
        self._logq.append((time.time(), s))

    def _drain_log(self):
        if not self._logq:
            return
        self.txt.configure(state="normal")
        while self._logq:
            t, s = self._logq.popleft()
            self.txt.insert("end", f"{time.strftime('%H:%M:%S', time.localtime(t))}  {s}\n")
        self.txt.see("end")
        self.txt.configure(state="disabled")


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="摆臂滑条控制器")
    ap.add_argument("--port", type=int, default=18570,
                    help="PX4 GCS UDP 端口（默认 18570，与 PX4 SITL 一致）")
    ap.add_argument("--no-guard", action="store_true",
                    help="关闭高度保护（默认开启）")
    args = ap.parse_args()

    root = tk.Tk()
    # 先建一个临时 log 壳，Link 里再替换
    app_holder = {}

    def early_log(s):
        print(s, flush=True)
        if "app" in app_holder:
            app_holder["app"].log(s)

    link = Link(args.port, guard=not args.no_guard, log=early_log)
    app = App(root, link)
    app_holder["app"] = app
    link.start()

    try:
        root.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        link.stop()
        print("已退出。", flush=True)


if __name__ == "__main__":
    try:
        main()
    except Exception:                                # noqa: BLE001
        import traceback
        traceback.print_exc()
        sys.exit(1)
