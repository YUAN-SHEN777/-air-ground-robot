#!/usr/bin/env python3
"""
joystick_drive.py —— 用 USB 遥控器（/dev/input/js0）直接操控 PX4 SITL 地面模式

为什么不用 QGC 的摇杆
--------------------
QGC 的摇杆依赖 SDL + 窗口状态，出现过「连着但一帧 MANUAL_CONTROL 都发不出去」
的情况（见 05_18_43.ulg：101 秒内 manual_control_setpoint 只有 1 帧且 valid=0），
结果 PX4 判定遥控丢失 → failsafe → 卡在 Hold。本脚本直接读内核 joystick 接口，
自己发 MANUAL_CONTROL，链路简单、可观测、不依赖 GUI。

轴映射（本机实测：OpenTX HelloRadioSky V14，8 轴 / 24 按钮）
------------------------------------------------------------
    axis0 = CH1 副翼  → 自回中   → 转向 (roll)
    axis1 = CH2 升降  → 自回中   → 前进/后退 (pitch)
    axis2 = CH3 油门  → 不回中   → 默认忽略（见下方警告）
    axis3 = CH4 方向  → 自回中   → 默认忽略
    axis4~7           → 未使用

⚠️ axis2 是油门杆，不自动回中，当前停在最低位 (-32767)。
   如果把它映射到 throttle，脚本一启动就会满倒车。所以默认不映射。
   想用左油门杆调速，加 --use-throttle，并且启动前确认杆在中位。

用法
----
    python3 joystick_drive.py                 # 只看数值，不发任何 MAVLink（安全）
    python3 joystick_drive.py --list          # 标定模式：显示所有轴/按钮编号
    python3 joystick_drive.py --send --servo 1.0
                                             # 真正操控：自动摆臂到52°+
                                             # 自动切 GROUND_DRIVE
    python3 joystick_drive.py --send --arm-btn 10 --mode-btn 11

完整驾驶流程（不依赖 QGC）
--------------------------
    终端 1：cd ~/PX4-Autopilot && HEADLESS=1 make px4_sitl gz_airground
    终端 2：cd "/home/yuan/robot air-ground/robot_urdf (4)"
            python3 joystick_drive.py --send --servo 1.0
    然后在 PX4 控制台解锁一次：  commander arm
    （或直接加 --arm-btn 用按钮解锁）

    推右摇杆上下 = 前进/倒车，左右 = 转向，松手 = 停。

正统 RC 通路（--rc）
-------------------
    默认走 MAVLink MANUAL_CONTROL（手柄语义）。加 --rc 改为发
    **RC_CHANNELS_OVERRIDE(70)**，PX4 会把它当成真正的遥控器：
        input_rc (RC_INPUT_SOURCE_MAVLINK) → rc_update → SOURCE_RC
    会走完整的 RC 参数（RC_MAP_* / RC1_MIN/MAX/TRIM / 通道开关 / 失控保护）。

    通道映射（AETR）：
        CH1 ← 转向轴(axis0)   CH2 ← 前进轴(axis1)
        CH3 ← 1500 常量（默认） CH4 ← 1500
    ⚠️ CH3（油门）默认不接油门杆：那根杆不自动回中，停在最低位时
       归一化为 -1，会让车满倒车。想接就加 --rc-throttle-axis 2。

    PID / RC 前提（已写进 airframe）：
        RC_CHAN_CNT=8, RC_MAP_ROLL=1/PITCH=2/THROTTLE=3/YAW=4,
        COM_RC_IN_MODE=5（RC 优先，MAVLink 兜底）

按键约定
--------
    --arm-btn N      按下 N 号按钮 → 解锁；再按 → 上锁
    --mode-btn N     按下 N 号按钮 → 切 GROUND_DRIVE
    具体编号用 --list 标定：按一下按钮，屏幕会打印它的编号。

退出
----
    Ctrl+C 会先连发 5 帧「零油门 + 中位」再断开，车不会带着上次的指令跑。
"""

import argparse
import os
import select
import socket
import struct
import sys
import time

from pymavlink.dialects.v20 import common as mav_common

# ── 内核 joystick 接口（无需任何第三方库）──
JS_DEV = "/dev/input/js0"
JS_EVENT_BUTTON = 0x01
JS_EVENT_AXIS = 0x02
JS_EVENT_INIT = 0x80
AXIS_MAX = 32767.0

# ── MAVLink ──
PORT = 18570          # PX4 MAVLink 服务端口（UDP server）
# ⚠️ 本地端口**不能**用 14550 —— 那是 QGC 的监听口。
#    两个进程抢同一 UDP 端口时，QGC 会收不到遥测，表现为「QGC 与 PX4 失联」。
#    PX4 是 UDP 服务器，谁给它发包它就回到谁的源端口，所以换个端口就能共存。
LOCAL_PORT = 14551
SYS_ID = 255
COMP_ID = 0

# 切外部模式 1：param1=CUSTOM_MODE_ENABLED, param2=MAIN_MODE_AUTO(4), param3=SUB_MODE_EXTERNAL1(11)
CMD_DO_SET_MODE = 176
MODE_EXT1 = (176, 1.0, 4.0, 11.0)
CMD_ARM_DISARM = 400


# ═══════════════════════ 摇杆读取 ═══════════════════════
class Joystick:
    def __init__(self, dev):
        try:
            self.fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as exc:
            sys.exit(f"打不开 {dev}: {exc}\n"
                     f"  检查：ls -l /dev/input/js*\n"
                     f"  若不存在，确认遥控器已切到「USB 摇杆/USB Joystick」模式")
        self.axes = {}
        self.buttons = {}
        self._initial = True

    def poll(self):
        """返回本周期内「刚按下」的按钮编号列表。"""
        pressed_now = []
        while True:
            ready, _, _ = select.select([self.fd], [], [], 0)
            if not ready:
                break
            try:
                data = os.read(self.fd, 8 * 64)
            except BlockingIOError:
                break
            if not data:
                break
            for i in range(0, len(data) - 7, 8):
                _, value, etype, number = struct.unpack("<IhBB", data[i:i + 8])
                etype &= ~JS_EVENT_INIT
                if etype == JS_EVENT_AXIS:
                    self.axes[number] = value
                elif etype == JS_EVENT_BUTTON:
                    if value and not self.buttons.get(number, 0):
                        pressed_now.append(number)
                    self.buttons[number] = value
        self._initial = False
        return pressed_now

    def axis(self, number):
        """归一化到 [-1, 1]。"""
        return self.axes.get(number, 0) / AXIS_MAX

    def close(self):
        try:
            os.close(self.fd)
        except OSError:
            pass


def shape(value, deadband, expo):
    """死区 + 柔和曲线。死区内归零，避免杆位漂移导致车自己爬。"""
    mag = abs(value)
    if mag <= deadband:
        return 0.0
    scaled = (mag - deadband) / (1.0 - deadband)
    if expo > 0.0:
        scaled = (1.0 - expo) * scaled + expo * scaled ** 3
    return scaled if value > 0 else -scaled


# ═══════════════════════ MAVLink 收发 ═══════════════════════
class Link:
    MAV_TYPE_GCS = 6
    MAV_AUTOPILOT_INVALID = 8
    ARMED_BIT = 128

    def __init__(self, local_port=LOCAL_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", local_port))
        self.sock.setblocking(False)
        self.dest = ("127.0.0.1", PORT)
        self.mav = mav_common.MAVLink(None, srcSystem=SYS_ID, srcComponent=COMP_ID)
        self.mav.robust_parsing = True
        self.armed = None
        self.hb_rx = 0
        self.t_heart = 0.0
        self.t_mode = 0.0

    def send(self, msg):
        try:
            self.sock.sendto(msg.pack(self.mav), self.dest)
        except OSError:
            pass

    def drain(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                return
            for msg in self.mav.parse_buffer(data) or []:
                if msg.get_type() == "HEARTBEAT" and msg.get_srcSystem() != SYS_ID:
                    self.hb_rx += 1
                    self.armed = bool(msg.base_mode & self.ARMED_BIT)

    def command_long(self, command, p1=0.0, p2=0.0, p3=0.0, p4=0.0,
                     p5=0.0, p6=0.0, p7=0.0):
        self.send(mav_common.MAVLink_command_long_message(
            1, 1, command, 0, p1, p2, p3, p4, p5, p6, p7))

    def heartbeat(self, now):
        if now - self.t_heart >= 0.1:
            self.send(mav_common.MAVLink_heartbeat_message(
                self.MAV_TYPE_GCS, self.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3))
            self.t_heart = now

    def manual_control(self, pitch, roll, throttle):
        """⚠️ MAVLink 的字段顺序是 x=pitch, y=roll, z=throttle, r=yaw。
        z 用 0~1000 表示，500 = 中立 = 停（对应 DShot 3D 的可逆中位）。"""
        self.send(mav_common.MAVLink_manual_control_message(
            1,
            int(round(1000 * pitch)),            # x ← pitch
            int(round(1000 * roll)),             # y ← roll
            int(round(500 + 500 * throttle)),    # z ← throttle
            0, 0,                                # r, buttons
            0, 0, 0, 0, 0, 0, 0, 0, 0, 0))       # 扩展字段必须全给

    def rc_override(self, channels):
        """RC_CHANNELS_OVERRIDE(70)：正统 RC 通路。

        PX4 的 mavlink_receiver 会把它原样填进 input_rc，
        再由 rc_update 打上 data_source = SOURCE_RC。
        ⚠️ PX4 **不**识别 MAVLink 规范的 "0 = 忽略此通道"，它是直接拷贝，
        所以 18 个通道必须全部给合理值（未用通道填 1500 中位）。
        """
        ch = list(channels) + [1500] * (18 - len(channels))
        self.send(mav_common.MAVLink_rc_channels_override_message(
            1, 1, *ch[:18]))

    def set_mode_ext1(self):
        self.command_long(*MODE_EXT1)

    def arm(self, on):
        self.command_long(CMD_ARM_DISARM, 1.0 if on else 0.0,
                          21196.0 if on else 0.0)   # 21196 = 强制


# ═══════════════════════ 主循环 ═══════════════════════
def main():
    ap = argparse.ArgumentParser(description="USB 遥控器直驱 PX4 SITL 地面模式")
    ap.add_argument("--send", action="store_true", help="真正发送 MAVLink（否则只显示数值）")
    ap.add_argument("--list", action="store_true", help="标定模式：打印所有轴/按钮编号")
    ap.add_argument("--dev", default=JS_DEV)
    ap.add_argument("--axis-fwd", type=int, default=1, help="前进/后退轴的编号（默认 1=CH2）")
    ap.add_argument("--axis-steer", type=int, default=0, help="转向轴的编号（默认 0=CH1）")
    ap.add_argument("--use-throttle", action="store_true",
                    help="把 axis 2（油门杆）也叠加到前进量上（⚠️ 该杆不回中）")
    ap.add_argument("--invert-fwd", action="store_true")
    ap.add_argument("--invert-steer", action="store_true")
    ap.add_argument("--deadband", type=float, default=0.06)
    ap.add_argument("--expo", type=float, default=0.0)
    ap.add_argument("--rate", type=float, default=50.0, help="发送频率 Hz")
    ap.add_argument("--local-port", type=int, default=LOCAL_PORT,
                    help="本地 UDP 端口（默认 14551；切勿用 14550，那是 QGC 的）")
    ap.add_argument("--no-mode", action="store_true", help="不自动切 GROUND_DRIVE")
    ap.add_argument("--arm-btn", type=int, default=None, help="解锁/上锁按钮编号")
    ap.add_argument("--mode-btn", type=int, default=None, help="切 GROUND_DRIVE 的按钮编号")
    ap.add_argument("--servo", type=float, default=None,
                    help="周期发送摆臂指令：1.0=52°(地面行驶位), 0.85≈48°, -1.0=0°。"
                         "不设此项则脚本不碰摆臂，保持飞控默认位置（可用 QGC Actions 切换）")
    ap.add_argument("--rc", action="store_true",
                    help="走正统 RC 通路：发 RC_CHANNELS_OVERRIDE 而不是 MANUAL_CONTROL")
    ap.add_argument("--rc-throttle-axis", type=int, default=None,
                    help="把某个轴接到 CH3(油门)。⚠️ 该杆不回中，停最低位时=满倒车")
    ap.add_argument("--rc-flight", action="store_true",
                    help="飞行用：CH3←油门杆(axis2)、CH4←方向杆(axis3) 全部接上。"
                         "⚠️ 地面模式千万不要用这个（油门杆停最低 = 满倒车）")
    args = ap.parse_args()

    js = Joystick(args.dev)
    link = Link(args.local_port) if args.send else None

    print("=" * 74)
    print("joystick_drive.py  ·  %s" % args.dev)
    print("  前进轴 = axis%s   转向轴 = axis%s   %s" %
          (args.axis_fwd, args.axis_steer,
           "油门 axis2 已叠加" if args.use_throttle else "油门 axis2 忽略"))
    if args.send:
        print("  通路：%s" % ("RC_CHANNELS_OVERRIDE（正统 RC）" if args.rc
                             else "MANUAL_CONTROL（手柄语义）"))
        print("  模式：**发送中** → 127.0.0.1:%d  (本地端口 %d)   %s" %
              (PORT, args.local_port,
               "自动切 GROUND_DRIVE" if not args.no_mode else "不自动切模式"))
        if args.servo is not None:
            print("  摆臂：310(免解锁)+187 周期发送 %.2f → 约 %d°" %
                  (args.servo, int(round(26 * (args.servo + 1)))))
        else:
            print("  摆臂：不控制（保持飞控默认，需行驶时用 QGC Actions 打到 52°）")
        if args.rc_flight:
            print("  通道：CH1副翼 CH2升降 CH3油门 CH4方向 —— **飞行模式**")
        elif args.rc:
            print("  通道：CH1副翼 CH2升降 CH3/CH4 恒定中位 —— **地面模式**")
    else:
        print("  模式：**只读预览**（不发任何数据）。确认数值正确后加 --send")
    print("  Ctrl+C 退出（会先把油门归零）")
    print("=" * 74)

    if args.list:
        print("\n【标定模式】推杆/按键，下面会实时列出编号。Ctrl+C 结束。\n")

    fwd = steer = 0.0
    throttle_extra = 0.0
    t_last = 0.0
    t_print = 0.0
    t_servo = 0.0
    t_act = 0.0
    period = 1.0 / max(1.0, args.rate)

    try:
        while True:
            pressed = js.poll()
            now = time.time()

            if args.list:
                # 标定模式：任何变化都打印
                if js.axes:
                    line = "  " + "  ".join(
                        "a%d=%6d" % (k, v) for k, v in sorted(js.axes.items()))
                    print("\r" + line, end="", flush=True)
                for b in pressed:
                    print("\n  ▶ 按钮 %d 按下" % b)
                time.sleep(0.02)
                continue

            fwd = shape(js.axis(args.axis_fwd), args.deadband, args.expo)
            steer = shape(js.axis(args.axis_steer), args.deadband, args.expo)
            if args.invert_fwd:
                fwd = -fwd
            if args.invert_steer:
                steer = -steer

            throttle_extra = shape(js.axis(2), args.deadband, args.expo) if args.use_throttle else 0.0
            total_fwd = max(-1.0, min(1.0, fwd + throttle_extra))

            # ── 按钮 ──
            for b in pressed:
                if args.arm_btn is not None and b == args.arm_btn:
                    want = not (link.armed if link else False)
                    if link:
                        link.arm(want)
                    print("\n  ⚑ 按钮 %d → %s" % (b, "解锁" if want else "上锁"))
                if args.mode_btn is not None and b == args.mode_btn:
                    if link:
                        link.set_mode_ext1()
                    print("\n  ⚑ 按钮 %d → 切 GROUND_DRIVE" % b)
                if args.list is False and args.arm_btn is None and args.mode_btn is None:
                    print("\n  ⚑ 按钮 %d（用 --list 可查看编号）" % b)

            # ── 发送 ──
            if link:
                link.heartbeat(now)
                if now - t_last >= period:
                    if args.rc:
                        # 轴(-1..1) → 通道脉宽(1000..2000 μs)，1500 = 中位
                        ch = [1500] * 18
                        ch[0] = 1500 + int(round(500 * steer))   # CH1 副翼 → roll
                        ch[1] = 1500 + int(round(500 * total_fwd))  # CH2 升降 → pitch
                        if args.rc_flight:
                            # 飞行用：CH3 接油门杆（不回中，停最低 = 1000μs）
                            #         CH4 接方向杆
                            thr = shape(js.axis(2), args.deadband, args.expo)
                            yaw = shape(js.axis(3), args.deadband, args.expo)
                            ch[2] = 1500 + int(round(500 * thr))
                            ch[3] = 1500 + int(round(500 * yaw))
                        elif args.rc_throttle_axis is not None:   # CH3 单独接
                            thr = shape(js.axis(args.rc_throttle_axis),
                                        args.deadband, args.expo)
                            ch[2] = 1500 + int(round(500 * thr))
                        # 否则 CH3/CH4 保持 1500 中位（地面模式：throttle=0, yaw=0）
                        link.rc_override(ch)
                    else:
                        link.manual_control(total_fwd, steer, 0.0)
                    if not args.no_mode and now - link.t_mode >= 2.0:
                        link.set_mode_ext1()
                        link.t_mode = now
                    t_last = now
                # 摆臂指令必须周期重发，而且要发【两种】：
                #   310  MAV_CMD_ACTUATOR_TEST(p1=值, p5=1000+301) → 免解锁
                #   187  MAV_CMD_DO_SET_ACTUATOR(p1=值)           → 需 armed/prearmed
                # ⚠️ 只发 187 会死锁：未解锁时 mixer_module.cpp:461 把输出变 NAN
                #    → 回落 DIS 值(0°) → 永远到不了 48°，而 ground_drive 要求
                #    ≥48° 才允许 arm。310 有专门的后门能绕过这个门控。
                if args.servo is not None:
                    if now - t_act >= 0.2:
                        link.command_long(310, args.servo, 0.0, 0.0, 0.0, 1301.0)
                        t_act = now
                    if now - t_servo >= 0.5:
                        link.command_long(187, args.servo)
                        t_servo = now
                link.drain()

            # ── 显示 ──
            if now - t_print >= 0.1:
                bar_f = "#" * int(abs(total_fwd) * 10)
                bar_s = "#" * int(abs(steer) * 10)
                state = ""
                if link:
                    state = "  armed=%s" % {True: "是", False: "否", None: "?"}[link.armed]
                print("\r  前进 %+0.2f %-10s | 转向 %+0.2f %-10s%s" %
                      (total_fwd, bar_f, steer, bar_s, state), end="", flush=True)
                t_print = now

    except KeyboardInterrupt:
        print("\n\n正在归零…")
        if link:
            for _ in range(5):
                link.heartbeat(time.time())
                link.manual_control(0.0, 0.0, 0.0)
                time.sleep(0.02)
        js.close()
        print("已断开。")


if __name__ == "__main__":
    main()
