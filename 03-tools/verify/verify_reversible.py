#!/usr/bin/env python3
"""
verify_reversible.py —— 验证「中位=停」的可逆语义（DShot 3D 对齐）是否生效

要验证的四件事
--------------
  A. 未解锁 + 摆臂 52°（轮位）→ 机器人【必须不动】（这就是 bug ①）
  B. 已解锁 + 油门 +0.5      → 前进（Δx > 0）
  C. 已解锁 + 油门 −0.5      → 倒车（Δx < 0）
  D. 解锁中 → 上锁后再看     → 立刻停

判定依据
--------
  位移从 Gazebo 的 /world/airground/pose/info 里读 airground_0 的 position。
  不用 PX4 的 local_position：地面模式下 EKF 不一定收敛，而且我们要看的是
  「物理上到底动没动」。

用法
----
  python3 verify_reversible.py
"""

import os
import re
import socket
import subprocess
import time

from pymavlink.dialects.v20 import common as mav_common

PORT = 18570
LOCAL_PORT = 14550            # = PX4 Normal 链路的 remote port
ACT_FUNC = 1301.0          # 1000 + SIM_GZ_SV_FUNC1(301)
SERVO_GROUND = 1.0         # 舵机指令 +1 → 52°（轮位）
NAV_EXTERNAL1 = 23


# ─────────────────────────── Gazebo 位姿 ───────────────────────────
def gz_pose(model="airground_0"):
    """返回 (x, y, z)；读不到就返回 None"""
    try:
        out = subprocess.run(
            ["gz", "topic", "-e", "-t", "/world/airground/pose/info", "-n", "1"],
            capture_output=True, text=True, timeout=8).stdout
    except Exception:
        return None

    # 找到 name: "airground_0" 之后紧跟的 position 块
    idx = out.find(f'name: "{model}"')
    if idx < 0:
        return None

    m = re.search(r"position\s*\{\s*x:\s*([-\d.e+]+)\s*y:\s*([-\d.e+]+)\s*z:\s*([-\d.e+]+)",
                  out[idx:])
    if not m:
        return None

    return tuple(float(g) for g in m.groups())


# ─────────────────────────── MAVLink ───────────────────────────
class Link:
    """自己管 UDP socket：固定本地端口 14550（= PX4 的 remote port）。

    为什么不用 mavutil.mavlink_connection("udpout:...")：
      udpout 绑的是随机源端口，PX4 只在「认识这个客户端」后才回发心跳。
      实测第二次运行起就收不到心跳，manual_control_signal_lost 恒为 True，
      于是永远进 failsafe（切 Return），我们的外部模式被顶掉。
      固定端口后 PX4 的回发地址 == 我们的源地址，链路稳定。
    """

    MAV_TYPE_GCS = 6
    MAV_AUTOPILOT_INVALID = 8
    ARMED_BIT = 128

    def __init__(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("127.0.0.1", LOCAL_PORT))
        self.sock.setblocking(False)
        self.dest = ("127.0.0.1", PORT)

        self.mav = mav_common.MAVLink(None, srcSystem=255, srcComponent=0)
        self.mav.robust_parsing = True

        self.armed = None
        self.hb_count = 0

        self.keep_mode = False       # True 时周期重发外部模式指令
        self.t_mode = 0.0

        self.throttle = 0.0
        self.steer = 0.0
        self.servo = SERVO_GROUND
        self.t_heart = 0.0
        self.t_310 = 0.0
        self.t_187 = 0.0

    # ── 低层收发 ──
    def _send(self, msg):
        try:
            self.sock.sendto(msg.pack(self.mav), self.dest)
        except OSError:
            pass

    def _drain(self):
        while True:
            try:
                data, _ = self.sock.recvfrom(65535)
            except (BlockingIOError, OSError):
                return

            for m in self.mav.parse_buffer(data) or []:
                if m.get_type() == "HEARTBEAT" and m.get_srcSystem() != 255:
                    self.hb_count += 1
                    self.armed = bool(m.base_mode & self.ARMED_BIT)

    def _cmd_long(self, command, p1=0.0, p2=0.0, p3=0.0,
                  p4=0.0, p5=0.0, p6=0.0, p7=0.0):
        self._send(mav_common.MAVLink_command_long_message(
            1, 1, command, 0, p1, p2, p3, p4, p5, p6, p7))

    # ── 周期发送 ──
    def pump(self):
        """周期发：心跳 10 Hz / MANUAL_CONTROL 10 Hz / 310 5 Hz / 187 2 Hz"""
        now = time.time()

        if now - self.t_heart > 0.1:
            self._send(mav_common.MAVLink_heartbeat_message(
                self.MAV_TYPE_GCS, self.MAV_AUTOPILOT_INVALID, 0, 0, 0, 3))
            # v20 common 的 MANUAL_CONTROL 有 6 个扩展字段，必须全部给出
            self._send(mav_common.MAVLink_manual_control_message(
                1, 0, int(round(1000 * self.steer)),
                int(round(500 + 500 * self.throttle)), 0, 0,
                0, 0, 0, 0, 0, 0, 0, 0, 0, 0))
            self.t_heart = now

        if now - self.t_310 > 0.2:
            self._cmd_long(310, self.servo, 0, 0, 0, ACT_FUNC)
            self.t_310 = now

        if now - self.t_187 > 0.5:
            self._cmd_long(187, self.servo)
            self.t_187 = now

        # ⚠️ 模式必须【周期重发】：
        #   GCS 链路一断 PX4 就进 failsafe 切 Return，我们切好的 ext1 被顶掉。
        #   而且它会把 ext1 记成 user_intended，等 failsafe 解除后才真正生效 ——
        #   所以只发一次会导致「前几秒没驱动、后面才突然开始动」。
        if self.keep_mode and now - self.t_mode > 2.0:
            self.set_mode()
            self.t_mode = now

        self._drain()

    def set_mode(self, _nav_state=23):
        """切到外部模式 1（GROUND_DRIVE）。
        PX4 的编码：param1=CUSTOM_MODE_ENABLED(1)，param2=MAIN_MODE_AUTO(4)，
        param3=PX4_CUSTOM_SUB_MODE_EXTERNAL1(11)。
        （直接传 nav_state 是旧写法，会被拒）"""
        self._cmd_long(176, 1, 4, 11)

    def arm(self, on, force=False):
        # param2 = 21196 是 MAVLink 的「强制」魔术值：
        # 不解锁时 PX4 会以「in flight / not landed」拒绝 disarm，
        # 而一旦卡在 armed + 非本模式，我们的门控会永久拒绝进入 ext1。
        self._cmd_long(400, 1 if on else 0, 21196 if force else 0)

    def wait_armed(self, want, timeout=4.0):
        """等待 armed 状态。
        ⚠️ 实测 PX4 不会把数据回发到本机（原始 socket 8 s 收 0 字节），
        所以 armed 位常常读不到；这里超时就当「未知」，靠后续位移判定。"""
        deadline = time.time() + timeout
        hb0 = self.hb_count

        while time.time() < deadline:
            self.pump()

            if self.armed == want:
                print(f"   状态确认：armed={want}")
                return True

            time.sleep(0.05)

        if self.hb_count == hb0:
            print("   （收不到回传心跳，按固定延时继续；结果看位移）")
        else:
            print(f"   ⚠️ armed={self.armed}，期望 {want}")

        return False

    def hold(self, secs):
        """持续发消息 secs 秒"""
        end = time.time() + secs
        while time.time() < end:
            self.pump()
            time.sleep(0.02)


# ─────────────────────────── 主流程 ───────────────────────────
def phase(title, link, secs, expect=""):
    """测量一段时间的平面位移，返回 (dx, dy)。
    ❗只测 Δx 是错的：车头朝向任意，实际位移可能是斜向的。"""
    p0 = gz_pose()
    link.hold(secs)
    p1 = gz_pose()

    if p0 is None or p1 is None:
        print(f"  {title:<26} 位姿读取失败")
        return None

    d = (p1[0] - p0[0], p1[1] - p0[1])
    dist = (d[0] ** 2 + d[1] ** 2) ** 0.5
    print(f"  {title:<26} Δ=({d[0]:+.2f},{d[1]:+.2f})  |Δ|={dist:.3f} m   {expect}")
    return d


def main():
    link = Link()

    print("① 把摆臂摆到 52°（轮位），切入 GROUND_DRIVE")
    link.servo = SERVO_GROUND

    # 先确保是【未】解锁状态（上一轮可能残留）
    # ⚠️ 必须强制：一旦卡在 armed + 非外部模式，我们的 arming gate
    #    （未激活时 armed → can_arm_and_run=false）会让模式永远切不进来。
    link.arm(False, force=True)
    link.hold(2.0)

    link.hold(3.0)                      # 310 先把摆臂摆过去
    link.keep_mode = True
    link.set_mode()
    link.t_mode = time.time()
    link.hold(6.0)                      # 等 failsafe 清除 + 模式生效 + 摆臂到位
    print("   已请求 mode=ext1（周期性重发，抗 failsafe）")

    print("\n② 四阶段验证（每阶段 5 s）")
    dA = phase("A 未解锁 + 油门 0", link, 5.0, "期望 ≈ 0")

    print("   → 解锁")
    link.arm(True)
    # ⚠️ 实测：解锁瞬间摆臂从 0°→52° 会把车体顶起来 → land detector 报
    #    「Takeoff detected」→ 触发 Hold 5 s 的 failsafe → 我们的模式被停用。
    #    等 failsafe 结束 + 周期重发 set_mode 把模式抢回来（约 10~15 s）。
    #    这段是「解锁瞬态」，不是编码问题。
    link.wait_armed(True, timeout=20.0)

    link.throttle = 0.5          # ★ 之前漏了这一行，B 段实际测的是「油门 0」
    dB = phase("B 已解锁 + 油门 +0.5", link, 5.0, "期望 > 0")

    link.throttle = -0.5
    dC = phase("C 已解锁 + 油门 -0.5", link, 5.0, "期望 > 0")

    print("   → 上锁")
    link.throttle = 0.0
    link.arm(False, force=True)
    link.wait_armed(False, timeout=6.0)
    dD = phase("D 已上锁 + 油门 0", link, 5.0, "期望 ≈ 0")

    # ── 正反判定：B 与 C 的位移应该【反号】 ──
    if dB and dC:
        dot = dB[0] * dC[0] + dB[1] * dC[1]
        nb = (dB[0] ** 2 + dB[1] ** 2) ** 0.5
        nc = (dC[0] ** 2 + dC[1] ** 2) ** 0.5
        cos = dot / (nb * nc) if nb * nc > 1e-9 else float("nan")
        verdict = "✅ 反向" if cos < -0.7 else ("⚠️ 未反向" if cos > 0.3 else "？不确定")
        print(f"\n③ 前进/倒车方向：cos(θ) = {cos:+.2f}  →  {verdict}")

    if dA and dD:
        na = (dA[0] ** 2 + dA[1] ** 2) ** 0.5
        nd = (dD[0] ** 2 + dD[1] ** 2) ** 0.5
        ok = "✅ 未解锁/上锁后确实不动" if (na < 0.05 and nd < 0.05) else "⚠️ 静止段有位移！"
        print(f"④ 静止验证：|A|={na:.3f} m，|D|={nd:.3f} m  →  {ok}")

    print("\n完成。")


if __name__ == "__main__":
    main()
