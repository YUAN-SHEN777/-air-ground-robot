#!/usr/bin/env python3
"""
verify_steer.py —— P5 转向 / 速度标定台（GROUND_DRIVE 仿真）

为什么需要它
------------
`verify_reversible.py` 只测**位移**，回答不了「转了多少度」。
P5 要定的三件事（左右映射、转向符号、转向增益）全部依赖**偏航角**，
所以单独做一个台子，复用它的 Link（MAVLink）与 gz_pose（位姿）。

测量原理
--------
1. 位姿：`gz topic -e -t /world/airground/pose/info -n 1`
   取 `name: "airground_0"` 之后的 position + orientation（四元数）→ yaw。
   ⚠️ 该话题包含世界内**所有**实体，必须先按名字切片再正则。

2. 电机实际输出：`gz topic -e -t /airground_0/command/motor_speed -n 1`
   给的是 mixer 之后的 `velocity`（0..1000，500 = 停）。
   1000 或 0 就是**饱和**，这比读 pxh 方便。

3. ⚠️ 读话题会阻塞 0.3~1 s。这段时间必须**继续发 MANUAL_CONTROL**，
   否则 `manual_control_signal_lost` 会让 `mcs.valid=false` → 输出清零 →
   测量被打断。所以用后台线程 30 Hz 持续 pump。

判据（写死在每段里，先定判据再测，避免「测出数就编解释」）
----------------------------------------------------------
S1  纯转向 ±0.5      |Δyaw| ≈ GD_YAW_GAIN·0.5·t；两段严格反号
S2  前进+转向        Δyaw 与横向位移必须自洽（右转 → 右偏）
S3  直线 10 s        横向漂移 < 3% 行程；|Δyaw| < 2°
S4  满油门           速度误差 < 10%；ctrl 是否到 1.0
S5  满油门+满转向    转弯半径 vs 理论（饱和会明显变大）
S6  低速扫掠         找出能起步的最小油门

用法
----
  python3 verify_steer.py              # 全跑
  python3 verify_steer.py --only S1    # 只跑某一段
  python3 verify_steer.py --only S1,S3
"""

import argparse
import math
import re
import subprocess
import threading
import time

from verify_reversible import Link, SERVO_GROUND

MODEL = "airground_0"
WORLD = "airground"
POSE_TOPIC = f"/world/{WORLD}/pose/info"
MOTOR_TOPIC = f"/{MODEL}/command/motor_speed"

# 与本项目 airframe / module.yaml 保持一致的标称值（仅用于算判据）
GD_MAX_SPEED = 1.2      # airframe: param set GD_MAX_SPEED 1.2（早期 0.6，已改）
GD_YAW_GAIN = 1.5
HALF_TRACK = 0.165 / 2


# ─────────────────────────── 位姿 / 电机读数 ───────────────────────────
def _gz_echo(topic, timeout=8, n=1):
    try:
        return subprocess.run(["gz", "topic", "-e", "-t", topic, "-n", str(n)],
                              capture_output=True, text=True, timeout=timeout).stdout
    except Exception:
        return ""


def _sim_t(txt):
    """取消息头里的仿真时间（秒）。用仿真时间做时基，
    这样 gz 子进程启动开销不会污染差分。"""
    m = re.search(r"stamp\s*\{\s*sec:\s*(\d+)\s*nsec:\s*(\d+)", txt)
    return int(m.group(1)) + int(m.group(2)) * 1e-9 if m else None


def gz_state(model=MODEL):
    """返回 (x, y, z, yaw, t)；yaw ∈ (-pi, pi]"""
    out = _gz_echo(POSE_TOPIC)
    idx = out.find(f'name: "{model}"')
    if idx < 0:
        return None
    seg = out[idx:]

    mp = re.search(r"position\s*\{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)\s*z:\s*([-\d.eE+]+)", seg)
    mq = re.search(r"orientation\s*\{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)\s*z:\s*([-\d.eE+]+)\s*w:\s*([-\d.eE+]+)", seg)
    if not mp or not mq:
        return None

    x, y, z = (float(v) for v in mp.groups())
    qx, qy, qz, qw = (float(v) for v in mq.groups())
    yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
    return (x, y, z, yaw, _sim_t(out))


def gz_yaw_series(n=30, model=MODEL):
    """一次抓 n 帧 pose/info → [(t, x, y, yaw)]。
    用来看偏航是「稳定转」还是「间歇卡死」，比首尾两点可靠得多。

    ⚠️ 两个坑：
    1. `header { stamp }` 是**整条消息一个**（gz.msgs.Pose_V），不在每个 pose 块里
       → 必须先按 `header {` 切消息，再按 `pose {` 切实体。
    2. pose/info 是全实体齐发（一次 ~320 行），帧数不能取大。
    """
    out = _gz_echo(POSE_TOPIC, n=n, timeout=15)
    series = []

    for msg in out.split("header {")[1:]:
        t = _sim_t(msg)
        if t is None:
            continue

        for chunk in msg.split("pose {")[1:]:
            if f'name: "{model}"' not in chunk:
                continue

            mp = re.search(r"position\s*\{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)\s*z:\s*([-\d.eE+]+)", chunk)
            mq = re.search(r"orientation\s*\{\s*x:\s*([-\d.eE+]+)\s*y:\s*([-\d.eE+]+)\s*z:\s*([-\d.eE+]+)\s*w:\s*([-\d.eE+]+)", chunk)
            if not (mp and mq):
                continue

            qx, qy, qz, qw = (float(v) for v in mq.groups())
            yaw = math.atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz))
            series.append((t, float(mp.group(1)), float(mp.group(2)), yaw))

    return series


def gz_motors():
    """返回 mixer 之后的 4 路 velocity（0..1000）；失败返回 None"""
    out = _gz_echo(MOTOR_TOPIC)
    vals = [float(v) for v in re.findall(r"velocity:\s*([-\d.eE+]+)", out)]
    return vals[:4] if vals else None


WHEELS = ["joint7", "joint8", "joint9", "joint10"]


def _joint_snapshot():
    """返回 (t_sim, {name: position})"""
    out = _gz_echo("/airground/joint_state")
    d = {}

    for chunk in out.split("joint {")[1:]:
        mn = re.search(r'name:\s*"([^"]+)"', chunk)
        mp = re.search(r"axis1\s*\{.*?position:\s*([-\d.eE+]+)", chunk, re.S)
        if mn and mp:
            d[mn.group(1)] = float(mp.group(1))

    return _sim_t(out), d


def gz_wheel_omega(min_dt=0.10):
    """差分求轮关节角速度（rad/s），用仿真时间戳做时基。
    ⚠️ 不能对角度做 ±π 归一化：连续关节在 DART 里是不回绕的累计角。"""
    t0, a0 = _joint_snapshot()
    time.sleep(min_dt)
    t1, a1 = _joint_snapshot()

    if not a0 or not a1 or t0 is None or t1 is None or t1 <= t0:
        return None

    dt = t1 - t0
    out = []
    for n in WHEELS:
        out.append((a1[n] - a0[n]) / dt if (n in a0 and n in a1) else float("nan"))
    return out

def gz_arm_deg():
    """摆臂实际角度（deg）—— mux 的切换阈值是 48°，低于它就回到飞行位"""
    _, p = _joint_snapshot()
    return math.degrees(p["servo_0"]) if "servo_0" in p else None


# ─────────────────────────── 后台 pump ───────────────────────────
class Pump:
    """测量期间必须持续发 MANUAL_CONTROL：读话题会阻塞 ~1 s，
    期间若断了摇杆，mcs.valid 会掉成 false → 输出清零 → 测量作废。"""

    def __init__(self, link, hz=30):
        self.link = link
        self.period = 1.0 / hz
        self._stop = threading.Event()
        self._t = threading.Thread(target=self._run, daemon=True)

    def _run(self):
        while not self._stop.is_set():
            self.link.pump()
            self._stop.wait(self.period)

    def __enter__(self):
        self._t.start()
        return self

    def __exit__(self, *a):
        self._stop.set()
        self._t.join(timeout=2.0)


# ─────────────────────────── 测量 ───────────────────────────
def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def phase(link, title, throttle, steer, secs, settle=1.2):
    """设定摇杆 → 稳定 → 测一段。返回 dict。时基全部用仿真时间戳。"""
    link.throttle = throttle
    link.steer = steer
    time.sleep(settle)                     # pump 在跑，指令生效

    s0 = gz_state()
    m = gz_motors()

    time.sleep(secs)

    s1 = gz_state()

    if s0 is None or s1 is None or s0[4] is None or s1[4] is None:
        print(f"  {title:<30} 位姿读取失败")
        return None

    dt = s1[4] - s0[4]
    dx, dy = s1[0] - s0[0], s1[1] - s0[1]
    dist = math.hypot(dx, dy)
    dyaw = wrap(s1[3] - s0[3])

    # 沿「初始航向」分解：纵向 = 应走的，横向 = 漂移
    yaw0 = s0[3]
    lon = math.cos(yaw0) * dx + math.sin(yaw0) * dy
    lat = -math.sin(yaw0) * dx + math.cos(yaw0) * dy   # 左为正

    r = dict(title=title, thr=throttle, steer=steer, secs=dt,
             dx=dx, dy=dy, dist=dist, lon=lon, lat=lat,
             dyaw=math.degrees(dyaw), motors=m)

    print(f"  {title:<30} |Δ|={dist:5.3f} m  纵={lon:+6.3f} 横={lat:+6.3f}  "
          f"Δyaw={r['dyaw']:+7.1f}°  电机={fmt_motors(m)}")
    return r


def fmt_motors(m):
    if not m:
        return "?"
    return "[" + ",".join(f"{v:.0f}" for v in m) + "]"


def setup(link):
    """摆臂 52° → 切 ext1 → 解锁"""
    link.servo = SERVO_GROUND
    link.arm(False, force=True)
    link.hold(2.0)

    link.hold(3.0)                 # 310 把摆臂推过去（会超时，靠两个命令一起发）
    link.keep_mode = True
    link.set_mode()
    link.t_mode = time.time()
    link.hold(6.0)                 # 等模式生效 + 摆臂到位

    link.arm(True)
    link.wait_armed(True, timeout=20.0)


def stop(link, secs=2.0):
    link.throttle = 0.0
    link.steer = 0.0
    link.hold(secs)


# ─────────────────────────── 各段 ───────────────────────────
def s1_symbol(link):
    print("\nS1 纯转向 —— 判「+roll 是不是右转」")
    print("   判据：两段严格反号；|Δyaw| ≈ GD_YAW_GAIN·0.5·2s = 86°")
    a = phase(link, "steer +0.5", 0.0, +0.5, 2.0)
    stop(link, 1.5)
    b = phase(link, "steer -0.5", 0.0, -0.5, 2.0)
    stop(link, 1.5)

    if a and b:
        print("\n   ⇒ ", end="")
        if a["dyaw"] * b["dyaw"] >= 0:
            print(f"❌ 两段同号（{a['dyaw']:+.1f} / {b['dyaw']:+.1f}）—— 转向轴接错，先查 mcs.roll")
        elif a["dyaw"] < 0:
            print(f"✅ +roll → Δyaw {a['dyaw']:+.1f}°（右转）—— 符号正确，无需改动")
        else:
            print(f"🔴 +roll → Δyaw {a['dyaw']:+.1f}°（左转）—— **符号反了**，需把 GD_YAW_GAIN 取负")
        eff = abs(a["dyaw"]) / 86.0
        print(f"      效率 = |Δyaw|/理论 = {eff*100:.0f}%（远低于 50% 说明原地转严重打滑）")
    return a, b


def s2_side(link):
    print("\nS2 前进 + 转向 —— 判左右映射是否自洽")
    print("   判据：右转（Δyaw<0）时横向位移应向右（横 < 0）")
    r = phase(link, "thr 0.4 / steer +0.5", 0.4, +0.5, 2.0)
    stop(link, 1.5)

    if r:
        consistent = (r["dyaw"] < 0) == (r["lat"] < 0)
        print(f"\n   ⇒ {'✅ 自洽' if consistent else '❌ 不自洽（转向与位移方向矛盾）'}"
              f"  Δyaw={r['dyaw']:+.1f}° 横={r['lat']:+.3f} m")
        v = r["lon"] / r["secs"]
        w = math.radians(r["dyaw"]) / r["secs"]
        if abs(w) > 0.05:
            print(f"      实测 v={v:+.3f} m/s  w={w:+.3f} rad/s  转弯半径 R={abs(v/w):.2f} m")
    return r


def s3_straight(link, secs=10.0):
    print("\nS3 直线一致性")
    print("   判据：横向漂移 < 3% 行程；|Δyaw| < 2°")
    r = phase(link, f"thr 0.4 / steer 0 / {secs:.0f}s", 0.4, 0.0, secs)
    stop(link, 1.5)

    if r:
        drift = abs(r["lat"]) / max(r["dist"], 1e-6) * 100
        ok = drift < 3.0 and abs(r["dyaw"]) < 2.0
        print(f"\n   ⇒ {'✅' if ok else '❌'} 横向漂移 {r['lat']:+.3f} m = {drift:.1f}%  "
              f"Δyaw={r['dyaw']:+.2f}°  v={r['lon']/r['secs']:.3f} m/s")
    return r


def s4_full(link):
    print("\nS4 满油门直行")
    print("   判据：速度误差 < 10%（期望 0.6 m/s）；电机应接近 1000 但不长期饱和")
    r = phase(link, "thr 1.0 / steer 0 / 5s", 1.0, 0.0, 5.0)
    stop(link, 1.5)

    if r:
        v = r["lon"] / r["secs"]
        err = (v - GD_MAX_SPEED) / GD_MAX_SPEED * 100
        ok = abs(err) < 10.0
        print(f"\n   ⇒ {'✅' if ok else '⚠️'} v={v:.3f} m/s（期望 {GD_MAX_SPEED}，偏差 {err:+.1f}%）")
        if r["motors"] and max(r["motors"]) >= 999:
            print(f"      电机已达满量程 {max(r['motors']):.0f} —— 上限由 GD_MAX_SPEED 决定，不是转速")
    return r


def s5_turnfull(link, secs=2.0):
    print("\nS5 满油门 + 满转向 —— 判饱和对轨迹的影响")
    print("   理论（不饱和）R = v/w = 0.6/1.5 = 0.40 m")
    print("   若 R 明显 > 0.40 m 说明外侧被削（当前「两侧独立限幅」的后果）")
    r = phase(link, f"thr 1.0 / steer 1.0 / {secs:.0f}s", 1.0, 1.0, secs)
    stop(link, 1.5)

    if r:
        dist = r["dist"]
        dyaw = abs(math.radians(r["dyaw"]))
        R = dist / dyaw if dyaw > 1e-3 else float("inf")
        print(f"\n   ⇒ 实测轨迹 {dist:.3f} m / {math.degrees(dyaw):.1f}°"
              f"  → R ≈ {R:.2f} m（理论 0.40）")
        print(f"      电机={fmt_motors(r['motors'])}"
              f"  {'← 有通道打到 1000：已饱和' if r['motors'] and max(r['motors']) >= 999 else ''}")
    return r


def s6_low(link):
    print("\nS6 低速扫掠 —— 找起步死区")
    print("   判据：能稳定走起来（|Δ| > 50% 理论）的最小油门")
    out = []
    for thr in (0.05, 0.10, 0.15, 0.20, 0.30):
        r = phase(link, f"thr {thr:.2f} / 3s", thr, 0.0, 3.0, settle=1.0)
        stop(link, 1.0)
        if r:
            expect = GD_MAX_SPEED * thr * r["secs"]
            ratio = r["dist"] / expect if expect > 1e-6 else 0
            out.append((thr, r["dist"], ratio))
            print(f"      期望 {expect:.3f} m → 实际 {r['dist']:.3f} m（{ratio*100:.0f}%）")

    good = [t for t, _, ratio in out if ratio > 0.5]
    print(f"\n   ⇒ 可用下限 ≈ {min(good) if good else '>0.30'} 油门")
    return out


# ─────────────────────────── 主流程 ───────────────────────────
ALL = ["PROBE", "S1", "S2", "S3", "S4", "S5", "S6"]


def probe(link, secs=3.0):
    """轮速探针 —— 判断转向失效是「打滑」「没下发」还是「间歇卡死」

    每段保持摇杆 secs 秒，期间读：电机输出 / 轮关节角速度 / 摆臂角度 /
    偏航时间序列（看是稳定转还是走走停停）。
    理论轮速：|signed|·(1000/64)，满舵 ±1.5625 rad/s。
    """
    print("\nPROBE 轮速 / 偏航 对照（参考：满舵理论轮速 3.09 rad/s、理论偏航 1.5 rad/s）")
    cases = [("纯转 steer +1.0", 0.0, +1.0),
             ("纯转 steer -1.0", 0.0, -1.0),
             ("纯转 steer +0.5", 0.0, +0.5),
             ("纯转 steer +0.75", 0.0, +0.75),
             ("直行 thr 0.4", 0.4, 0.0),
             ("前行+满舵", 0.4, +1.0)]

    for title, thr, steer in cases:
        link.throttle, link.steer = thr, steer
        time.sleep(1.5)                       # 先让它「破静摩擦」

        m = gz_motors()
        w0 = gz_wheel_omega(0.12)
        arm = gz_arm_deg()
        s0 = gz_state()
        time.sleep(secs)
        s1 = gz_state()

        if s0 is None or s1 is None or s0[4] is None or s1[4] is None:
            print(f"  {title:<18} 位姿读取失败")
            continue

        dt = s1[4] - s0[4]
        dyaw = math.degrees(wrap(s1[3] - s0[3]))
        rate = math.radians(dyaw) / dt if dt > 0 else 0.0
        armt = f"{arm:5.1f}°" if arm is not None else "  ?  "
        print(f"  {title:<18} 电机={fmt_motors(m)}  臂={armt}  轮速={fmt_omega(w0)}  "
              f"Δyaw={dyaw:+6.1f}°/{dt:.1f}s → {rate:+.3f} rad/s")

        link.throttle = link.steer = 0.0
        time.sleep(1.0)


def unwrap_rate(ser):
    """把 yaw 序列逐帧增量累加（unwrap），返回 (总转角 rad, 角速率 rad/s, 时长 s)。
    ⚠️ 必须逐帧累加：直接 `wrap(末-初)` 在转过多圈时会折回 0，把大转动测成「完全没转」。"""
    if len(ser) < 3:
        return None, None, 0.0

    acc = 0.0
    prev = ser[0][3]

    for _t, _x, _y, yaw in ser[1:]:
        acc += wrap(yaw - prev)
        prev = yaw

    dt = ser[-1][0] - ser[0][0]
    return acc, (acc / dt if dt > 1e-6 else 0.0), dt


def scan_turn_authority(link, steer=1.0, n=90, settle=1.5):
    """扫描「同舵角下，车速对偏航的影响」（用时间序列，抗多圈折返）。

    动机：本项目 L(0.21 m) > W(0.165 m)，前后轴侧向力力臂大于纵向差速力臂
    → 强不足转向。若降速能明显恢复偏航，就可以用「转弯自动减速」来补。
    """
    print(f"\nSCAN 转向权限 vs 车速（steer={steer:+.1f}，理论偏航 {GD_YAW_GAIN*steer:+.2f} rad/s）")
    print("   thr    电机              臂      轮速[rad/s]             实测w      效率")

    for thr in (0.0, 0.1, 0.2, 0.4, 0.7, 1.0):
        link.throttle, link.steer = thr, steer
        time.sleep(settle)

        m = gz_motors()
        wl = gz_wheel_omega(0.12)
        arm = gz_arm_deg()
        ser = gz_yaw_series(n=n)

        acc, rate, dt = unwrap_rate(ser)
        if rate is None:
            print(f"   {thr:.2f}   读取失败（{len(ser)} 帧）")
            link.throttle = link.steer = 0.0
            time.sleep(1.0)
            continue

        theory = GD_YAW_GAIN * steer
        eff = abs(rate / theory) * 100 if theory else 0
        armt = f"{arm:5.1f}°" if arm is not None else "  ?  "
        print(f"   {thr:.2f}   {fmt_motors(m):<16} {armt}  {fmt_omega(wl):<23} "
              f"{rate:+.3f}   {eff:3.0f}%   ({math.degrees(acc):+.0f}°/{dt:.2f}s)")

        link.throttle = link.steer = 0.0
        time.sleep(1.0)


def probe_series(link, thr, steer, secs=3.0):
    """偏航时间序列 —— 区分「稳定转动」与「间歇卡死 / 抖动」"""
    link.throttle, link.steer = thr, steer
    time.sleep(1.5)
    ser = gz_yaw_series(n=30)

    if len(ser) < 4:
        print(f"  时间序列读取失败（{len(ser)} 帧）")
        return

    t0, x0, y0, yaw0 = ser[0]
    print(f"  thr={thr:+.2f} steer={steer:+.2f}  共 {len(ser)} 帧 / "
          f"{(ser[-1][0]-t0):.2f}s（{len(ser)/max(ser[-1][0]-t0,1e-6):.1f} Hz）")
    print("      t[s]   Δyaw[°]   Δ航向速率[rad/s]")
    prev_t, prev_y = t0, yaw0
    for t, _x, _y, yaw in ser:
        if t - prev_t < 0.15:            # 抽稀，避免刷屏
            continue
        dy_total = math.degrees(wrap(yaw - yaw0))
        rate = wrap(yaw - prev_y) / max(t - prev_t, 1e-6)   # wrap 已返回弧度，别再 radians()
        print(f"     {t-t0:6.2f}  {dy_total:+8.1f}   {rate:+.3f}")
        prev_t, prev_y = t, yaw
    link.throttle = link.steer = 0.0
    time.sleep(1.2)


def fmt_omega(w):
    if not w:
        return "?"
    return "[" + ",".join(f"{v:+.2f}" for v in w) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--only", default="", help="逗号分隔，如 S1,S3；空=全跑")
    args = ap.parse_args()

    only = [s.strip().upper() for s in args.only.split(",") if s.strip()] or ALL
    for s in only:
        if s not in ALL:
            print(f"未知段 {s}")
            return 1

    link = Link()

    print("=" * 78)
    print("P5 转向 / 速度标定")
    print(f"  标称值：GD_MAX_SPEED={GD_MAX_SPEED}  GD_YAW_GAIN={GD_YAW_GAIN}  "
          f"半轮距={HALF_TRACK:.4f} m")
    print("=" * 78)

    with Pump(link):
        print("\n① 摆臂 52° → ext1 → 解锁")
        setup(link)
        print("   就绪")

        if "PROBE" in only:
            probe(link)
        if "S1" in only:
            s1_symbol(link)
        if "S2" in only:
            s2_side(link)
        if "S3" in only:
            s3_straight(link)
        if "S4" in only:
            s4_full(link)
        if "S5" in only:
            s5_turnfull(link)
        if "S6" in only:
            s6_low(link)

        print("\n② 停车 / 上锁")
        stop(link, 2.0)
        link.arm(False, force=True)
        link.hold(2.0)

    print("\n完成。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
