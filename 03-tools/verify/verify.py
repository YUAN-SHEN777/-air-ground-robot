#!/usr/bin/env python3
"""
verify.py —— 地面驱动统一验证入口

取代原来 6 个各自为战的脚本（verify_boost*.py / verify_dither.py /
verify_pivot.py / smoke_test.py，已归档到 `_deprecated/`）。

保留的两个底层文件：
    verify_reversible.py   Link 类（MAVLink + 固定 UDP 端口）+ 4 阶段位移验证
    verify_steer.py        S1~S6 转向/速度标定 + gz 话题读取函数

用法：
    python3 verify.py smoke            # 冒烟：直线 / 满舵 / 半舵（约 40 s）
    python3 verify.py repeat -n 10     # 重复性：同一指令跑 N 次看双稳态
    python3 verify.py scan             # 纵向速度 → 转向能力 扫描
    python3 verify.py gain             # GD_YAW_GAIN 扫描
    python3 verify.py pivot            # 转向时 v_cmd 归零 + 借动量
    python3 verify.py armangle --degs 52,45,38,35,34   # 摆臂折叠角扫描
    python3 verify.py steer            # 转发到 verify_steer.py（S1~S6 标定）
    python3 verify.py reversible       # 转发到 verify_reversible.py（4 阶段位移）

前提：PX4 SITL + Gazebo 已在运行。只需一条命令：
    cd ~/PX4-Autopilot && make px4_sitl gz_airground
它**永远**是以 `gz sim -r -s`（server only）启动的，不会开窗口；
`HEADLESS` / `PX4_GZ_HEADLESS` 这两个变量对 gz 都无效。
想看画面另开终端 `./gui.sh`。

⚠️ 两个必须知道的坑（都踩过）：
  1. 读 gz 话题会阻塞 0.3~1 s，期间必须继续发 MANUAL_CONTROL，
     否则 manual_control_signal_lost 会让输出清零 → 测量作废。
     所以所有「等待」都走 link.hold()，绝不用 time.sleep()。
  2. COM_DISARM_LAND 会把「轮子剧烈打滑」误判成起降并自动上锁，
     导致整组数据变成 [500,500,500,500]。脚本启动时会强制设 0。
"""

import argparse
import math
import statistics
import time

from pymavlink.dialects.v20 import common as mav_common

from verify_reversible import Link, SERVO_GROUND
from verify_steer import gz_state, gz_motors, wrap

# 与本项目 airframe / module.yaml 保持一致
PARAM_SLOW = "GD_TURN_SLOW"
PARAM_GAIN = "GD_YAW_GAIN"
PARAM_BOOST_GONE = ("GD_TURN_BOOST", "GD_TURN_STICK")   # P5 已删除

_MAV_PARAM_TYPE_REAL32 = 9
DEG = 57.2957795


# ─────────────────────────── 通用工具 ───────────────────────────
def set_param(link, name, value, settle=1.2):
    """设浮点参数。PX4 对类型不匹配会静默丢弃，所以显式给 REAL32。"""
    buf = name.encode() if isinstance(name, str) else name
    link._send(mav_common.MAVLink_param_set_message(
        1, 1, buf, float(value), _MAV_PARAM_TYPE_REAL32))
    link.hold(settle)


def setup(link, slow=1.0, gain=1.5):
    """摆臂 52° → 切 ext1 → 解锁，并把本次要用的参数钉死"""
    link.servo = SERVO_GROUND
    link.arm(False, force=True)
    link.hold(2.0)
    link.hold(3.0)                       # 310 把摆臂推过去
    link.keep_mode = True
    link.set_mode()
    link.t_mode = time.time()
    link.hold(6.0)                       # 等模式生效 + 摆臂到位
    link.arm(True)
    link.wait_armed(True, timeout=20.0)

    # 每次运行都把标定钉一遍：airframe 已显式写，这里只是防参数被上层改动
    set_param(link, "COM_DISARM_LAND", 0.0)
    set_param(link, PARAM_SLOW, slow)
    set_param(link, PARAM_GAIN, gain)


def stop(link, secs=2.5):
    link.throttle = 0.0
    link.steer = 0.0
    link.hold(secs)


def measure(link, title, throttle, steer, secs, settle=1.2, pre_thr=None, pre_secs=0.0):
    """设定摇杆 → 稳定 → 测一段。

    pre_thr / pre_secs 不为 0 时先直线预跑一段（用于「借动量」类测试）。
    返回 dict；位姿读取失败返回 None。
    """
    link.arm(True)                       # 防 COM_DISARM_LAND 把整组作废
    link.hold(0.3)

    if pre_secs > 0:
        link.throttle = pre_thr
        link.steer = 0.0
        link.hold(pre_secs)

    link.throttle = throttle
    link.steer = steer
    link.hold(settle)

    s0 = gz_state()
    m = gz_motors()

    link.hold(secs)

    s1 = gz_state()

    if s0 is None or s1 is None or s0[4] is None or s1[4] is None:
        print(f"  {title:<34} 位姿读取失败")
        return None

    dt = s1[4] - s0[4]
    dx, dy = s1[0] - s0[0], s1[1] - s0[1]
    dist = (dx * dx + dy * dy) ** 0.5
    dyaw = wrap(s1[3] - s0[3])
    yaw0 = s0[3]
    lon = dx * math.cos(yaw0) + dy * math.sin(yaw0)
    lat = -dx * math.sin(yaw0) + dy * math.cos(yaw0)

    r = dict(title=title, dt=dt, dist=dist, lon=lon, lat=lat,
             dyaw=dyaw * DEG, rate=(dyaw / dt * DEG) if dt > 0 else 0.0, motors=m)

    mm = "?" if not m else "[" + ",".join(f"{v:.0f}" for v in m) + "]"
    print(f"  {title:<34} Δt={dt:4.1f}s Δψ={r['dyaw']:+7.1f}° "
          f"({r['rate']:+6.1f}°/s) 位移={dist:5.3f}m 电机={mm}")
    return r


def ctrl_of(m):
    """电机输出 [0,1000] → 归一化 ctrl（500 = 停）"""
    return [round((v - 500.0) / 500.0, 3) for v in m] if m else None


def summary(vals):
    a = [abs(v) for v in vals]
    ok = sum(1 for x in a if x > 10.0)
    return statistics.median(a), min(a), max(a), ok, len(a)


# ─────────────────────────── 子命令 ───────────────────────────
def cmd_smoke(link, args):
    """冒烟：确认标定链路与三项近期改动都生效"""
    print("\n[1] 直线 thr=0.5, steer=0   （期望 ctrl=+0.400 → 电机 700）")
    r = measure(link, "直线 thr=0.5", 0.5, 0.0, 2.0)
    if r and r["motors"]:
        print(f"      ctrl={ctrl_of(r['motors'])}")
    stop(link, 2.0)

    print("\n[2] 满舵 thr=1.0             （TURN_SLOW=1.0 ⇒ v_cmd 归零，只剩 ±0.0825 差速）")
    r = measure(link, "满舵 thr=1.0", 1.0, 1.0, 2.0)
    if r and r["motors"]:
        print(f"      ctrl={ctrl_of(r['motors'])}   期望 ≈ [-0.083,+0.083,-0.083,+0.083]")
        print("      （若出现 [+0.32,+0.48] 说明 GD_TURN_SLOW 没生效）")
    stop(link, 2.0)

    print("\n[3] 半舵 thr=1.0, steer=0.5  （v_cmd=0.6 m/s，均值 ctrl 应 ≈ 0.400）")
    r = measure(link, "半舵 thr=1.0", 1.0, 0.5, 2.0)
    if r and r["motors"]:
        c = ctrl_of(r["motors"])
        print(f"      ctrl={c}  均值={sum(c) / len(c):+.3f}")

    print("\n[4] 已删除参数检查（发送后不应报错，且无任何效果）")
    for name in PARAM_BOOST_GONE:
        set_param(link, name, 0.5, settle=0.5)
    print(f"      已发送 {PARAM_BOOST_GONE} —— 参数表里已无这两个名字")

    stop(link, 1.5)
    print("\n完成。另可在 PX4 控制台执行 `ground_drive status` 看 calib 两行。")


def cmd_repeat(link, args):
    """重复性：同一指令跑 N 次，看双稳态分布"""
    thr, secs, n = args.throttle, args.secs, args.n
    print(f"\n{'=' * 74}")
    print(f"REPEAT  thr={thr:+.2f}  steer=+1.0  ×{n}   （每条 {secs:.0f} s）")
    print(f"{'=' * 74}")

    vals = []
    for i in range(n):
        r = measure(link, f"#{i + 1}", thr, +1.0, secs, settle=1.0)
        stop(link, 2.5)
        if r:
            vals.append(r["dyaw"])

    if not vals:
        return

    med, lo, hi, ok, tot = summary(vals)
    print(f"\n  |Δψ| 中位={med:5.1f}°  最小={lo:5.1f}°  最大={hi:5.1f}°  "
          f"标准差={statistics.pstdev([abs(v) for v in vals]):5.1f}°")
    print(f"  成功率(|Δψ|>10°) = {ok}/{tot} ({100 * ok / tot:.0f}%)")
    if lo < 1.0 and hi > 40.0:
        print("  ⇒ 双稳态（要么 0°、要么大角度）。这是 W/L<1 的静摩擦锁死特征，"
              "不是随机故障 —— 见文档 §14.2")


def cmd_scan(link, args):
    """纵向速度 → 转向能力 扫描"""
    values = [0.05, 0.10, 0.15, 0.20, 0.30, 0.40, 0.50, 0.65, 0.85, 1.00]
    print(f"\n{'=' * 74}\nSCAN  纵向速度 → 转向能力（steer=+1.0 固定）\n{'=' * 74}")
    print(f"  {'throttle':>9} {'轮速 m/s':>9} {'Δψ °':>8} {'偏航率 °/s':>11}")

    for v in values + [-x for x in values]:
        r = measure(link, f"thr={v:+.2f}", v, +1.0, args.secs, settle=1.2)
        stop(link, 2.5)
        if not r:
            continue
        m = r["motors"]
        wheel = float("nan")
        if m and len(m) == 4:
            ca, cb = (m[0] - 500) / 500, (m[1] - 500) / 500
            wheel = abs(0.5 * (ca + cb)) * (2400.0 / 64.0) * 0.04
        print(f"  {v:+9.2f} {wheel:9.3f} {r['dyaw']:+8.1f} {r['rate']:+11.1f}")


def cmd_gain(link, args):
    """GD_YAW_GAIN 扫描（纯 pivot）"""
    gains = [float(g) for g in args.gains.split(",")]
    print(f"\n{'=' * 74}\nGAIN  GD_YAW_GAIN 扫描（纯 pivot, thr=0, ×{args.n}）\n{'=' * 74}")

    for g in gains:
        set_param(link, PARAM_GAIN, g)
        vals = []
        print(f"\n  —— GAIN = {g:.1f} ——")
        for i in range(args.n):
            r = measure(link, f"gain={g:.1f} #{i + 1}", 0.0, +1.0, args.secs, settle=1.0)
            stop(link, 2.5)
            if r:
                vals.append(r["dyaw"])
        if vals:
            med, lo, hi, ok, tot = summary(vals)
            print(f"  ⇒ 中位={med:5.1f}° 最小={lo:5.1f}° 最大={hi:5.1f}° "
                  f"成功={ok}/{tot} ({100 * ok / tot:.0f}%)")
    set_param(link, PARAM_GAIN, 1.5)


def cmd_armangle(link, args):
    """摆臂折叠角扫描

    动机：摆臂绕 X 轴转，所以改工作角会同时改变两个东西 ——
      ① 轮距（角越小越宽）：52°→0.128 m，45°→0.146 m，38°→0.163 m
      ② 轮轴倾角（角越小越斜）：38° → 45° → 52°，圆柱轮从线接触退化成点接触
    两者对转向的作用方向相反，只能实测。

    ⚠️ 跑之前必须把 mux 的 <arm_threshold_deg> 降到 <= 最小测试角，
       否则低于 48° 时轮子根本不驱动（实测会得到一堆 [500,500,500,500]）。
    """
    degs = [float(x) for x in args.degs.split(",")]

    print(f"\n{'=' * 74}")
    print("ARMANGLE  摆臂折叠角 → 转向能力（纯 pivot, thr=0, roll=+1）")
    print("  理论轮距：52°→0.128 m (W/L 0.61) / 45°→0.146 (0.70) / 38°→0.163 (0.78)")
    print("  副作用：轮轴倾角 52°→38° / 45°→45° / 38°→52°（越斜接地越差）")
    print(f"{'=' * 74}")

    # 门控阈值必须 <= 最小测试角，否则低角度下 manualDrive 直接输出 0
    set_param(link, "GD_ARM_MIN_DEG", min(degs) - 5.0)

    for deg in degs:
        # 187 的 param1 → 角度：angle = 26 × (param1 + 1)
        link.servo = deg / 26.0 - 1.0
        link.hold(4.0)              # 等摆臂到位 + 车身落地稳定

        vals = []
        dists = []
        print(f"\n  ── 摆臂 {deg:.0f}° ──")

        for i in range(args.n):
            r = measure(link, f"{deg:.0f}° #{i + 1}", 0.0, +1.0, args.secs, settle=1.2)
            stop(link, 2.5)
            if r:
                vals.append(r["dyaw"])
                dists.append(r["dist"])

        if vals:
            med, lo, hi, ok, tot = summary(vals)
            print(f"  ⇒ 中位={med:5.1f}° 最小={lo:5.1f}° 最大={hi:5.1f}° "
                  f"成功={ok}/{tot} ({100 * ok / tot:.0f}%)  位移中位={statistics.median(dists):.3f}m")
            if statistics.median(dists) < 0.01:
                print("     （位移≈0：轮子可能没接地，这个角度不可用）")

    # 恢复
    link.servo = SERVO_GROUND
    set_param(link, "GD_ARM_MIN_DEG", 48.0)


def cmd_pivot(link, args):
    cases = [
        ("静止直接满舵", 0.0, 0.0),
        ("预跑 thr=0.5 → 满舵收油", 0.5, 1.5),
        ("预跑 thr=1.0 → 满舵收油", 1.0, 1.5),
    ]
    print(f"\n{'=' * 74}\nPIVOT  v_cmd 归零（GD_TURN_SLOW=1.0）×{args.n}\n{'=' * 74}")

    for title, pre_thr, pre_secs in cases:
        vals = []
        print(f"\n  —— {title} ——")
        for i in range(args.n):
            r = measure(link, f"{title} #{i + 1}", 0.0, +1.0, args.secs,
                        settle=1.0, pre_thr=pre_thr, pre_secs=pre_secs)
            stop(link, 2.5)
            if r:
                vals.append(r["dyaw"])
        if vals:
            med, lo, hi, ok, tot = summary(vals)
            print(f"  ⇒ 中位={med:5.1f}° 最小={lo:5.1f}° 最大={hi:5.1f}° "
                  f"成功={ok}/{tot} ({100 * ok / tot:.0f}%)")


def main():
    ap = argparse.ArgumentParser(description="地面驱动统一验证入口")
    sub = ap.add_subparsers(dest="cmd", required=True)

    sub.add_parser("smoke", help="冒烟测试（约 40 s）")

    p_rep = sub.add_parser("repeat", help="重复性（双稳态分布）")
    p_rep.add_argument("--throttle", type=float, default=0.0)
    p_rep.add_argument("--secs", type=float, default=3.0)
    p_rep.add_argument("-n", type=int, default=10)

    p_scan = sub.add_parser("scan", help="纵向速度 → 转向能力 扫描")
    p_scan.add_argument("--secs", type=float, default=3.0)

    p_gain = sub.add_parser("gain", help="GD_YAW_GAIN 扫描")
    p_gain.add_argument("--gains", default="1.5,3.0,5.0")
    p_gain.add_argument("--secs", type=float, default=3.0)
    p_gain.add_argument("-n", type=int, default=5)

    p_piv = sub.add_parser("pivot", help="v_cmd 归零 + 借动量")
    p_piv.add_argument("--secs", type=float, default=3.0)
    p_piv.add_argument("-n", type=int, default=6)

    p_arm = sub.add_parser("armangle", help="摆臂折叠角扫描")
    p_arm.add_argument("--degs", default="52,45,38")
    p_arm.add_argument("--secs", type=float, default=3.0)
    p_arm.add_argument("-n", type=int, default=6)

    sub.add_parser("steer", help="转发 verify_steer.py")
    sub.add_parser("reversible", help="转发 verify_reversible.py")

    args, rest = ap.parse_known_args()

    # 转发型子命令：不起 Link，直接调对方 main
    if args.cmd == "steer":
        import sys
        import verify_steer
        sys.argv = ["verify_steer.py"] + rest
        verify_steer.main()
        return

    if args.cmd == "reversible":
        import verify_reversible
        verify_reversible.main()
        return

    link = Link()
    print("=" * 74)
    print(f"verify.py · {args.cmd}")
    print("=" * 74)
    setup(link)

    try:
        {"smoke": cmd_smoke, "repeat": cmd_repeat, "scan": cmd_scan,
         "gain": cmd_gain, "pivot": cmd_pivot, "armangle": cmd_armangle}[args.cmd](link, args)
    finally:
        try:
            stop(link, 1.5)
        except Exception:
            pass


if __name__ == "__main__":
    main()
