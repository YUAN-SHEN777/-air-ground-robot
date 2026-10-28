#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
flight_report.py —— PX4 ULog 飞行数据速览（空-地两栖机器人专用）

用法:
    python3 flight_report.py                    # 自动扫 PX4 rootfs/log 最近 2 天
    python3 flight_report.py --days 1           # 只扫最近 1 天
    python3 flight_report.py --all              # 扫全部历史
    python3 flight_report.py a.ulg b.ulg        # 指定文件
    python3 flight_report.py -v                 # 打印事件明细

输出：每个日志一段摘要 + 末尾汇总表。
"""

import argparse
import glob
import os
import sys
import time

import numpy as np

try:
    from pyulog import ULog
except ImportError:
    sys.exit("需要 pyulog： pip install pyulog")

LOG_ROOT = "/home/yuan/PX4-Autopilot/build/px4_sitl_default/rootfs/log"

# ── 只看这些话题（大幅省内存/时间，188 MB 的 log 也能秒开）────────────
TOPICS = [
    "vehicle_status",
    "vehicle_local_position",
    "vehicle_attitude",
    "vehicle_land_detected",
    "actuator_motors",
    "actuator_servos",
    "vehicle_command",
    "cpuload",
    "log_message",
]

NAV_STATE = {
    0: "MANUAL", 1: "ALTCTL", 2: "POSCTL", 3: "MISSION", 4: "LOITER",
    5: "RTL", 6: "POS_SLOW", 7: "GUIDED_COURSE", 8: "ALT_CRUISE",
    9: "FREE3", 10: "ACRO", 11: "FREE2", 12: "DESCEND", 13: "TERMINATION",
    14: "OFFBOARD", 15: "STAB", 16: "FREE1", 17: "TAKEOFF", 18: "LAND",
    19: "FOLLOW", 20: "PRECLAND", 21: "ORBIT", 22: "VTOL_TAKEOFF",
    23: "EXTERNAL1", 24: "EXTERNAL2", 25: "EXTERNAL3", 26: "EXTERNAL4",
    27: "EXTERNAL5", 28: "EXTERNAL6", 29: "EXTERNAL7", 30: "EXTERNAL8",
}

# MAV_CMD 常用号
CMD = {11: "LAND", 22: "TAKEOFF", 176: "DO_SET_MODE", 187: "DO_SET_ACTUATOR",
       310: "ACTUATOR_TEST", 400: "ARM/DISARM", 511: "SET_MESSAGE_INTERVAL"}

LEVEL = {0: "EMERG", 1: "ALERT", 2: "CRIT", 3: "ERR", 4: "WARN",
         5: "NOTICE", 6: "INFO", 7: "DEBUG"}


# ═══════════════════════════════════════════════════════════════════
# 工具
# ═══════════════════════════════════════════════════════════════════

def ds(ulog, name):
    """安全取 dataset 的 .data 字典；没有就返回 None"""
    try:
        return ulog.get_dataset(name).data
    except Exception:
        return None


def array_field(d, base):
    """pyulog 把数组字段拆成 base[0], base[1] … 这里收回来 → (N, n)"""
    if d is None:
        return None
    if base in d:                       # 有些版本不拆
        a = np.asarray(d[base])
        return a if a.ndim == 2 else a[:, None]
    keys = [k for k in d if k.startswith(base + "[")]
    if not keys:
        return None
    keys.sort(key=lambda k: int(k[k.index("[") + 1:-1]))
    return np.column_stack([d[k] for k in keys])


def quat_to_euler(q):
    """q: (N,4) [w,x,y,z] → roll, pitch, yaw (rad)"""
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    roll = np.arctan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)


def segments(mask, ts):
    """把布尔 mask 切成 [(t_start, t_end), ...]"""
    out, start = [], None
    for i, m in enumerate(mask):
        if m and start is None:
            start = ts[i]
        elif not m and start is not None:
            out.append((start, ts[i]))
            start = None
    if start is not None:
        out.append((start, ts[-1]))
    return out


def rng(a):
    """(min, max, ptp) —— 全 NaN 时返回 None"""
    if a is None or len(a) == 0:
        return None
    a = a[np.isfinite(a)]
    if a.size == 0:
        return None
    return float(a.min()), float(a.max()), float(a.max() - a.min())


def fnum(v, unit="", nd=3):
    return "—" if v is None else f"{v:.{nd}f}{unit}"


# ═══════════════════════════════════════════════════════════════════
# 单文件分析
# ═══════════════════════════════════════════════════════════════════

def analyze(path):
    ulog = ULog(path, message_name_filter_list=TOPICS)

    # ── 统一时间基准 ──
    # ⚠️ vehicle_status 的首个 timestamp 可能早于 ulog.start_timestamp，
    #    用 uint64 相减会【下溢成 2^64】→ 必须转 int64 并自行取基准
    base = None
    for nm in ("vehicle_status", "vehicle_local_position", "vehicle_attitude",
               "actuator_motors", "actuator_servos", "vehicle_command"):
        d = ds(ulog, nm)
        if d is not None and len(d["timestamp"]):
            v = int(d["timestamp"][0])
            base = v if base is None else min(base, v)
    if base is None:
        base = int(ulog.start_timestamp)

    def rel(ts):
        return (np.asarray(ts).astype(np.int64) - base) / 1e6

    dur = (ulog.last_timestamp - base) / 1e6

    r = {
        "path": path,
        "name": os.path.basename(path),
        "size_mb": os.path.getsize(path) / 1048576,
        "dur": dur,
        "n_topics": len(ulog.data_list),
        "boot_s": base / 1e6,
    }

    # ── 固件 / 机型 ──
    mi = ulog.msg_info_dict
    r["ver"] = str(mi.get("ver_sw_release", "?"))
    r["branch"] = mi.get("ver_sw_branch", "?")
    r["sys_name"] = mi.get("sys_name", "?")

    # ── 机型识别（判断这个 log 是不是本机器人）──
    ip = ulog.initial_parameters
    r["autostart"] = ip.get("SYS_AUTOSTART")
    r["airframe"] = ip.get("CA_AIRFRAME")
    r["sv_func1"] = ip.get("SIM_GZ_SV_FUNC1")
    r["sv_maxa1"] = ip.get("SIM_GZ_SV_MAXA1")
    r["thr_hover"] = ip.get("MPC_THR_HOVER")
    r["sdlog_mode"] = ip.get("SDLOG_MODE")

    # ── 解锁 / 模式 ──
    st = ds(ulog, "vehicle_status")
    if st is not None:
        t = rel(st["timestamp"])
        arm = st["arming_state"]
        nav = st["nav_state"]
        r["armed_n"] = int(np.sum(arm[1:] == 2))
        segs = segments(arm == 2, t)
        r["armed_segs"] = segs
        r["armed_time"] = sum(b - a for a, b in segs)
        # 模式序列
        modes, last = [], None
        for i, n in enumerate(nav):
            if n != last:
                modes.append((t[i], int(n)))
                last = n
        r["modes"] = modes
        # 起降次数（nav_state 进入 17=TAKEOFF / 18=LAND）
        r["n_takeoff"] = int(np.sum((nav[1:] == 17) & (nav[:-1] != 17)))
        r["n_land"] = int(np.sum((nav[1:] == 18) & (nav[:-1] != 18)))
    else:
        r.update(armed_n=0, armed_segs=[], armed_time=0.0, modes=[],
                 n_takeoff=0, n_land=0)

    # ── 位置 / 高度 ──
    lp = ds(ulog, "vehicle_local_position")
    if lp is not None:
        t = rel(lp["timestamp"])
        z = -lp["z"]                                    # NED → 向上为正
        # ⚠️ x/y 是相对 EKF 原点，可能带很大的固定偏移（原点≠出生点）
        #    → 用【相对首个样本】的位移衡量真实行驶距离
        dx = lp["x"] - lp["x"][0]
        dy = lp["y"] - lp["y"][0]
        travel = np.hypot(dx, dy)
        spd = np.hypot(lp["vx"], lp["vy"])
        r["alt_max"] = float(np.nanmax(z))
        r["alt_final"] = float(z[-1])
        r["travel_max"] = float(np.nanmax(travel))
        r["origin_off"] = float(np.hypot(lp["x"][0], lp["y"][0]))
        r["spd_max"] = float(np.nanmax(spd))
        r["vz_max"] = float(np.nanmax(np.abs(lp["vz"])))
        r["climb_max"] = float(max(0.0, -np.nanmin(lp["vz"])))   # NED 里上升为负
        # 悬停时长：解锁 且 高度 > 0.3 m
        if st is not None:
            arm_i = np.interp(t, rel(st["timestamp"]),
                              st["arming_state"].astype(float))
            hover = (arm_i > 1.5) & (z > 0.3)
            r["hover_time"] = sum(b - a for a, b in segments(hover, t))
            r["hover_segs"] = segments(hover, t)
        else:
            r["hover_time"] = 0.0
    else:
        r.update(alt_max=None, travel_max=None, spd_max=None, hover_time=0.0)

    # ── 姿态 ──
    att = ds(ulog, "vehicle_attitude")
    if att is None:
        att = ds(ulog, "vehicle_attitude_groundtruth")
    if att is not None:
        q = np.column_stack([att[f"q[{i}]"] for i in range(4)])
        roll, pitch, yaw = quat_to_euler(q)
        r["roll"] = rng(roll)
        r["pitch"] = rng(pitch)
        r["yaw"] = rng(yaw)
        r["flip"] = bool(np.nanmax(np.abs(roll)) > 90 or
                         np.nanmax(np.abs(pitch)) > 90)

    # ── 电机 ──
    ctrl = array_field(ds(ulog, "actuator_motors"), "control")
    if ctrl is not None:
        r["n_am"] = ctrl.shape[1]
        r["mot_min"] = float(np.nanmin(ctrl))
        r["mot_max"] = float(np.nanmax(ctrl))
        r["mot_nan"] = int(np.sum(~np.isfinite(ctrl)))
        # 离散度：每个时刻 4 个电机的极差，取平均（越小越均衡）
        c4 = ctrl[:, :4]
        ok = np.all(np.isfinite(c4), axis=1)
        r["mot_spread"] = float(np.mean(c4[ok].max(1) - c4[ok].min(1))) if ok.any() else None

    # ── 舵机 ──
    svc = array_field(ds(ulog, "actuator_servos"), "control")
    if svc is not None:
        r["sv_rng"] = rng(svc[:, 0])
        if svc.shape[1] > 1:
            r["sv_rng1"] = rng(svc[:, 1])

    # ── 命令统计 ──
    vc = ds(ulog, "vehicle_command")
    cmds = {}
    if vc is not None:
        for k in np.unique(vc["command"]):
            cmds[int(k)] = int(np.sum(vc["command"] == k))
    r["cmds"] = cmds

    # ── CPU ──
    cp = ds(ulog, "cpuload")
    if cp is not None:
        r["cpu_max"] = float(np.nanmax(cp["load"]) * 100)

    # ── 事件 ──
    # ⚠️ 用 ULog 内嵌的 logging 段（ulog.logged_messages），
    #    不是 uORB 的 log_message 话题 —— 后者在 SDLOG_MODE=1 时常常不存在
    ev = []
    for m in getattr(ulog, "logged_messages", []):
        try:
            lv = int(m.log_level)
        except Exception:
            lv = 6
        s = str(m.message)
        t_s = (int(m.timestamp) - base) / 1e6
        if lv <= 4 or any(k in s for k in
                          ("Armed", "armed", "Disarmed", "Takeoff",
                           "Landing", "landed", "Failsafe", "failsafe",
                           "Disarm", "denied", "Denied", "Critical")):
            ev.append((t_s, str(LEVEL.get(lv, lv)), s))
    r["events"] = ev

    return r


# ═══════════════════════════════════════════════════════════════════
# 打印
# ═══════════════════════════════════════════════════════════════════

def verdict(r):
    """一句话判定这次会话干了什么"""
    if r.get("armed_n", 0) == 0:
        tag = "仅地面/舵机测试，全程未解锁"
    elif r.get("hover_time", 0) > 1.0:
        tag = f"含实际悬停 {r['hover_time']:.1f} s"
    elif r.get("alt_max") and r["alt_max"] > 0.3:
        tag = f"离地 {r['alt_max']:.2f} m（未稳定悬停）"
    else:
        tag = "解锁但未离地"
    if r.get("flip"):
        tag = "⚠️ 出现 >90° 翻转（炸机）→ " + tag
    return tag


def show(r, idx, total, verbose=False):
    W = 78
    print("═" * W)
    print(f"[{idx}/{total}] {r['name']}   {r['size_mb']:.2f} MB   {r['path']}")
    print("─" * W)

    print(f"  固件      PX4 {r['ver']} @ {r['branch']}   boot+{r['boot_s']:.0f}s")
    print(f"  时长      {r['dur']:.1f} s   话题 {r['n_topics']} 个")
    print(f"  机型      SYS_AUTOSTART={r.get('autostart')}"
          f"  CA_AIRFRAME={r.get('airframe')}"
          f"  SV_FUNC1={r.get('sv_func1')}"
          f"  THR_HOVER={r.get('thr_hover')}"
          f"  SDLOG={r.get('sdlog_mode')}")
    if r.get("autostart") not in (4030, None):
        print(f"  ⚠️        非本项目机架（SYS_AUTOSTART={r['autostart']}）")

    # 解锁
    if r.get("armed_n", 0):
        segs = ", ".join(f"{a:.1f}~{b:.1f}s" for a, b in r["armed_segs"][:6])
        more = f" …共{len(r['armed_segs'])}段" if len(r["armed_segs"]) > 6 else ""
        print(f"  解锁      {len(r['armed_segs'])} 段 / 合计 {r['armed_time']:.1f} s"
              f"   [{segs}{more}]")
    else:
        print("  解锁      ❌ 全程未解锁")

    # 模式
    if r.get("modes"):
        s = " → ".join(f"{NAV_STATE.get(n, n)}({n})" for _, n in r["modes"][:8])
        more = f" …共{len(r['modes'])}次切换" if len(r["modes"]) > 8 else ""
        print(f"  模式      {s}{more}")

    # 飞行
    print(f"  高度      最大 {fnum(r.get('alt_max'), ' m')}"
          f"   末态 {fnum(r.get('alt_final'), ' m')}"
          f"   爬升率 max {fnum(r.get('climb_max'), ' m/s', 2)}")
    print(f"  位移      相对起点 max {fnum(r.get('travel_max'), ' m')}"
          f"   （EKF 原点偏移 {fnum(r.get('origin_off'), ' m', 1)}）")
    print(f"  速度      水平 max {fnum(r.get('spd_max'), ' m/s')}"
          f"   垂直 max {fnum(r.get('vz_max'), ' m/s')}"
          f"   悬停 {r.get('hover_time', 0):.1f} s")
    print(f"  起降      起飞 {r.get('n_takeoff', 0)} 次 / 降落 {r.get('n_land', 0)} 次")

    # 姿态
    if r.get("roll"):
        def sp(k):
            v = r.get(k)
            return f"{v[2]:.2f}°" if v else "—"
        print(f"  姿态幅度  roll {sp('roll')}   pitch {sp('pitch')}   yaw {sp('yaw')}"
              + ("   ⚠️ >90°" if r.get("flip") else ""))

    # 电机
    if "mot_min" in r:
        print(f"  电机      control [{r['mot_min']:.3f}, {r['mot_max']:.3f}]"
              f"   4轴离散度均值 {fnum(r.get('mot_spread'), '', 4)}"
              f"   NaN {r['mot_nan']}")

    # 舵机
    if r.get("sv_rng"):
        a, b, _ = r["sv_rng"]
        print(f"  舵机      aux-servo0 control [{a:.3f}, {b:.3f}]"
              f"  →  角度 [{26*(a+1):.1f}°, {26*(b+1):.1f}°]")
    if r.get("sv_rng1"):
        a, b, _ = r["sv_rng1"]
        print(f"            aux-servo1 control [{a:.3f}, {b:.3f}]"
              f"  →  角度 [{26*(a+1):.1f}°, {26*(b+1):.1f}°]")

    # 命令
    if r.get("cmds"):
        items = [f"{CMD.get(k, 'CMD' + str(k))}({k})×{v}"
                 for k, v in sorted(r["cmds"].items(), key=lambda x: -x[1])[:8]]
        print("  命令      " + "  ".join(items))

    if "cpu_max" in r:
        print(f"  CPU       max {r['cpu_max']:.1f}%")

    print(f"  ▸ 判定    {verdict(r)}")

    if r.get("events"):
        n = len(r["events"]) if verbose else min(10, len(r["events"]))
        print(f"  事件      （共 {len(r['events'])} 条，显示前 {n}）")
        for t_s, lv, m in r["events"][:n]:
            print(f"      {t_s:7.2f}s [{lv:6s}] {m[:100]}")
    print()


def table(rs):
    print("═" * 132)
    print("汇总表（按会话顺序）")
    print("─" * 132)
    h = (f"{'#':>2}  {'日志':<14} {'MB':>6} {'时长s':>7} {'解锁s':>7} "
         f"{'高度max':>8} {'悬停s':>7} {'位移m':>7} {'roll°':>7} {'pitch°':>7} "
         f"{'CPU%':>6}  判定")
    print(h)
    print("─" * 132)
    for i, r in enumerate(rs, 1):
        def g(k):
            v = r.get(k)
            return f"{v:.2f}" if isinstance(v, float) else "—"
        print(f"{i:>2}  {r['name']:<14} {r['size_mb']:>6.1f} {r['dur']:>7.1f} "
              f"{r.get('armed_time', 0):>7.1f} {g('alt_max'):>8} "
              f"{r.get('hover_time', 0):>7.1f} {g('travel_max'):>7} "
              f"{(r['roll'][2] if r.get('roll') else float('nan')):>7.2f} "
              f"{(r['pitch'][2] if r.get('pitch') else float('nan')):>7.2f} "
              f"{r.get('cpu_max', float('nan')):>6.1f}  {verdict(r)}")
    print("═" * 132)


# ═══════════════════════════════════════════════════════════════════

def find_logs(days):
    if not os.path.isdir(LOG_ROOT):
        sys.exit(f"日志根目录不存在: {LOG_ROOT}")
    files = glob.glob(os.path.join(LOG_ROOT, "*", "*.ulg"))
    if days == 0:
        return sorted(files)
    cutoff = time.time() - days * 86400
    return sorted([f for f in files if os.path.getmtime(f) >= cutoff])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="*", help="指定 ulg 文件")
    ap.add_argument("--days", type=int, default=2, help="扫最近 N 天（默认 2）")
    ap.add_argument("--all", action="store_true", help="扫全部历史")
    ap.add_argument("-v", "--verbose", action="store_true", help="事件明细")
    a = ap.parse_args()

    files = a.files if a.files else find_logs(0 if a.all else a.days)
    if not files:
        sys.exit("没找到日志")

    files = sorted(files, key=os.path.getmtime)
    print(f"\n共 {len(files)} 个日志，按时间升序\n")

    rs = []
    for i, f in enumerate(files, 1):
        try:
            t0 = time.time()
            r = analyze(f)
            r["_t"] = time.time() - t0
            show(r, i, len(files), a.verbose)
            rs.append(r)
        except Exception as e:
            print("═" * 78)
            print(f"[{i}/{len(files)}] {os.path.basename(f)}  ❌ 分析失败: {e}\n")

    if rs:
        table(rs)


if __name__ == "__main__":
    main()
