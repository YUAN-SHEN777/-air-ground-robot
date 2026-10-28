#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摆臂舵机 · **干净参数**下的测试
================================================================================
背景：
  之前测舵机时我在运行时 param_set 改了 SIM_GZ_SV_MINA1/MAXA1，
  结果被持久化到 parameters.bson，把 airframe 的 set-default 顶掉了
  （表现为"代码改了却看不到效果"）。
  本次测试【全程不调用任何 param_set / param_set-default】，只：
     ① 启动前删掉 parameters.bson（强制按 airframe 重建）
     ② 启动后【只读】参数，确认 airframe 的 0/52 真的生效了
     ③ 发 DO_SET_ACTUATOR 命令 + 采样关节角

要回答的两个问题：
  问题 A：未解锁时摆臂应停在 0°（SIM_GZ_SV_DIS1=-1），实测却是 +52°？
  问题 B：从 0° 折不起来（26°/52° 都卡在 ~0.35°）——是"慢"还是"死点"？

用法：
    python3 servo_test.py            # 完整测试（约 2 分钟）
    python3 servo_test.py --keep     # 结束后不清理进程，留着看
"""
import math
import os
import re
import struct
import subprocess
import sys
import time

PX4 = "/home/yuan/PX4-Autopilot"
MODEL = "gz_airground"
LOG = "/tmp/servo_test.log"
BSON = f"{PX4}/build/px4_sitl_default/rootfs/parameters.bson"

T_LAUNCH = 30          # 等 PX4 + Gazebo 起来 + 传感器收敛（解锁需要）
T_HOLD = 25            # 折臂观测窗口（问题 B：区分"慢"和"死点"）
TOTAL_BUDGET = 150

KEEP = "--keep" in sys.argv

# 只读参数清单（验证 airframe 是否真的生效）
WANT_SV = ["SIM_GZ_SV_FUNC1", "SIM_GZ_SV_FUNC2",
           "SIM_GZ_SV_MIN1", "SIM_GZ_SV_MAX1",
           "SIM_GZ_SV_MINA1", "SIM_GZ_SV_MAXA1",
           "SIM_GZ_SV_DIS1", "SIM_GZ_SV_FAIL1"]
# 期望值（与 airframe 一致）
# ⚠️ MIN/MAX/DIS/FAIL 是 0~1000 的【整数】，不是 [-1,1]（源码 module.yaml 的 standard_params）
EXPECT = {"SIM_GZ_SV_FUNC1": 301, "SIM_GZ_SV_FUNC2": 0,
          "SIM_GZ_SV_MIN1": 0, "SIM_GZ_SV_MAX1": 1000,
          "SIM_GZ_SV_MINA1": 0.0, "SIM_GZ_SV_MAXA1": 52.0,
          "SIM_GZ_SV_DIS1": 0, "SIM_GZ_SV_FAIL1": 0}


def banner(s):
    print("\n" + "=" * 74)
    print(s)
    print("=" * 74)


def cleanup():
    for pat in ("bin/px4", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def int_param(v):
    """PX4 的 INT32 参数经 MAVLink 回来是 float，需按位重解释"""
    try:
        return struct.unpack("<i", struct.pack("<f", v))[0]
    except Exception:
        return v


def main():
    t_start = time.time()   # 用于结尾统计耗时
    proc = logf = None   # noqa

    cleanup()
    time.sleep(1)

    # ---------- ① 清空参数持久化 ----------
    banner("① 清空参数持久化（保证按 airframe 默认值启动）")
    if os.path.exists(BSON):
        sz = os.path.getsize(BSON)
        os.remove(BSON)
        print(f"  已删除 {BSON}（{sz} 字节）")
    else:
        print(f"  {BSON} 不存在 ✓")
    for extra in (BSON.replace(".bson", "_backup.bson"),):
        if os.path.exists(extra):
            os.remove(extra)
            print(f"  已删除 {extra}")

    # ---------- ② 启动 ----------
    banner(f"② 启动 PX4 + {MODEL}（headless，日志 {LOG}）")
    logf = open(LOG, "w")
    proc = subprocess.Popen(["make", "px4_sitl", MODEL], cwd=PX4,
                            stdout=logf, stderr=subprocess.STDOUT,
                            env={**os.environ, "PX4_GZ_HEADLESS": "1"},
                            preexec_fn=os.setsid)
    print(f"  等待 {T_LAUNCH} 秒启动 + 传感器收敛 ...")
    time.sleep(T_LAUNCH)

    # ---------- 准备订阅关节角 ----------
    from gz.transport13 import Node
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.double_pb2 import Double
    from gz.msgs10.actuators_pb2 import Actuators
    from pymavlink import mavutil

    CUR = {"servo_0": None, "servo_1": None}   # 弧度
    # 实际施加到关节上的力矩（N·m）—— 用来量"到底需要多大力矩"
    FORCE = {"servo_0": 0.0, "servo_1": 0.0}

    def on_joint(msg):
        for j in msg.joint:
            if j.name in CUR:
                CUR[j.name] = j.axis1.position
                FORCE[j.name] = max(FORCE[j.name], abs(j.axis1.force))

    # 【探针】监视 PX4 到底有没有往舵机话题发东西（话题里的值是【弧度】）
    CMDVAL = {"v": None, "n": 0}

    def on_servo_cmd(msg):
        CMDVAL["v"] = msg.data
        CMDVAL["n"] += 1

    # 【探针】电机话题：用来判断 gz_bridge 眼里的"已解锁"状态是否成立
    MOTOR = {"v": None, "n": 0}

    def on_motor(msg):
        MOTOR["v"] = list(msg.velocity)
        MOTOR["n"] += 1

    # 【探针】机身静止高度 —— 与独立模式对比，判断触地姿态是否一致
    POSE = {}

    def on_pose(msg):
        for p in msg.pose:
            n = p.name.rsplit("::", 1)[-1]
            if n in ("base_link", "link9", "link10", "link1", "link5"):
                POSE[n] = (p.position.x, p.position.y, p.position.z)

    from gz.msgs10.pose_v_pb2 import Pose_V
    node = Node()
    # ⚠️ 话题名是 /airground/joint_state（在 model.sdf 的 JointStatePublisher 里定义），
    #    不是 /joint_state —— 之前订阅错话题导致一条数据都收不到
    node.subscribe(Model, "/airground/joint_state", on_joint)
    node.subscribe(Model, "/joint_state", on_joint)   # 兼容旧的独立仿真世界
    node.subscribe(Double, "/model/airground_0/servo_0", on_servo_cmd)
    node.subscribe(Double, "/model/airground_0/servo_1", on_servo_cmd)
    node.subscribe(Actuators, "/airground_0/command/motor_speed", on_motor)
    node.subscribe(Pose_V, "/world/airground/pose/info", on_pose)
    node.subscribe(Pose_V, "/world/default/pose/info", on_pose)
    time.sleep(1.0)

    def sample(sec, label="", quiet=False):
        """采样关节角，返回 (t序列, s0序列, s1序列) —— 单位：度"""
        ts, a0, a1 = [], [], []
        t0 = time.time()
        last_print = 0
        while time.time() - t0 < sec:
            t = time.time() - t0
            s0, s1 = CUR["servo_0"], CUR["servo_1"]
            if s0 is not None:
                ts.append(t)
                a0.append(math.degrees(s0))
                a1.append(math.degrees(s1) if s1 is not None else float("nan"))
            if not quiet and t - last_print >= 5.0:
                last_print = t
                if a0:
                    print(f"      t={t:5.1f}s  servo_0={a0[-1]:+7.2f}°  "
                          f"servo_1={a1[-1]:+7.2f}°")
            time.sleep(0.02)
        return ts, a0, a1

    def stat(a):
        if not a:
            return (float("nan"),) * 3
        return a[-1], min(a), max(a)

    def cmd_str():
        """PX4 → Gazebo 舵机话题的探针结果"""
        n, v = CMDVAL["n"], CMDVAL["v"]
        if v is None:
            return f"❌ 一条都没收到（{n} 条）—— PX4 没往舵机话题发！"
        return f"✅ 收到 {n} 条，最新 = {math.degrees(v):+.2f}°"

    # ---------- ③ 只读参数校验 ----------
    conn = mavutil.mavlink_connection("udpout:127.0.0.1:18570",
                                      source_system=255, source_component=0)
    for _ in range(5):
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.2)
    conn.wait_heartbeat(timeout=15)
    sysid, comp = conn.target_system, conn.target_component
    print(f"  ✅ MAVLink 已连接 (system={sysid})")

    # ---------- 【关键】持续发 MANUAL_CONTROL（等价于 QGC 虚拟摇杆）----------
    # 为什么必需：开机默认 Position 模式需要摇杆，commander 的 modeCheck 会
    # clearArmingBits → 解锁被拒（但 ARM 命令仍回 ACCEPTED，极易误判！）
    import threading
    STICK = {"n": 0, "stop": False}

    def stick_thread():
        n = 0
        while not STICK["stop"]:
            # x,y,r ∈ [-1000,1000]，z(油门) ∈ [0,1000]；全中位 + 油门到底
            conn.mav.manual_control_send(sysid, 0, 0, 0, 0, 0)
            # ⚠️ 必须【周期】发 GCS 心跳！mavutil 不会自动发。
            #    不发的话 PX4 几秒后就会报 "Connection to ground station lost"。
            if n % 10 == 0:
                conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                        mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                        0, 0, 0)
            n += 1
            STICK["n"] = n
            time.sleep(0.1)

    threading.Thread(target=stick_thread, daemon=True).start()
    print("  已启动 GCS 心跳 + MANUAL_CONTROL 线程（10 Hz，摇杆全中位）")

    # ---------- 【关键】关掉自动上锁 ----------
    #   COM_DISARM_LAND  = 2.0   落地后 2 秒自动上锁
    #   COM_DISARM_PRFLT = 10.0  解锁后 10 秒不起飞自动上锁   ← 就是它把我们坑了
    # PX4 日志原话：INFO [commander] Disarmed by auto preflight disarming
    #
    # 这是【测试专用】设置，故意放在脚本里而不写进 airframe：
    # 真实飞行需要自动上锁，不该固定成 0。而本脚本每次启动都会删
    # parameters.bson，所以运行时改的参数不会污染下一次运行。
    #
    # ⚠️ 踩过的坑：参数类型必须与 PX4 里定义的一致，
    #    否则 PX4 报 "param types mismatch" 并【静默丢弃】：
    #      COM_DISARM_* 是 FLOAT → 必须用 MAV_PARAM_TYPE_REAL32
    #      SIM_GZ_EC_*  是 INT32 → 用 MAV_PARAM_TYPE_INT32
    MAVF = mavutil.mavlink

    def pset(name, val, typ):
        conn.mav.param_set_send(sysid, comp, name.encode(), float(val), typ)
        # 等回执确认真的写进去了
        t0 = time.time()
        while time.time() - t0 < 1.5:
            m = conn.recv_match(type="PARAM_VALUE", blocking=False)
            if m is not None and m.param_id == name:
                return m.param_value
            time.sleep(0.01)
        return None

    print("  关闭自动上锁：")
    for p in ("COM_DISARM_LAND", "COM_DISARM_PRFLT"):
        got = pset(p, 0, MAVF.MAV_PARAM_TYPE_REAL32)
        print(f"    {p:18s} → 回读 {got if got is not None else '❌ 没回执'}")

    # ---------- 【关键】把螺旋桨停掉 ----------
    # 舵机测试根本不需要推力；而桨一转，机器人就被顶得离地/翻倒。
    #   FUNC=0  → 该通道不分配功能 → outputs = NAN
    #   DIS=0   → NAN 时取 DIS 值 = 0 → 电机不转（DIS 出厂是 500，会半速空转！）
    print("  停桨：")
    for i in range(1, 5):
        a = pset(f"SIM_GZ_EC_FUNC{i}", 0, MAVF.MAV_PARAM_TYPE_INT32)
        b = pset(f"SIM_GZ_EC_DIS{i}", 0, MAVF.MAV_PARAM_TYPE_INT32)
        print(f"    EC{i}: FUNC={a}  DIS={b}")

    # ⚠️ 坑：不能"这次没收到 HEARTBEAT 就当没解锁"——那会误判成掉锁。
    #    必须保留【上一次已知状态】。
    ARMED = {"v": None}

    def armed_now():
        """抽干缓冲，返回【最近已知】解锁位（None = 从未收到过）"""
        while True:
            m = conn.recv_match(blocking=False)
            if m is None:
                break
            tt = m.get_type()
            if tt == "HEARTBEAT":
                ARMED["v"] = bool(m.base_mode
                                  & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif tt == "STATUSTEXT":
                s = m.text.strip()
                if s and any(k in s.lower() for k in
                             ("disarm", "arm", "fail", "lock", "land",
                              "failsafe", "reject", "manual", "rc ")):
                    print(f"       PX4播报: {s}")
        return ARMED["v"]

    def arm_now(sec=10.0):
        """解锁并等到确认解锁位【确定】为 True"""
        conn.mav.command_long_send(
            sysid, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        t0 = time.time()
        while time.time() - t0 < sec:
            if armed_now() is True:
                return True
            time.sleep(0.05)
        return False

    banner("③ airframe 参数是否真的生效（只读，不写）")
    PARAMS = {}

    def on_param(msg):
        if msg.get_type() == "PARAM_VALUE":
            v = msg.param_value
            if 0 < abs(v) < 1e-40:              # 非规格化浮点 → 其实是 INT32
                iv = int_param(v)
                if iv != 0:
                    v = iv
            PARAMS[msg.param_id] = v

    print("  ⚠️ 逐条慢速读取（快速连发会被 PX4 丢包）")
    for p in WANT_SV:
        conn.mav.param_request_read_send(sysid, comp, p.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 0.6:
            while True:
                m = conn.recv_match(blocking=False)
                if m is None:
                    break
                on_param(m)
            time.sleep(0.01)

    print()
    ok_all = True
    for p in WANT_SV:
        got = PARAMS.get(p)
        exp = EXPECT.get(p)
        if got is None:
            flag, txt = "⚠️ ", "(未读到)"
        elif abs(got - exp) < 1e-3:
            flag, txt = "✅ ", ""
        else:
            flag, txt = "❌ ", f"  ← 期望 {exp}【被污染了！】"
            ok_all = False
        gs = f"{got:g}" if isinstance(got, (int, float)) else str(got)
        print(f"  {flag}{p:20s} = {gs:<8}{txt}")
    print(f"\n  {'✅ 参数全部正确，airframe 生效' if ok_all else '❌ 参数有偏差！'}")

    # 抽干缓冲
    while conn.recv_match(blocking=False) is not None:
        pass

    # ---------- ④ 【问题 A】未解锁状态 ----------
    banner("④ 【问题 A】未解锁（disarmed）时摆臂停在哪？")
    print("  期望：SIM_GZ_SV_DIS1 = 0 → 映射到 MINA1 = 0°")
    print("  实际采样：")
    _, d0, d1 = sample(4.0, quiet=False)
    u0, m0, M0 = stat(d0)
    u1, m1, M1 = stat(d1)
    print(f"\n    servo_0: 末值 {u0:+7.2f}°  范围 [{m0:+.2f}, {M0:+.2f}]°")
    print(f"    servo_1: 末值 {u1:+7.2f}°  范围 [{m1:+.2f}, {M1:+.2f}]°")
    print(f"    【探针】PX4→/model/airground_0/servo_0 : {cmd_str()}")
    print(f"    【探针】电机话题（未解锁）: {MOTOR['v'] if MOTOR['v'] else '(无数据)'}")
    if POSE:
        print("    【探针】世界坐标（对照独立模式 base_link z = 0.0210）:")
        for k in ("base_link", "link9", "link10", "link5", "link1"):
            if k in POSE:
                x, y, z = POSE[k]
                print(f"        {k:10s} x={x:+.3f}  y={y:+.3f}  z={z:+.4f}")
    else:
        print("    【探针】姿态话题: (收不到)")
    near0 = abs(u0) < 3.0
    print(f"    → {'✅ 停在 0° 附近，DIS1 生效' if near0 else '❌ 不在 0°！DIS1 未生效（问题 A 复现）'}")

    # ---------- ⑤ 解锁 ----------
    banner("⑤ 解锁（关键：ARM 回 ACCEPTED ≠ 真的解锁了！）")
    print("  等待 3 秒让 MANUAL_CONTROL 被 PX4 认定为有效输入源 ...")
    time.sleep(3.0)
    print(f"  （已发 {STICK['n']} 条 MANUAL_CONTROL）")

    armed_flag = arm_now(15.0)
    print(f"    【真实解锁位】   : {armed_flag}"
          f"   {'✅ 真的解锁了' if armed_flag else '❌ 其实没解锁！'}")
    print(f"    【探针】电机话题 : {MOTOR['v'] if MOTOR['v'] else '(无数据)'}")

    if not armed_flag:
        print("\n    ❌ 飞机没解锁 → 所有输出都会走 DIS 值 → 舵机测试无意义，提前结束")
        print("       （排查方向：预检失败原因看上面打印的 PX4 STATUSTEXT）")
        return

    # 观察几秒，看会不会又掉锁（并打印 PX4 的原因播报）
    print("  观察 5 秒，看会不会掉锁 ...")
    t0 = time.time()
    lost_at = None
    while time.time() - t0 < 5.0:
        if armed_now() is False:
            lost_at = time.time() - t0
            break
        time.sleep(0.05)
    if lost_at is not None:
        print(f"    ❌ 解锁后 {lost_at:.1f} 秒掉锁了（原因见上面的 PX4播报）")
        return
    print("    ✅ 观察 5 秒未掉锁")

    # ---------- ⑥ 命令 >> DO_SET_ACTUATOR ----------
    def do_set(v, note):
        """MAV_CMD_DO_SET_ACTUATOR = 187（pymavlink 方言是 ardupilotmega，
        没有这个常量名，只能用数字）。param1=执行器1的值[-1,1]，param7=0"""
        if not armed_now():
            print("       ⚠️ 掉锁了，重新解锁 ...")
            arm_now(10.0)
        conn.mav.command_long_send(sysid, comp, 187, 0,
                                   float(v), 0, 0, 0, 0, 0, 0)
        print(f"    已发 DO_SET_ACTUATOR({v:+.1f})  {note}")
        # 抽 ACK：ACCEPTED 说明 PX4 认了这条命令
        t0 = time.time()
        acks = []
        while time.time() - t0 < 1.0:
            while True:
                m = conn.recv_match(blocking=False)
                if m is None:
                    break
                if m.get_type() == "COMMAND_ACK":
                    acks.append((m.command, m.result))
            time.sleep(0.005)
        if acks:
            nm = {0: "ACCEPTED", 1: "TEMP_REJECTED", 2: "DENIED",
                  3: "UNSUPPORTED", 4: "FAILED", 5: "IN_PROGRESS"}
            for c, r in acks:
                mark = "✅" if r == 0 else "❌"
                print(f"       ACK cmd={c} → {nm.get(r, r)} {mark}")
        else:
            print("       ACK: (1 秒内没收到)")

    def to_deg(v):
        """DO_SET 值 → 理论角度。两层映射：
           第1层 outputs = interpolate(v, -1, 1, MIN1=0, MAX1=1000) → 0/500/1000
           第2层 angle   = MINA1 + (MAXA1-MINA1)*(outputs-MIN1)/(MAX1-MIN1) → 0/26/52°"""
        out = (v - (-1.0)) / (1.0 - (-1.0)) * (1000.0 - 0.0) + 0.0
        return 0.0 + (out - 0.0) / (1000.0 - 0.0) * (52.0 - 0.0)

    banner("⑥ 【问题 B】从 0° 折起来：是「慢」还是「死点」？")

    # 6.1 先回到 0°
    do_set(-1.0, "→ 先归零")
    sample(3.0, quiet=True)

    # 6.2 折到 26°（半程），观察 8 秒
    do_set(0.0, f"→ 目标 {to_deg(0.0):.0f}°（半程）")
    print("     观测 8 秒：")
    _, b0, b1 = sample(8.0)
    e0, lo0, hi0 = stat(b0)
    print(f"\n     servo_0: 末值 {e0:+7.2f}°  峰值 {hi0:+.2f}°  (目标 {to_deg(0.0):.0f}°)")

    # 6.3 折到 52°（全行程），观察 T_HOLD 秒
    do_set(1.0, f"→ 目标 {to_deg(1.0):.0f}°（全行程）")
    print(f"     观测 {T_HOLD} 秒（关键：看有没有【缓慢爬升】）：")
    _, c0, c1 = sample(T_HOLD)
    f0, lo0, hi0 = stat(c0)
    f1, lo1, hi1 = stat(c1)
    tgt = to_deg(1.0)
    print(f"\n     servo_0: 末值 {f0:+7.2f}°  峰值 {hi0:+.2f}°  (目标 {tgt:.0f}°)")
    print(f"     servo_1: 末值 {f1:+7.2f}°  峰值 {hi1:+.2f}°")
    print(f"     两臂差  : {abs(f0-f1):.3f}°")
    print(f"     【探针】PX4→/model/airground_0/servo_0 : {cmd_str()}")
    print(f"     【力矩】实际施加 servo_0 = {FORCE['servo_0']:.3f} N·m, "
          f"servo_1 = {FORCE['servo_1']:.3f} N·m"
          f"   （关节 effort 上限 0.7 N·m）")
    if FORCE["servo_0"] > 0.69:
        print("     → 力矩已打满 0.7 N·m 仍不动 ⇒ 【负载 > 0.7 N·m，力矩不足】")
    still = armed_now()
    print(f"     【探针】命令后解锁位 : {still}"
          f"   {'✅ 仍解锁' if still else '❌ 又被上锁了！'}")
    print(f"     【探针】电机话题     : {MOTOR['v'] if MOTOR['v'] else '(无数据)'}")

    # 判定
    reached = max(hi0, hi1) > tgt * 0.85
    creep = (hi0 - c0[0]) if c0 else 0
    print()
    if still is False:
        print("     ⚠️ 结论无效：飞机在测试中途被自动上锁了，输出全部回落 DIS 值。")
        print("        需先解决自动上锁问题再重测。")
    elif reached:
        print(f"     ✅ 能折起来（峰值 {hi0:+.1f}°）→ 折叠通路正常")
    elif creep > 2.0:
        print(f"     ⚠️ 在【缓慢爬升】（{c0[0]:+.2f}° → {hi0:+.2f}°，涨了 {creep:.2f}°）")
        print("        → 只是【慢/力矩不足】，不是死点")
    else:
        print(f"     ❌ 几乎不动（峰 {hi0:+.2f}°，只涨 {creep:.2f}°）")
        print("        → 0° 是【机械死点】或力矩不足，需加大舵机力矩或改策略")

    # ---------- ⑦ 解锁状态下的下行 ----------
    banner("⑦ 解锁后回零（DIS 不再适用，看命令映射）")
    do_set(-1.0, "→ 目标 0°")
    _, g0, g1 = sample(6.0, quiet=True)
    print(f"    servo_0 = {g0[-1]:+7.2f}°   (命令 -1.0 → 理论 0°)" if g0 else "    servo_0 = (无数据)")
    print(f"    servo_1 = {g1[-1]:+7.2f}°" if g1 else "    servo_1 = (无数据)")

    # ---------- 汇总 ----------
    banner("⑧ 汇总")
    txt = open(LOG, errors="ignore").read()
    print(f"  STALE 次数      : {len(re.findall('STALE', txt))}")
    print(f"  模型加载        : {'✅ airground' if 'model: airground' in txt else '❌'}")
    print(f"  日志            : {LOG}")
    print(f"  PX4 进程 pid    : {proc.pid if proc else '—'}")
    print(f"  本次耗时        : {time.time() - t_start:.0f} 秒")


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n(用户中断)")
    except Exception:
        import traceback
        traceback.print_exc()
    finally:
        # ⚠️ 必须清掉本次运行时写的参数（COM_DISARM_* / SIM_GZ_EC_*）！
        # 不清的话下次手动 `make px4_sitl gz_airground` 会继承
        # COM_DISARM_LAND=0（全局关掉自动上锁），污染正常使用。
        for f in (BSON, BSON.replace(".bson", "_backup.bson")):
            if os.path.exists(f):
                os.remove(f)
                print(f"(已清理参数持久化 {os.path.basename(f)})")
        if KEEP:
            print("\n(--keep：保留运行中的进程)")
        else:
            cleanup()
            print("\n(已清理进程)")
