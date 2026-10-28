#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
PX4 + airground 一键诊断
========================
把之前需要反复手工跑的检查全部合并成【一次运行】，
全程最长约 70 秒，到点自动清理所有进程，不会挂住终端。

用法：
    python3 px4_check.py

检查项与判据：
  ① 模型是否被 PX4 加载          → 日志里有 "model: airground_0"
  ② IMU 是否持续发布            → 数量应 ≈ 250 × 秒数（设计值 250 Hz）
  ③ airframe 参数是否生效        → CA_ROTOR_COUNT 应为 4（注意 INT/FLOAT 混读陷阱）
  ④ 解锁是否通过                → ACCEPTED / TEMP_REJECTED
  ⑤ 电机指令是否真下发          → 解锁后应不再是 [0,0,0,0]
"""
import os
import re
import signal
import struct
import subprocess
import time

PX4 = "/home/yuan/PX4-Autopilot"
MODEL = "gz_airground"
LOG = "/tmp/px4_check.log"
T_LAUNCH = 24          # 等 PX4 + Gazebo 启动
T_SETTLE = 15          # 启动后再静置（本项是为验证 STALE 是否自行恢复）
T_IMU = 3              # IMU 计数窗口
TOTAL_BUDGET = 75      # 全程硬上限


def banner(s):
    print("\n" + "=" * 68)
    print(s)
    print("=" * 68)


def int_param(v):
    """PX4 的 INT32 参数经 MAVLink 传回来是 float，需要按位重解释。
    例：CA_ROTOR_COUNT=4 会读成 5.60519e-45"""
    try:
        return struct.unpack("<i", struct.pack("<f", v))[0]
    except Exception:
        return v


def cleanup():
    for pat in ("bin/px4", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    t_start = time.time()

    def left():
        return TOTAL_BUDGET - (time.time() - t_start)

    cleanup()
    time.sleep(1)

    banner(f"启动 PX4 + {MODEL}（后台，日志 {LOG}）")
    logf = open(LOG, "w")
    proc = subprocess.Popen(["make", "px4_sitl", MODEL], cwd=PX4,
                            stdout=logf, stderr=subprocess.STDOUT,
                            env={**os.environ, "PX4_GZ_HEADLESS": "1"},
                            preexec_fn=os.setsid)
    print(f"  等待 {T_LAUNCH} 秒启动 ...")
    time.sleep(T_LAUNCH)

    try:
        from gz.transport13 import Node
        from gz.msgs10.actuators_pb2 import Actuators
        from pymavlink import mavutil

        # ---------- ① 模型是否加载 ----------
        banner("① 模型加载")
        txt = open(LOG, errors="ignore").read()
        m = re.search(r"model: (\w+)", txt)
        print(f"  gz_bridge 模型名 : {m.group(1) if m else '❌ 未找到'}")

        # ---------- ② IMU 是否持续发布 ----------
        banner(f"② IMU 连续发布检查（{T_IMU} 秒窗口，设计 250 Hz）")
        # ⚠️ 世界名变了话题路径就变！我们的世界叫 airground（不是 default）。
        #    这里两个都订阅，避免"世界改名后静默读到 0 条"这种坑。
        IMU_TOPICS = [
            "/world/airground/model/airground_0/link/base_link/sensor/imu_sensor/imu",
            "/world/default/model/airground_0/link/base_link/sensor/imu_sensor/imu",
        ]
        CNT = [0]
        node = Node()
        imu_type = __import__("gz.msgs10.imu_pb2", fromlist=["IMU"]).IMU
        for t in IMU_TOPICS:
            node.subscribe(imu_type, t, lambda msg: CNT.__setitem__(0, CNT[0] + 1))
        time.sleep(T_IMU)
        exp = 250 * T_IMU
        rtf = CNT[0] / exp
        print(f"  收到 {CNT[0]} 条 / 期望 {exp} 条  →  RTF ≈ {rtf:.3f}")
        print(f"  {'✅ 发布正常' if rtf > 0.8 else '❌ 发布不足！'}")

        # ---------- ②b 【关键】谁在订阅这个话题 ----------
        banner("②b 【关键】IMU 话题的发布者/订阅者")
        for one_topic in IMU_TOPICS:
            try:
                r = subprocess.run(["gz", "topic", "-i", "-t", one_topic],
                                   capture_output=True, text=True, timeout=10)
                info = (r.stdout or r.stderr).strip()
                if not info:
                    continue
                print(f"    [{one_topic.rsplit('/', 3)[0]}/...]")
                for line in info.splitlines():
                    s = line.strip()
                    if s and " " in s:
                        print(f"      {s[:110]}")
                print(f"      → 订阅者 {info.lower().count('subscriber')} 个")
            except Exception as e:
                print(f"    查询失败: {e}")
        conn = mavutil.mavlink_connection("udpout:127.0.0.1:18570",
                                          source_system=255, source_component=0)
        for _ in range(5):
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                    0, 0, 0)
            time.sleep(0.2)
        conn.wait_heartbeat(timeout=10)
        sysid, comp = conn.target_system, conn.target_component
        print(f"\n  ✅ MAVLink 已连接 (system={sysid})")

        CMD = []
        node.subscribe(Actuators, "/airground_0/command/motor_speed",
                       lambda msg: CMD.append(list(msg.velocity)))

        def drain(sec):
            """⚠️ 必须一次抽干缓冲：遥测 ~50 Hz，每次只读一条会严重滞后。
            这个坑我在本项目里已经踩过三次了。"""
            out = []
            t0 = time.time()
            while time.time() - t0 < sec:
                while True:
                    msg = conn.recv_match(blocking=False)
                    if msg is None:
                        break
                    if msg.get_type() == "STATUSTEXT":
                        out.append(msg.text.strip())
                time.sleep(0.005)
            return out

        # ---------- ③ 参数 ----------------
        banner("③ airframe 参数（注意 INT 会读成极小浮点）")
        PARAMS = {}

        def on_param(msg):
            if msg.get_type() == "PARAM_VALUE":
                v = msg.param_value
                # 若数值小到像"非规格化浮点"，按 INT32 重解释
                if 0 < abs(v) < 1e-40:
                    iv = int_param(v)
                    if iv != 0:
                        v = iv
                PARAMS[msg.param_id] = v

        WANT = ["CA_AIRFRAME", "CA_ROTOR_COUNT",
                "CA_ROTOR0_PX", "CA_ROTOR0_PY", "CA_ROTOR0_KM",
                "CA_ROTOR1_KM", "CA_ROTOR2_KM", "CA_ROTOR3_KM",
                "SIM_GZ_EC_FUNC1", "SIM_GZ_EC_FUNC2",
                "SIM_GZ_EC_FUNC3", "SIM_GZ_EC_FUNC4"]
        for p in WANT:
            conn.mav.param_request_read_send(sysid, comp, p.encode(), -1)
        t0 = time.time()
        while time.time() - t0 < 4:
            while True:
                msg = conn.recv_match(blocking=False)
                if msg is None:
                    break
                on_param(msg)
            time.sleep(0.005)

        ok_params = True
        for p in WANT:
            v = PARAMS.get(p, None)
            print(f"    {p:20s} = {v if v is not None else '(未读到)'}")
        if PARAMS.get("CA_ROTOR_COUNT") != 4:
            ok_params = False
        print(f"  {'✅ 控制分配参数正确' if ok_params else '❌ 参数不对！'}")

        # ---------- 静置，验证 STALE 是否自行恢复 ----------
        banner(f"④ 静置 {T_SETTLE} 秒（验证 STALE 是否只是启动瞬态）")
        remainder = max(0, min(T_SETTLE, left() - 20))
        print(f"  等待 {remainder:.0f} 秒 ...")
        time.sleep(remainder)

        # ---------- ⑤ 解锁 ----------
        banner("⑤ 解锁")
        for p, v in ((b"NAV_DLL_ACT", 0), (b"COM_RC_IN_MODE", 1)):
            conn.mav.param_set_send(sysid, comp, p, float(v),
                                    mavutil.mavlink.MAV_PARAM_TYPE_INT32)
            time.sleep(0.3)
        drain(1.0)
        CMD.clear()
        conn.mav.command_long_send(
            sysid, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        res = None
        t0 = time.time()
        while time.time() - t0 < 10:
            while True:
                msg = conn.recv_match(blocking=False)
                if msg is None:
                    break
                tt = msg.get_type()
                if tt == "COMMAND_ACK":
                    print(f"    ACK cmd={msg.command} result={msg.result}")
                    if msg.command == 400:
                        res = msg.result
                elif tt == "STATUSTEXT":
                    print(f"    PX4: {msg.text.strip()}")
            time.sleep(0.005)
        names = {0: "ACCEPTED ✅", 1: "TEMP_REJECTED ❌", 2: "DENIED ❌",
                 3: "UNSUPPORTED", 4: "FAILED ❌"}
        print(f"    ARM 结果 : {names.get(res, res)}")
        print(f"    电机指令 : {[f'{v:.1f}' for v in CMD[-1]] if CMD else '（无）'}")
        conn.mav.command_long_send(
            sysid, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            0, 0, 0, 0, 0, 0, 0)

        # ---------- 汇总 ----------
        banner("⑥ 本次日志里的 STALE 次数")
        txt = open(LOG, errors="ignore").read()
        n_stale = len(re.findall(r"STALE", txt))
        print(f"  STALE 出现 {n_stale} 次")
        for line in set(re.findall(r"ERROR \[[a-z_]+\] .*STALE.*", txt)):
            print(f"    {line[:100]}")

    finally:
        cleanup()
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        logf.close()
        print(f"\n(已清理进程，总耗时 {time.time()-t_start:.0f} 秒)")


if __name__ == "__main__":
    main()
