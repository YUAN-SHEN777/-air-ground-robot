#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
验证：能不能用【一个连续通道（滑条）】直接控制摆臂舵机？
================================================================================
思路：把舵机功能号临时改成 407（RC_AUX1），
      再发带 aux1 扩展字段的 MANUAL_CONTROL，看摆臂跟不跟。

为什么这样可行（源码依据）：
  · FunctionManualRC.hpp：
        _data[5] = manual_control_setpoint.aux1;     ← 功能号 407 = RC_AUX1
  · ManualControlSetpoint.msg：有 aux1~aux6 字段
  · common.xml 的 MANUAL_CONTROL(69) 扩展字段里有 aux1~aux6
        enabled_extensions 的 bit2 = aux1 有效
  · mavlink_receiver.cpp:2343：
        manual_control_setpoint.aux1 = mavlink_manual_control.aux1 / 1000.0f

用法：python3 servo_aux_test.py
"""
import math
import os
import struct
import subprocess
import threading
import time

PX4 = "/home/yuan/PX4-Autopilot"
MODEL = "gz_airground"
LOG = "/tmp/servo_aux_test.log"
BSON = f"{PX4}/build/px4_sitl_default/rootfs/parameters.bson"
T_LAUNCH = 30

POS = {"servo_0": None, "servo_1": None}
OUT = []
SHELL = {"lock": threading.Lock()}


def pxh(cmd, wait=2.5):
    """往 PX4 的 shell 写一条命令并等回显（用 mark 划边界）"""
    mark = f"__M{abs(hash(cmd)) % 100000}__"
    n0 = len(OUT)
    with SHELL["lock"]:
        SHELL["proc"].stdin.write(f"{cmd}\necho {mark}\n")
        SHELL["proc"].stdin.flush()
    t0 = time.time()
    while time.time() - t0 < wait:
        if any(mark in l for l in OUT[n0:]):
            break
        time.sleep(0.05)
    time.sleep(0.2)
    lines = []
    for l in OUT[n0:]:
        if mark in l:
            break
        lines.append(l)
    if lines and cmd.split()[0] in lines[0]:
        lines = lines[1:]
    return [l.strip() for l in lines if l.strip()]


def main():
    for pat in ("bin/px4", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    if os.path.exists(BSON):
        os.remove(BSON)

    print("=" * 74)
    print("验证：连续通道（aux1）能否直接控制摆臂舵机")
    print("=" * 74)
    logf = open(LOG, "w")
    _proc = subprocess.Popen(["make", "px4_sitl", MODEL], cwd=PX4,
                             stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                             stderr=subprocess.STDOUT, text=True, bufsize=1,
                             env={**os.environ, "PX4_GZ_HEADLESS": "1"})
    SHELL["proc"] = _proc

    def _reader():
        for line in _proc.stdout:
            OUT.append(line.rstrip())
            logf.write(line)
            logf.flush()

    threading.Thread(target=_reader, daemon=True).start()
    print(f"  等待 {T_LAUNCH} 秒启动 ...")
    time.sleep(T_LAUNCH)

    from gz.transport13 import Node
    from gz.msgs10.model_pb2 import Model
    from pymavlink import mavutil
    # ⚠️ 必须用 v20.common 方言！
    #    mavutil 默认解析成 ardupilotmega，那个方言的 MANUAL_CONTROL
    #    只有 target/x/y/z/r/buttons，【没有 aux1~aux6 扩展字段】。
    #    v20.common 里才有：buttons2, enabled_extensions, s, t, aux1..aux6
    from pymavlink.dialects.v20 import common as mav_common

    def enc_manual_control(target, x, y, z, r, ax1):
        """手工编一帧带 aux1 扩展的 MANUAL_CONTROL
        enabled_extensions: bit2 = aux1 有效"""
        m = mav_common.MAVLink_manual_control_message(
            target, x, y, z, r, 0,        # target,x,y,z,r,buttons
            0,                            # buttons2
            1 << 2,                       # enabled_extensions: bit2=aux1
            0, 0,                         # s, t
            ax1, 0, 0, 0, 0, 0)           # aux1..aux6
        return m

    # ---- 自检：确认 pymavlink 真的把 aux1 打进了字节流 ----
    _mav = mav_common.MAVLink(None, srcSystem=255, srcComponent=0)
    _mav.robust_parsing = True
    _t = enc_manual_control(0, 0, 0, 0, 0, 1000)
    _t._header.srcSystem, _t._header.srcComponent = 255, 0
    _buf = _t.pack(_mav)
    _dec = _mav.parse_char(_buf)
    _ax = getattr(_dec, "aux1", None)
    print(f"   自检：报文 {len(_buf)}B  aux1 解码回={_ax} "
          f"ext={getattr(_dec, 'enabled_extensions', None)}  "
          f"{'✅' if _ax == 1000 else '❌ 打包丢了扩字段'}")
    if _ax != 1000:
        print("   ⇒ pymavlink 版本不支持 aux 扩字段，需升级或手改方言")
        return

    def on_joint(msg):
        for j in msg.joint:
            if j.name in POS:
                POS[j.name] = j.axis1.position

    node = Node()
    node.subscribe(Model, "/airground/joint_state", on_joint)
    time.sleep(1.5)

    conn = mavutil.mavlink_connection("udpout:127.0.0.1:18570",
                                      source_system=255, source_component=0)
    for _ in range(5):
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.2)
    conn.wait_heartbeat(timeout=15)
    sysid, comp = conn.target_system, conn.target_component
    print(f"  ✅ MAVLink 已连接 (system={sysid})")

    # ---------- ① 确认功能号已是 407 ----------
    # ⚠️ 不能靠运行时 param_set 改 FUNC！
    #    mixer_module.cpp: updateSubscriptions()
    #        if (!_need_function_update || _armed.armed) return false;
    #    解锁后该参数不重建映射，改了也白改。
    #    必须先：  SERVO_SRC=aux1 python3 build_px4_model.py
    #    让它写进 airframe（启动就生效）。
    print("\n① 回读 SIM_GZ_SV_FUNC1（应为 407 = RC_AUX1）")

    def _pid(m):
        v = m.param_id
        return v.decode() if isinstance(v, (bytes, bytearray)) else str(v)

    def _as_int32(fval):
        """PX4 把 INT32 参数的位模式塞进 float32 字段发出来，
        直接读会得到 denormal（例：407 → 5.70328e-43）。
        按位重解释才能拿回真值。"""
        return struct.unpack("<i", struct.pack("<f", fval))[0]

    func_val = None
    t0 = time.time()
    while time.time() - t0 < 5.0:
        conn.mav.param_request_read_send(sysid, comp, b"SIM_GZ_SV_FUNC1", -1)
        for _ in range(20):
            m = conn.recv_match(type="PARAM_VALUE", blocking=False)
            if m is not None and _pid(m).strip("\x00") == "SIM_GZ_SV_FUNC1":
                func_val = _as_int32(m.param_value)
            time.sleep(0.02)
        if func_val is not None:
            break
    print(f"   SIM_GZ_SV_FUNC1 = {func_val:g}" if func_val is not None
          else "   ❌ 没读到")
    if func_val != 407:
        print("   ❌ 功能号不是 407 —— 请先跑：")
        print("        SERVO_SRC=aux1 python3 build_px4_model.py")
        print("        然后重新 make px4_sitl gz_airground")
        for pat in ("bin/px4", "gz sim"):
            subprocess.run(["pkill", "-9", "-f", pat],
                           stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        if os.path.exists(BSON):
            os.remove(BSON)
        return

    # ⚠️ 默认 COM_RC_IN_MODE=0 是 RcOnly，会直接丢弃 MAVLink 手动输入！
    #    ManualControlSelector.cpp:  case RcOnly: match = isRc(data_source);
    #    必须设成 1 (MavLinkOnly) 才让 manual_control_input 通过 → setpoint
    # ⚠️ 用 pxh 设参数而不是 MAVLink param_set：
    #    实测 MAVLink 批量 param_set（尤其是 SIM_GZ_EC_FUNC*）会让 gz_bridge
    #    的函数映射重建卡住，导致后续函数输出全变 NAN。
    print("   用 pxh 设参数（COM_RC_IN_MODE=1 + 关自动上锁 + 屏蔽电机）")
    for c in ("param set COM_RC_IN_MODE 1",
              "param set COM_DISARM_LAND 0",
              "param set COM_DISARM_PRFLT 0"):
        pxh(c)
    for i in range(1, 5):
        pxh(f"param set SIM_GZ_EC_FUNC{i} 0")
        pxh(f"param set SIM_GZ_EC_DIS{i} 0")
    time.sleep(1.0)

    # ---------- ② 持续发 MANUAL_CONTROL（带 aux1）----------
    AUX = {"v": 0.0, "stop": False}

    def stick():
        while not AUX["stop"]:
            ax1 = int(round(AUX["v"] * 1000))
            conn.mav.send(enc_manual_control(sysid, 0, 0, 0, 0, ax1))
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.1)

    threading.Thread(target=stick, daemon=True).start()
    time.sleep(1.0)

    # ---------- ③ 解锁 ----------
    ARMED = {"v": None}

    def armed_now():
        while True:
            m = conn.recv_match(type="HEARTBEAT", blocking=False)
            if m is None:
                break
            if m.get_srcSystem() == sysid:
                ARMED["v"] = bool(m.base_mode
                                  & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
        return ARMED["v"]

    print("\n② 解锁（mixer 未解锁就不输出函数值）")
    conn.mav.command_long_send(
        sysid, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0, 1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    armed_flag = False
    while time.time() - t0 < 12.0:
        if armed_now() is True:
            armed_flag = True
            break
        time.sleep(0.1)
    print(f"   真实解锁位: {armed_flag}   {'✅' if armed_flag else '❌ 没解锁'}")
    if not armed_flag:
        print("   放弃（后面检查会因未解锁而全部无效）")

    # ---------- ④ 扫 aux1 ----------
    def measure(sec, tag):
        t0 = time.time()
        while time.time() - t0 < sec:
            conn.recv_match(blocking=False)
            time.sleep(0.01)
        a = [math.degrees(POS[k]) if POS[k] is not None else float("nan")
             for k in ("servo_0", "servo_1")]
        print(f"   {tag:28s} aux1={AUX['v']:+.2f}  "
              f"servo_0={a[0]:+7.2f}°  servo_1={a[1]:+7.2f}°")
        return a

    print("\n③ 扫 aux1：-1 → 0 → +1")
    AUX["v"] = -1.0
    measure(4.0, "aux1 = -1（应 → 0°）")
    AUX["v"] = 0.0
    measure(4.0, "aux1 =  0（应 → 26°）")
    AUX["v"] = 1.0
    a = measure(6.0, "aux1 = +1（应 → 52°）")

    print()
    if armed_flag and a[0] > 40:
        print("  ✅ 通路成立：一个连续通道（aux1）就能直接驱动摆臂")
        print("     ⇒ QGC 里只要能发 aux1，就能用一个滑条控制")
        print("     ⚠️ 但注意：输入超时/断开时 FunctionManualRC 会 reset 成 NAN，")
        print("        摆臂回落到 disarmed 位 —— 不能“定住”在某个角度")
    elif not armed_flag:
        print("  ❌ 未解锁 —— 本次测量无效")
    else:
        print("  ❌ 已解锁但没反应 —— aux1 真的不通")

    AUX["stop"] = True
    for pat in ("bin/px4", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    if os.path.exists(BSON):
        os.remove(BSON)
    print(f"\n(已清理，日志 {LOG})")


if __name__ == "__main__":
    try:
        main()
    except Exception:
        import traceback
        traceback.print_exc()
