#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
aux1 通路逐段排查（借助 PX4 SITL 的 stdin shell）
================================================================================
在一边持续发 aux1 的同时，在 PX4 里跑 listener，看数据卡在哪一段：

  ① 发送端     pymavlink 是否真把 aux1 编码进去      （脚本内自检）
  ② 接收端     /manual_control_input 里 aux1 到了吗
  ③ 选择器     ManualControlSelector(COM_RC_IN_MODE) 是否放行 → /manual_control_setpoint
  ④ 功能映射   gz_bridge 的通道 1 是不是 407
  ⑤ 输出       mixer 实际算出的 output 值

用法：python3 aux_diag.py
"""
import math
import os
import subprocess
import threading
import time

PX4 = "/home/yuan/PX4-Autopilot"
MODEL = "gz_airground"
LOG = "/tmp/aux_diag.log"
BSON = f"{PX4}/build/px4_sitl_default/rootfs/parameters.bson"
T_LAUNCH = 32

OUT = []          # PX4 stdout 累积
SHELL = {"lock": threading.Lock()}


def pxh(cmd, wait=2.5):
    """往 PX4 的 shell 写一条命令，等它回显完，返回截取的输出"""
    mark = f"__MARK_{int(time.time()*1000)}_{abs(hash(cmd)) % 10000}__"
    n0 = len(OUT)
    with SHELL["lock"]:
        SHELL["proc"].stdin.write(f"{cmd}\necho {mark}\n")
        SHELL["proc"].stdin.flush()
    t0 = time.time()
    while time.time() - t0 < wait:
        if any(mark in l for l in OUT[n0:]):
            break
        time.sleep(0.05)
    time.sleep(0.3)
    lines = []
    for l in OUT[n0:]:
        if mark in l or l.strip().startswith("echo " + mark):
            break
        lines.append(l)
    # 去掉回显的那一行命令本身
    if lines and cmd.split()[0] in lines[0]:
        lines = lines[1:]
    return [l for l in lines if l.strip()]


def grab(topics, n=1):
    """listener 一批 topic，返回 {topic: [原始行]}"""
    res = {}
    for t in topics:
        res[t] = pxh(f"listener {t} -n {n}", wait=3.0)
    return res


def show(title, lines, keep=None):
    print(f"\n--- {title} ---")
    if not lines:
        print("   (无输出)")
        return
    for l in lines:
        ls = l.strip()
        if keep and not any(k in ls for k in keep):
            continue
        print("   ", ls[:120])


def main():
    for pat in ("bin/px4", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)
    if os.path.exists(BSON):
        os.remove(BSON)

    print("=" * 78)
    print("aux1 通路逐段排查")
    print("=" * 78)

    logf = open(LOG, "w")
    p = subprocess.Popen(["make", "px4_sitl", MODEL], cwd=PX4,
                         stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                         stderr=subprocess.STDOUT, text=True, bufsize=1,
                         env={**os.environ, "PX4_GZ_HEADLESS": "1"})
    SHELL["proc"] = p

    def reader():
        for line in p.stdout:
            OUT.append(line.rstrip())
            logf.write(line)
            logf.flush()

    threading.Thread(target=reader, daemon=True).start()
    print(f"  等待 {T_LAUNCH} 秒启动 ...")
    time.sleep(T_LAUNCH)

    from gz.transport13 import Node
    from gz.msgs10.model_pb2 import Model
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mav_common

    def enc(target, x, y, z, r, ax1, ax2=0):
        return mav_common.MAVLink_manual_control_message(
            target, x, y, z, r, 0, 0, (1 << 2) | (1 << 3), 0, 0,
            ax1, ax2, 0, 0, 0, 0)

    POS = {"servo_0": None, "servo_1": None}

    def on_joint(msg):
        for j in msg.joint:
            if j.name in POS:
                POS[j.name] = j.axis1.position

    node = Node()
    node.subscribe(Model, "/airground/joint_state", on_joint)
    time.sleep(1.5)

    # ---------------- ① 发送端自检 ----------------
    print("\n① 发送端：pymavlink 编码自检")
    _mav = mav_common.MAVLink(None, srcSystem=255, srcComponent=0)
    _mav.robust_parsing = True
    _t = enc(0, 0, 0, 0, 0, 1000)
    _t._header.srcSystem, _t._header.srcComponent = 255, 0
    buf = _t.pack(_mav)
    dec = _mav.parse_char(buf)
    print(f"   报文 {len(buf)}B | aux1={dec.aux1} | ext={dec.enabled_extensions}"
          f"   {'✅' if dec.aux1 == 1000 else '❌'}")

    # ---------------- 连 MAVLink ----------------
    conn = mavutil.mavlink_connection("udpout:127.0.0.1:18570",
                                      source_system=255, source_component=0)
    for _ in range(5):
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.2)
    conn.wait_heartbeat(timeout=15)
    sysid = conn.target_system
    print(f"\n   ✅ MAVLink 已连接 (sys={sysid})")

    AUX = {"v": 0.0, "stop": False}

    def stick():
        while not AUX["stop"]:
            conn.mav.send(enc(sysid, 0, 0, 0, 0, int(round(AUX["v"] * 1000))))
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.05)

    threading.Thread(target=stick, daemon=True).start()

    # ---------------- ② 接收端：manual_control_input ----------------
    print("\n② 接收端：MANUAL_CONTROL 有没有被 PX4 收到？")
    AUX["v"] = 1.0
    time.sleep(2.0)
    r = grab(["manual_control_input"], n=1)
    show("manual_control_input（PX4 收报文后发布的原始输入）", r["manual_control_input"],
         keep=["aux1", "valid", "data_source", "roll", "pitch"])
    got_input = any("aux1: 1.00000" in l for l in r["manual_control_input"])
    print(f"   ⇒ aux1 到达接收端: {'✅' if got_input else '❌'}")

    # ---------------- ③ 选择器：COM_RC_IN_MODE ----------------
    print("\n③ 选择器：COM_RC_IN_MODE 是否放行？")
    show("当前值", pxh("param show COM_RC_IN_MODE"), keep=["COM_RC_IN_MODE"])
    r2 = grab(["manual_control_setpoint"], n=1)
    show("manual_control_setpoint（选择器之后，FunctionManualRC 读的就是它）",
         r2["manual_control_setpoint"], keep=["aux1", "valid", "data_source"])
    got_sp = any("valid: True" in l for l in r2["manual_control_setpoint"])
    print(f"   ⇒ setpoint 有效: {'✅' if got_sp else '❌ 被卡住了'}")
    if not got_sp:
        print("   ⇒ 试：把 COM_RC_IN_MODE 设成 1 再发一次")
        pxh("param set COM_RC_IN_MODE 1")
        time.sleep(1.5)
        r2b = grab(["manual_control_setpoint"], n=1)
        show("设完后 manual_control_setpoint", r2b["manual_control_setpoint"],
             keep=["aux1", "valid", "data_source"])
        got_sp = any("valid: True" in l for l in r2b["manual_control_setpoint"])
        print(f"   ⇒ 现在是: {'✅ 有效' if got_sp else '❌ 仍无效'}")

    # ---------------- ④ 功能映射 ----------------
    print("\n④ 功能映射：gz_bridge 通道 1 挂的是什么功能？")
    show("gz_bridge status", pxh("gz_bridge status", wait=3.0),
         keep=["Channel", "func", "value", "1:", "2:"])
    show("SIM_GZ_SV_FUNC1", pxh("param show SIM_GZ_SV_FUNC1"), keep=["FUNC1"])

    # ---------------- ⑤ 输出 ----------------
    print("\n⑤ 输出：解锁后摆臂跟不跟 aux1？")
    print("   ⑤a 先用【MAVLink ARM】（QGC 走的就是这条路）")

    def deg():
        return [math.degrees(POS[k]) if POS[k] is not None else float("nan")
                for k in ("servo_0", "servo_1")]

    def sweep(tag):
        for v in (-1.0, 0.0, 1.0):
            AUX["v"] = v
            t0 = time.time()
            while time.time() - t0 < 4.0:
                conn.recv_match(blocking=False)
                time.sleep(0.01)
            a = deg()
            print(f"   {tag} aux1={v:+.2f}   servo_0={a[0]:+7.2f}°  "
                  f"servo_1={a[1]:+7.2f}°")
        return deg()[0]

    conn.mav.command_long_send(sysid, conn.target_component,
                               mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                               0, 1, 0, 0, 0, 0, 0, 0)
    t0 = time.time()
    armed = False
    while time.time() - t0 < 12.0:
        m = conn.recv_match(type="HEARTBEAT", blocking=False)
        if m is not None and m.get_srcSystem() == sysid:
            if m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED:
                armed = True
        time.sleep(0.1)
    print(f"   MAVLink ARM → armed = {armed}")
    top = sweep("a) MAVLink arm:")

    if top < 40:
        print("\n   ⑤b 换成 PX4 内部命令 commander arm")
        show("commander arm", pxh("commander arm", wait=5.0),
             keep=["arm", "Arm", "ARM", "WARN", "ERROR", "Disarm"])
        top = sweep("b) pxh arm:  ")

    print()
    if top > 40:
        print("   ✅ aux1 通路成立：连续通道可以直接驱动摆臂")
    else:
        print("   ❌ 仍不通")

    # ---------------- 清理 ----------------
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
