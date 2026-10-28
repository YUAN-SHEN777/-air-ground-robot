#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
舵机通路深度探针 —— 直接问 PX4 内部状态
================================================================================
思路：与其猜，不如让 PX4 自己说。
  PX4 SITL 默认开启 pxh shell 并读 stdin（`-d` 才关闭），
  所以可以用 stdin 给它发 `gz_bridge status`，
  该命令会调用 MixingOutput::printStatus()，打印每个输出通道的：
      func / value / failsafe / disarmed / min / max / center

对比三个时刻的状态：
  ① 未解锁
  ② 解锁后
  ③ 发完 DO_SET_ACTUATOR(+1.0) 之后
就能确定卡在哪一环。

用法：
    python3 servo_probe.py
"""
import os
import subprocess
import time

PX4 = "/home/yuan/PX4-Autopilot"
MODEL = "gz_airground"
LOG = "/tmp/servo_probe.log"
BSON = f"{PX4}/build/px4_sitl_default/rootfs/parameters.bson"

T_LAUNCH = 30


def banner(s):
    print("\n" + "=" * 74)
    print(s)
    print("=" * 74)


def cleanup():
    for pat in ("bin/px4", "gz sim"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    cleanup()
    time.sleep(1)
    if os.path.exists(BSON):
        os.remove(BSON)
        print(f"已删除 {BSON}")

    banner(f"启动 PX4 + {MODEL}（stdin 接管，用于发 shell 命令）")
    logf = open(LOG, "w")
    proc = subprocess.Popen(["make", "px4_sitl", MODEL], cwd=PX4,
                            stdin=subprocess.PIPE,
                            stdout=logf, stderr=subprocess.STDOUT,
                            env={**os.environ, "PX4_GZ_HEADLESS": "1"},
                            text=True)
    print(f"  等待 {T_LAUNCH} 秒 ...")
    time.sleep(T_LAUNCH)

    def sh(cmd, wait=1.5, label=""):
        """给 PX4 的 shell 发一条命令，并把新出现的日志打出来"""
        mark = logf.tell()
        logf.flush()
        try:
            proc.stdin.write(cmd + "\n")
            proc.stdin.flush()
        except Exception as e:
            print(f"    ⚠️ 写入 stdin 失败: {e}")
            return ""
        time.sleep(wait)
        logf.flush()
        with open(LOG, errors="ignore") as f:
            f.seek(mark)
            out = f.read()
        if label:
            print(f"\n--- {label} （$ {cmd}）---")
            for line in out.splitlines():
                s = line.strip()
                if s and not s.startswith("pxh>"):
                    print(f"    {s}")
        return out

    try:
        from pymavlink import mavutil
        import threading

        conn = mavutil.mavlink_connection("udpout:127.0.0.1:18570",
                                          source_system=255, source_component=0)
        for _ in range(5):
            conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            time.sleep(0.2)
        conn.wait_heartbeat(timeout=15)
        sysid, comp = conn.target_system, conn.target_component
        print(f"  ✅ MAVLink 已连接 (system={sysid})")

        stop = {"v": False}

        def stick():
            while not stop["v"]:
                conn.mav.manual_control_send(sysid, 0, 0, 0, 0, 0)
                time.sleep(0.1)

        threading.Thread(target=stick, daemon=True).start()

        def poll(sec):
            """抽干 MAVLink"""
            t0 = time.time()
            armed = False
            while time.time() - t0 < sec:
                while True:
                    m = conn.recv_match(blocking=False)
                    if m is None:
                        break
                    if m.get_type() == "HEARTBEAT":
                        armed = bool(m.base_mode
                                     & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                time.sleep(0.005)
            return armed

        # ---------- ① 未解锁 ----------
        banner("① 未解锁时的通道状态")
        sh("gz_bridge status", 2.5, "未解锁")

        # ---------- ② 解锁 ----------
        banner("② 解锁")
        time.sleep(3.0)
        conn.mav.command_long_send(
            sysid, comp, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
            1, 0, 0, 0, 0, 0, 0)
        armed = poll(8.0)
        print(f"    【真实解锁位】: {armed}  {'✅' if armed else '❌'}")
        sh("gz_bridge status", 2.5, "已解锁")

        # ---------- ③ 发舵机命令 ----------
        banner("③ 发 DO_SET_ACTUATOR(+1.0)，再看状态")
        conn.mav.command_long_send(sysid, comp, 187, 0, 1.0, 0, 0, 0, 0, 0, 0)
        poll(2.0)
        sh("gz_bridge status", 2.5, "命令后")

        # ---------- ④ 换个功能号验证 ----------
        banner("④ 附加：查 vehicle_command 是否真的进 uORB")
        sh("listener vehicle_command -n 3", 3.0, "当前 vehicle_command")

        stop["v"] = True

    finally:
        cleanup()
        try:
            logf.close()
        except Exception:
            pass
        print(f"\n(已清理) 完整日志: {LOG}")


if __name__ == "__main__":
    main()
