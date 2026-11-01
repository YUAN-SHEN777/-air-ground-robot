#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""test_ground_gate.py —— 验证 ground_drive 的模式门控只看 10 路。

步骤：
  1. 10 路切到 135°，等待 3 s（外推 45→135 约 2.25 s）；
  2. 发 DO_SET_MODE 进入 GROUND_DRIVE（nav_state=23）；
  3. 10 路切回 45°，确认【不会】自动退出 GROUND_DRIVE（防止空中误切）；
     此时 AirGroundMux 应切回空中位、轮子停转。
"""

import atexit
import time

from pymavlink import mavutil

import tilt_yaw_px4 as T
import test_mode_switch as M


def read_mode(conn, timeout=4.0, want_external=None):
    """返回 (main_mode, sub_mode)；external1 = main 4 / sub 11。"""
    t0 = time.time()
    last = None
    while time.time() - t0 < timeout:
        m = conn.recv_match(type="HEARTBEAT", blocking=True, timeout=1)
        if m is None:
            continue
        main = (m.custom_mode >> 16) & 0xFF
        sub = (m.custom_mode >> 24) & 0xFF
        last = (main, sub)
        is_ext = (main == 4 and sub == 11)
        if want_external is None:
            return last
        if want_external and is_ext:
            return last
        if (not want_external) and (not is_ext):
            return last
    return last


def main():
    T.cleanup()
    if not T.boot():
        return 1
    atexit.register(T.cleanup)

    conn = mavutil.mavlink_connection(
        "udpout:127.0.0.1:18570", source_system=255, source_component=0)
    conn.param_set_send("COM_PREARM_MODE", 2,
                        mavutil.mavlink.MAV_PARAM_TYPE_INT32)
    time.sleep(7)

    print("① 10 路 → 135°（地面）")
    M.send_loop(conn, 4.0, mode=1.0)

    print("② 尝试进入 GROUND_DRIVE")
    conn.mav.command_long_send(1, 1, 176, 0, 1, 4, 11, 0, 0, 0, 0)
    mode = read_mode(conn, timeout=4.0, want_external=True)
    ok = (mode == (4, 11))
    print(f"   mode(main,sub) = {mode}  {'✅ GROUND_DRIVE' if ok else '❌ 未进入'}")

    print("③ 10 路 → 45°（空中）")
    M.send_loop(conn, 4.0, mode=-1.0)
    mode2 = read_mode(conn, timeout=2.0, want_external=True)
    ok2 = (mode2 == (4, 11))
    print(f"   mode(main,sub) = {mode2}  "
          f"{'✅ 仍保持 GROUND_DRIVE（不自动退出，符合设计）' if ok2 else '❌ 被自动退出'}")

    print("\n=== ground_drive / mux 日志 ===")
    for ln in open(T.LOG, errors="ignore").read().splitlines():
        if ("ground_drive" in ln) or ("AirGroundMux] mode" in ln):
            print(ln)

    return 0 if (ok and ok2) else 1


if __name__ == "__main__":
    raise SystemExit(main())
