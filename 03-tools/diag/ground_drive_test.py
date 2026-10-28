#!/usr/bin/env python3
"""
ground_drive_test.py —— 地面驱动（GROUND_DRIVE）仿真测试台

为什么需要它
------------
`MAV_CMD_ACTUATOR_TEST (310)` 在 PX4 里是【有超时的】：约 3 秒不重发，
mixer 就回落到 disarmed 值（摆臂 0°）。所以"摆一下就走"在仿真里不成立，
必须像 QGC 滑条那样【持续重发】才能把摆臂保持在 52° 地面位。

本脚本同时做四件事：
  ① 发 heartbeat（PX4 需要，否则报 gcs_connection_lost）
  ② 发 MANUAL_CONTROL（摇杆；仿真里没有遥控器，不发力模式都 arm 不了）
  ③ 5 Hz 重发 310（ACTUATOR_TEST）——【未解锁】时驱动舵机，但它 3 s 会超时
  ④ 2 Hz 重发 187（DO_SET_ACTUATOR）——【已解锁】时靠它锁存保持，无超时

⚠️ 为什么两个都要发？
   · 310 走 ActuatorTest（override），绕过 armed 检查 → 未解锁也能动，但有超时
   · 187 走 FunctionActuatorSet（订阅 vehicle_command）→ 锁存无超时，但未解锁时会被 DIS 值覆盖
   ⇒ 两个同时发，才能在「解锁/未解锁」两种状态下都把摆臂钉在目标角度。

用法
----
  python3 ground_drive_test.py --arm 1.0 --throttle 0.5 --steer 0 --secs 10
  python3 ground_drive_test.py --arm 1.0 --throttle -0.5 --secs 8     # 后退
  python3 ground_drive_test.py --arm 1.0 --throttle 0 --steer 0.5 --secs 8  # 原地转

参数
----
  --arm       舵机指令 [-1, 1]：-1→0°（桨位）；+1→52°（轮位）
  --throttle  油门 [-1, 1]：0 = 停车，+1 = 满前进，-1 = 满后退
  --steer     转向 [-1, 1]：右摇杆横向（roll）
  --secs      持续时间 [s]

⚠️ MAVLink MANUAL_CONTROL 的 z 是【单极 0..1000】，500 才是中点
   （PX4: throttle = z/500 - 1），本脚本内部已换算，你只管给 [-1,1]。
"""

import argparse
import time

from pymavlink import mavutil

CMD_DO_SET_ACTUATOR = 187
CMD_ACTUATOR_TEST = 310
ACT_FUNC_PARAM = 1301.0          # 1000 + SIM_GZ_SV_FUNC1(301)


def main() -> int:
    ap = argparse.ArgumentParser(description="GROUND_DRIVE 仿真测试台")
    ap.add_argument("--arm", type=float, default=1.0,
                    help="舵机指令 [-1,1]：-1→0°, +1→52°")
    ap.add_argument("--throttle", type=float, default=0.0,
                    help="油门 [-1,1]")
    ap.add_argument("--steer", type=float, default=0.0,
                    help="转向（roll）[-1,1]")
    ap.add_argument("--secs", type=float, default=8.0, help="持续时间 [s]")
    ap.add_argument("--port", type=int, default=18570, help="PX4 MAVLink UDP 端口")
    ap.add_argument("--arm-hz", type=float, default=5.0, help="310 重发频率 [Hz]")
    ap.add_argument("--set-hz", type=float, default=2.0, help="187 重发频率 [Hz]")
    args = ap.parse_args()

    arm = max(-1.0, min(1.0, args.arm))
    thr = max(-1.0, min(1.0, args.throttle))
    st = max(-1.0, min(1.0, args.steer))

    # 换算成 MAVLink 单位
    z = int(round(500.0 + 500.0 * thr))      # [0, 1000]，500 = 中点
    y = int(round(1000.0 * st))              # [-1000, 1000]

    conn = mavutil.mavlink_connection(
        f"udpout:127.0.0.1:{args.port}", source_system=255, source_component=0)

    print(f"舵机指令 {arm:+.2f} → 目标角 {(arm + 1) * 26:.0f}°；"
          f"油门 {thr:+.2f}（z={z}）；转向 {st:+.2f}（y={y}）；持续 {args.secs:.0f}s")

    period = 0.1
    arm_period = 1.0 / max(args.arm_hz, 0.5)
    set_period = 1.0 / max(args.set_hz, 0.2)
    t_end = time.time() + args.secs
    t_arm = 0.0
    t_set = 0.0
    n = 0

    while time.time() < t_end:
        conn.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        conn.mav.manual_control_send(1, 0, y, z, 0, 0)

        now = time.time()

        # 310：未解锁时驱动舵机（有超时，需高频重发）
        if (now - t_arm) >= arm_period:
            conn.mav.command_long_send(
                1, 1, CMD_ACTUATOR_TEST, 0,
                arm, 1.0, 0.0, 0.0, ACT_FUNC_PARAM, 0.0, 0.0)
            t_arm = now

        # 187：已解锁时锁存保持（无超时）；param7=0 ⇒ param1~6 对应执行器 1~6
        if (now - t_set) >= set_period:
            conn.mav.command_long_send(
                1, 1, CMD_DO_SET_ACTUATOR, 0,
                arm, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            t_set = now

        n += 1
        time.sleep(period)

    print(f"结束（发了 {n} 轮）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
