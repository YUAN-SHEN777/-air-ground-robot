#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""drift_full_test.py —— 1.0 油门直线漂移补充实验（当前 9/10 路架构）

复用 verify_steer 的位姿/位移测量与 verify_reversible 的 Link，
但额外周期发送 10 路 135°（187 param7=2 / 310 1302），
使 AirGroundMux 真正切到地面位。

输出：10 s 内的纵向位移、横向漂移、Δyaw、平均速度。
"""

import atexit
import os
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")
os.environ.setdefault("HEADLESS", "1")

import tilt_yaw_px4 as T
import verify_steer as V


class GroundLink(V.Link):
    """在旧 Link 的 9 路命令之外，补发 10 路模式切换 135°。"""

    def __init__(self):
        super().__init__()
        self.mode_servo = 1.0          # +1 → SIM_GZ_SV_MINA2/MAXA2 45/135
        self.t_310_mode = 0.0
        self.t_187_mode = 0.0

    def pump(self):
        super().pump()
        now = time.time()

        if now - self.t_310_mode > 0.2:
            # 310: param5 = 1000 + 功能号 302
            self._cmd_long(310, self.mode_servo, 0, 0, 0, 1302)
            self.t_310_mode = now

        if now - self.t_187_mode > 0.5:
            # 187: param7=2 → 只写 10 路
            self._cmd_long(187, self.mode_servo, 0, 0, 0, 0, 0, 2)
            self.t_187_mode = now

        self._drain()


def main():
    arm_deg = float(os.environ.get("ARM_DEG", "52"))
    throttle = float(os.environ.get("THR", "1.0"))
    T.cleanup()
    if not T.boot():
        print("❌ PX4 SITL 未就绪")
        return 1
    atexit.register(T.cleanup)

    link = GroundLink()
    with V.Pump(link):
        print("① 摆臂 → 轮位；10 路 → 135°；切 GROUND_DRIVE；解锁")
        V.setup(link)
        if abs(arm_deg - 52.0) > 1e-6:
            link.servo = arm_deg / 26.0 - 1.0
            link.hold(6.0)
        print(f"   就绪（摆臂指令 {arm_deg:.1f}°）")

        print(f"② 油门 {throttle:.2f} 直线 10 s（steer=0）")
        r = V.phase(link, f"thr {throttle:.2f} / steer 0 / 10s", throttle, 0.0, 10.0)

        if r:
            dist = max(r["dist"], 1e-9)
            print("\n=== RESULT ===")
            print(f"t        = {r['secs']:.3f} s")
            print(f"lon      = {r['lon']:+.4f} m")
            print(f"lat      = {r['lat']:+.6f} m")
            print(f"dist     = {r['dist']:.4f} m")
            print(f"drift    = {abs(r['lat']) / dist * 100:.4f} %")
            print(f"dyaw     = {r['dyaw']:+.4f} deg")
            print(f"v_avg    = {r['lon'] / max(r['secs'], 1e-9):.4f} m/s")
            print(f"motors   = {V.fmt_motors(r['motors'])}")

        V.stop(link, 2.0)
        link.arm(False, force=True)
        link.hold(2.0)

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
