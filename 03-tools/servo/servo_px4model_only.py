#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
二分实验：把【PX4 生成的 model.sdf】放到普通世界跑，完全脱离 PX4。
直接往 /model/airground_0/servo_0 发角度（弧度），看摆臂能不能折叠。

判据：
  · 能折   → model.sdf 没问题，差异是【PX4 运行时】引入的
  · 不能折 → model.sdf 与 robot.urdf 之间有隐藏差异

用法：python3 servo_px4model_only.py
"""
import math
import os
import subprocess
import time

PX4 = "/home/yuan/PX4-Autopilot"
HERE = os.path.dirname(os.path.abspath(__file__))
MODELS = f"{PX4}/Tools/simulation/gz/models"
TMP_WORLD = "/tmp/px4model_world.sdf"
LOG = "/tmp/servo_px4model_only.log"
TARGET_RAD = math.radians(52.0)

# 诊断用：模型初始高度。>0.5 表示悬空（脱离地面接触），
# 用来判断"4 ms 步长卡死"到底跟地面接触有没有关系。
# 用法：SPAWN_Z=2.0 python3 servo_px4model_only.py
SPAWN_Z = os.environ.get("SPAWN_Z", "0.05")

POS = {"servo_0": None, "servo_1": None}


def build_world():
    """用【我们生成的世界】airground.sdf（这样 STEP_SIZE / GRAVITY 等
    诊断开关都会生效），插入模型引用后写到 /tmp。"""
    src = f"{PX4}/Tools/simulation/gz/worlds/airground.sdf"
    if not os.path.exists(src):
        src = f"{PX4}/Tools/simulation/gz/worlds/default.sdf"
    t = open(src, encoding="utf-8").read()
    inc = ('    <include>\n      <uri>model://airground</uri>\n'
           '      <name>airground</name>\n'
           f'      <pose>0 0 {SPAWN_Z} 0 0 0</pose>\n    </include>\n')
    t = t.replace("</world>", inc + "  </world>")
    open(TMP_WORLD, "w", encoding="utf-8").write(t)
    print(f"  模型初始高度 z={SPAWN_Z}"
          + ("（悬空，脱离地面接触）" if float(SPAWN_Z) > 0.5 else ""))
    import re as _re
    m = _re.search(r'<physics type="(\w+)">.*?<max_step_size>([^<]*)</max_step_size>',
                   t, _re.S)
    if m:
        print(f"  世界参数: physics={m.group(1)}, step={m.group(2)} s")


def main():
    for pat in ("gz sim", "bin/px4"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)

    build_world()
    env = {**os.environ, "GZ_SIM_RESOURCE_PATH": MODELS}

    print("=" * 70)
    print("二分实验：只用 PX4 的 model.sdf，不接 PX4")
    print("=" * 70)
    logf = open(LOG, "w")
    subprocess.Popen(["gz", "sim", "-s", "-r", "-v", "2", TMP_WORLD],
                     stdout=logf, stderr=subprocess.STDOUT, env=env,
                     preexec_fn=os.setsid)
    print("  等待 12 秒 ...")
    time.sleep(12)

    from gz.transport13 import Node
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.double_pb2 import Double

    def on_joint(msg):
        for j in msg.joint:
            if j.name in POS:
                POS[j.name] = j.axis1.position

    node = Node()
    node.subscribe(Model, "/airground/joint_state", on_joint)
    node.subscribe(Model, "/joint_state", on_joint)
    time.sleep(2.0)

    if POS["servo_0"] is None:
        print("  ❌ 收不到关节状态 —— 模型没起来？看", LOG)
        return

    def show(tag):
        a = [math.degrees(POS[k]) if POS[k] is not None else float("nan")
             for k in ("servo_0", "servo_1")]
        print(f"  {tag:20s} servo_0={a[0]:+7.2f}°  servo_1={a[1]:+7.2f}°")

    show("① 初始")
    pub = node.advertise("/model/airground_0/servo_0", Double)
    time.sleep(0.5)
    print(f"\n  发 servo_0 = {math.degrees(TARGET_RAD):.1f}°，观察 12 秒：")
    peak = -1e9
    t0 = time.time()
    last = 0
    while time.time() - t0 < 12.0:
        m = Double()
        m.data = TARGET_RAD
        pub.publish(m)
        t = time.time() - t0
        if POS["servo_0"] is not None:
            peak = max(peak, math.degrees(POS["servo_0"]))
        if t - last >= 3.0:
            last = t
            show(f"   t={t:4.1f}s")
        time.sleep(0.05)
    show("② 12 秒后")

    print()
    if peak > 40:
        print(f"  ✅ 能折（峰值 {peak:+.2f}°）⇒ model.sdf 没问题")
        print("     差异是【PX4 运行时】引入的（PX4 在跑，占了 CPU / 改了状态）")
    else:
        print(f"  ❌ 也折不动（峰值 {peak:+.2f}°）⇒ model.sdf 与 robot.urdf 有差异")
    print(f"\n(日志 {LOG})")


if __name__ == "__main__":
    try:
        main()
    finally:
        subprocess.run(["pkill", "-9", "-f", "gz sim"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("(已清理)")
