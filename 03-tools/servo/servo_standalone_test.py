#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
独立模式（不用 PX4）舵机折叠测试 —— 用来回答
    "为什么之前不接 PX4 可以，现在不可以了？"

做法：用【当前的 robot.urdf】启动独立世界 robot_world.sdf，
      直接往 /cmd/arm_angle 发角度指令，测量关节实际角度。

判据：
  · 如果能折到 ~48°  → 模型没问题，差异是 PX4 环境引入的
  · 如果也折不动      → 是模型本身变了（跟 PX4 无关）

用法：python3 servo_standalone_test.py
"""
import math
import os
import subprocess
import time

HERE = os.path.dirname(os.path.abspath(__file__))
WORLD = os.path.join(HERE, "robot_world.sdf")
LOG = "/tmp/servo_standalone.log"

TARGET_DEG = 52.0      # 与 PX4 测试一致
TARGET_RAD = math.radians(TARGET_DEG)

POS = {"joint1": None, "joint2": None}
HIST = {"joint1": [], "joint2": []}


def main():
    for pat in ("gz sim", "gz-sim"):
        subprocess.run(["pkill", "-9", "-f", pat],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    time.sleep(1)

    env = {**os.environ}
    # 让 gz 能找到 world 里引用的 meshes
    env["GZ_SIM_RESOURCE_PATH"] = HERE + ":" + env.get("GZ_SIM_RESOURCE_PATH", "")

    print("=" * 70)
    print("独立模式测试（当前 robot.urdf，不用 PX4）")
    print("=" * 70)
    logf = open(LOG, "w")
    subprocess.Popen(
        ["gz", "sim", "-s", "-r", "-v", "2", WORLD],
        stdout=logf, stderr=subprocess.STDOUT, env=env,
        preexec_fn=os.setsid)

    print("  等待 12 秒让物理稳定 ...")
    time.sleep(12)

    from gz.transport13 import Node
    from gz.msgs10.model_pb2 import Model
    from gz.msgs10.double_pb2 import Double

    def on_joint(msg):
        for j in msg.joint:
            if j.name in POS:
                POS[j.name] = j.axis1.position

    # 【探针】世界坐标：和 PX4 模式对比机身/摆臂的静止高度
    POSE = {}

    def on_pose(msg):
        for p in msg.pose:
            n = p.name.rsplit("::", 1)[-1]
            if n in ("base_link", "base", "link9", "link10", "link5", "link1"):
                POSE[n] = (p.position.x, p.position.y, p.position.z)

    from gz.msgs10.pose_v_pb2 import Pose_V
    node = Node()
    node.subscribe(Model, "/joint_state", on_joint)
    node.subscribe(Pose_V, "/world/default/pose/info", on_pose)
    node.subscribe(Pose_V, "/world/robot/pose/info", on_pose)
    time.sleep(1.5)

    if POSE:
        print("  【世界坐标】")
        for k, v in POSE.items():
            print(f"      {k:10s} x={v[0]:+.3f}  y={v[1]:+.3f}  z={v[2]:+.4f}")
    else:
        print("  【世界坐标】(pose 话题收不到)")

    if POS["joint1"] is None:
        print("  ❌ 收不到 /joint_state —— 世界没起来？看", LOG)
        return

    def show(tag):
        a = [math.degrees(POS[k]) for k in ("joint1", "joint2")]
        print(f"  {tag:22s} joint1={a[0]:+7.2f}°  joint2={a[1]:+7.2f}°")
        return a

    show("① 初始（未发指令）")

    # ---------- 发指令 ----------
    pub = node.advertise("/cmd/arm_angle", Double)
    time.sleep(0.5)
    print(f"\n  发 /cmd/arm_angle = {TARGET_DEG:.1f}° ({TARGET_RAD:.4f} rad)，观察 15 秒：")
    t0 = time.time()
    last = 0
    while time.time() - t0 < 15.0:
        m = Double()
        m.data = TARGET_RAD
        pub.publish(m)
        t = time.time() - t0
        if t - last >= 3.0:
            last = t
            show(f"   t={t:4.1f}s")
        HIST["joint1"].append(math.degrees(POS["joint1"]))
        time.sleep(0.05)

    show("\n② 15 秒后")

    peak = max(HIST["joint1"]) if HIST["joint1"] else 0.0
    print()
    if peak > TARGET_DEG * 0.8:
        print(f"  ✅ 独立模式能折起来（峰值 {peak:+.2f}°）")
        print("     ⇒ 模型本身没问题，差异是【PX4 环境】引入的")
    elif peak > 3.0:
        print(f"  ⚠️ 只折到 {peak:+.2f}°（明显受阻但能动）")
        print("     ⇒ 两边都受阻，只是程度不同")
    else:
        print(f"  ❌ 独立模式也折不起来（峰值 {peak:+.2f}°）")
        print("     ⇒ 【模型本身变了】，跟 PX4 无关")

    print(f"\n(日志 {LOG})")


if __name__ == "__main__":
    try:
        main()
    finally:
        subprocess.run(["pkill", "-9", "-f", "gz sim"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        print("(已清理)")
