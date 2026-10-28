#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
四桨转向分配对照实验
====================
目的：确认 turningDirection 怎么配才能让反扭力矩互相抵消。

做法：
  1. 把 robot.urdf 复制成 robot_rot_test.urdf，话题加 /tune 前缀，
     世界名改成 tune（避免和你开着的 Gazebo 窗口串话），只改 turningDirection
  2. 机器人吊到 20 m 高空，用【悬停推力】驱动 —— 净升力≈0，
     所以机器人不受上下加速干扰，只剩扭矩在起作用
  3. 高频采样姿态，拟合【偏航角加速度】——它正比于四桨净反扭力矩
  4. 对比两种分配方案，跑完自动清理

用法：  python3 rot_check.py
"""
import math
import os
import re
import signal
import subprocess
import time
from pathlib import Path

from gz.transport13 import Node
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.model_pb2 import Model
from gz.msgs10.actuators_pb2 import Actuators

HERE = Path(__file__).resolve().parent
# 仓库结构：本脚本在 03-tools/diag/ 下，模型文件在 <repo>/01-simulation/world/
_SIM = HERE.parent.parent / "01-simulation" / "world"
if not (HERE / "robot.urdf").exists() and (_SIM / "robot.urdf").exists():
    HERE = _SIM
SRC_U = HERE / "robot.urdf"
SRC_W = HERE / "robot_world.sdf"
TU = HERE / "robot_rot_test.urdf"
TW = HERE / "robot_rot_test_world.sdf"

T_STATE = "/tune/joint_state"
T_MOTOR = "/tune/cmd/motor_speed"
T_POSE = "/world/tune/pose/info"

G, MASS, KT = 9.81, 0.67986, 4.325e-7
HOVER = math.sqrt(MASS * G / (4 * KT))

BLK = re.compile(
    r'<plugin filename="gz-sim-multicopter-motor-model-system".*?</plugin>', re.S)

# (名称, [joint3, joint4, joint5, joint6] 的 turningDirection)
#   joint3=前左  joint4=前右  joint5=后左  joint6=后右
#   轴分别是  +Z, -Z, -Z, +Z   ← 注意本身就是镜像的
CONFIGS = [
    ("A 对角配对 ccw/cw/cw/ccw", ["ccw", "cw", "cw", "ccw"]),
    ("B 四桨同值 全 ccw",        ["ccw", "ccw", "ccw", "ccw"]),
    ("C 另一种对角 cw/ccw/ccw/cw", ["cw", "ccw", "ccw", "cw"]),
]


def make_files(tds):
    u = SRC_U.read_text()
    u = u.replace("<topic>/joint_state</topic>", f"<topic>{T_STATE}</topic>")
    u = u.replace("<commandSubTopic>/cmd/motor_speed</commandSubTopic>",
                  f"<commandSubTopic>{T_MOTOR}</commandSubTopic>")
    it = iter(tds)

    def one(m):
        return re.sub(r"<turningDirection>\w+</turningDirection>",
                      f"<turningDirection>{next(it)}</turningDirection>",
                      m.group(0), count=1)

    TU.write_text(BLK.sub(one, u))
    w = SRC_W.read_text().replace('<world name="default">', '<world name="tune">')
    w = re.sub(r"<uri>file://[^<]*robot\.urdf</uri>", f"<uri>file://{TU}</uri>", w)
    TW.write_text(w)


POSE = {}
JOINT = {}


def on_pose(msg):
    for p in msg.pose:
        if p.name != 'robot':
            continue
        q = p.orientation
        POSE['yaw'] = math.degrees(
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        POSE['pitch'] = math.degrees(
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))


def on_joint(msg):
    for j in msg.joint:
        if j.name in ('joint3', 'joint4', 'joint5', 'joint6'):
            JOINT[j.name] = j.axis1.velocity


def quad_fit(pairs):
    """拟合 v = a0 + a1*t + (a2/2)*t^2，返回 a2（加速度，单位与 v 一致 /s）"""
    sel = [p for p in pairs if p[0] > 0.03]
    if len(sel) < 12:
        return 0.0
    t0 = sel[0][0]
    ts = [p[0] - t0 for p in sel]
    vs = [p[1] for p in sel]
    n = len(ts)
    rows = [[1.0, t, t * t / 2.0] for t in ts]
    m = [[sum(rows[k][i] * rows[k][j] for k in range(n)) for j in range(3)]
         + [sum(rows[k][i] * vs[k] for k in range(n))]
         for i in range(3)]
    for c in range(3):
        piv = max(range(c, 3), key=lambda r: abs(m[r][c]))
        m[c], m[piv] = m[piv], m[c]
        for r in range(3):
            if r != c:
                f = m[r][c] / m[c][c]
                for k in range(c, 4):
                    m[r][k] -= f * m[c][k]
    return math.degrees(m[2][3] / m[2][2])


def trial(tag, tds, node, pub):
    make_files(tds)
    proc = subprocess.Popen(["gz", "sim", "-s", "-r", str(TW)], cwd=str(HERE),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            preexec_fn=os.setsid)
    try:
        POSE.clear()
        JOINT.clear()
        t1 = time.time()
        while time.time() - t1 < 30 and not POSE:
            time.sleep(0.1)
        time.sleep(1.0)

        subprocess.run(
            ["gz", "service", "-s", "/world/tune/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "2000", "--req",
             'name: "robot" position {x: 0 y: 0 z: 20.0}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        time.sleep(0.3)

        msg = Actuators()
        msg.velocity.extend([HOVER] * 4)
        rec = []
        t0 = time.time()
        while time.time() - t0 < 0.20:      # 只看到翻覆发生之前
            pub.publish(msg)
            rec.append((time.time() - t0, POSE.get('yaw', 0.0),
                        POSE.get('pitch', 0.0)))
            time.sleep(0.004)

        yaw_acc = quad_fit([(r[0], r[1]) for r in rec])
        pitch_acc = quad_fit([(r[0], r[2]) for r in rec])
        # 平均偏航角速度（0.05~窗口末）——比加速度更抗噪声
        w = [(r[0], r[1]) for r in rec if r[0] > 0.05]
        yaw_rate = ((w[-1][1] - w[0][1]) / (w[-1][0] - w[0][0])) if len(w) > 2 else 0.0
        speeds = [JOINT.get(k, 0.0) for k in
                  ('joint3', 'joint4', 'joint5', 'joint6')]
        yaws = [r[1] for r in rec]

        print(f"\n【{tag}】")
        print(f"   turningDirection : {tds}")
        print("   joint3~6 实测转速: " + ", ".join(f"{v:+.0f}" for v in speeds))
        print(f"   平均偏航角速度   : {yaw_rate:+9.3f} °/s    ← 净反扭力矩（主指标）")
        print(f"   偏航角加速度     : {yaw_acc:+9.1f} °/s²")
        print(f"   偏航摆幅(0.2s)   : {max(yaws) - min(yaws):7.3f}°")
        print(f"   俯仰角加速度     : {pitch_acc:+9.1f} °/s²  ← 对照组")
        return yaw_acc, yaw_rate, max(yaws) - min(yaws)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        time.sleep(1.0)


def main():
    subprocess.run(["pkill", "-9", "-f", "robot_rot_test"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = Node()
    node.subscribe(Pose_V, T_POSE, on_pose)
    node.subscribe(Model, T_STATE, on_joint)
    pub = node.advertise(T_MOTOR, Actuators)

    print(f"悬停推力转速 = {HOVER:.0f} rad/s（净升力≈0，机器人在 20 m 高空只受扭矩）")
    print("桨的关节轴：joint3=+Z  joint4=-Z  joint5=-Z  joint6=+Z  （本身已镜像）")

    results = []
    try:
        for tag, tds in CONFIGS:
            results.append((tag, trial(tag, tds, node, pub)))
    finally:
        for f in (TU, TW):
            if f.exists():
                f.unlink()
        subprocess.run(["pkill", "-9", "-f", "robot_rot_test"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    print("\n" + "=" * 78)
    print(f"{'方案':<28} {'平均偏航角速度':>16} {'偏航摆幅':>12}")
    print("-" * 78)
    best = min(results, key=lambda r: abs(r[1][1]))
    for tag, (acc, rate, exc) in results:
        mark = "  ⭐ 最优" if tag == best[0] else ""
        print(f"{tag:<28} {rate:>+13.3f} °/s {exc:>10.3f}°{mark}")
    print("=" * 78)
    print("\n临时文件已清理，robot.urdf 未被修改。")


if __name__ == "__main__":
    main()
