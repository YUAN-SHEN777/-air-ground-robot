#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
摆臂倾角 ↔ 偏航权限 对照实验
=============================
目的：验证"摆臂往下折 = 四桨外张倾斜"能否提升偏航控制权限。

做法（沿用 rot_check.py 已验证的方法）：
  1. 把 joint1/joint2 由 revolute 改成 fixed，安装角设为 θ（左臂 +θ，右臂 −θ），
     这样四个螺旋桨的推力轴都朝外倾斜 θ 度
  2. 话题换成 /tilt 前缀，世界名换成 tilt（不和你开着的 Gazebo 串话）
  3. 机器人吊到 20 m 高空，用【本角度对应的悬停推力】驱动 —— 净升力≈0
  4. 发一路【纯偏航差动】指令：joint4/joint5 加 δ，joint3/joint6 减 δ，
     前后/左右推力总和都不变，所以只激发偏航
  5. 采样姿态 0.25 s，拟合偏航角速度 → 正比于该倾角下的偏航力矩

桨的关节轴：joint3=+Z  joint4=-Z  joint5=-Z  joint6=+Z
turningDirection 全部 ccw  →  世界系下 joint3/joint6 逆时针、joint4/joint5 顺时针
因此 +Z 偏航指令 = 加速 joint4/joint5，减速 joint3/joint6

用法：  python3 tilt_check.py
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
SRC_U = HERE / "robot.urdf"
SRC_W = HERE / "robot_world.sdf"
TU = HERE / "robot_tilt_test.urdf"
TW = HERE / "robot_tilt_test_world.sdf"

T_STATE = "/tilt/joint_state"
T_MOTOR = "/tilt/cmd/motor_speed"
T_POSE = "/world/tilt/pose/info"

G, MASS, KT = 9.81, 0.67986, 4.325e-7
DELTA_FRAC = 0.08          # 差动幅度 = 悬停转速的 8%
WINDOW = 0.25              # 采样窗口 s

ANGLES = [0, 5, 10, 15, 20, 25]


def hover_speed(deg):
    """该倾角下刚好抵消重力的转速（垂直分量 = cosθ）"""
    return math.sqrt(MASS * G / (4 * KT * math.cos(math.radians(deg))))


def patch_urdf(deg):
    u = SRC_U.read_text()
    u = u.replace("<topic>/joint_state</topic>", f"<topic>{T_STATE}</topic>")
    u = u.replace("<commandSubTopic>/cmd/motor_speed</commandSubTopic>",
                  f"<commandSubTopic>{T_MOTOR}</commandSubTopic>")

    # joint1 绕 +X 转 +θ；joint2 绕 -X 转 +θ（等价于绕 +X 转 -θ）
    for jname, ang in (("joint1", +deg), ("joint2", -deg)):
        pat = re.compile(r'<joint name="%s" type="revolute">(.*?)</joint>' % jname, re.S)

        def sub(m, ang=ang, jname=jname):
            body = m.group(1)
            body = re.sub(
                r'<origin xyz="([^"]+)" rpy="[^"]*"\s*/>',
                lambda mm: '<origin xyz="%s" rpy="%.6f 0 0"/>'
                           % (mm.group(1), math.radians(ang)),
                body, count=1)
            body = re.sub(r'\s*<axis[^/]*/>', '', body)
            body = re.sub(r'\s*<limit[^/]*/>', '', body)
            return '<joint name="%s" type="fixed">%s</joint>' % (jname, body)

        u, n = pat.subn(sub, u)
        assert n == 1, f"{jname} 替换失败"

    TU.write_text(u)

    w = SRC_W.read_text().replace('<world name="default">', '<world name="tilt">')
    w = re.sub(r"<uri>file://[^<]*robot\.urdf</uri>", f"<uri>file://{TU}</uri>", w)
    TW.write_text(w)


POSE, JOINT = {}, {}


def on_pose(msg):
    for p in msg.pose:
        if p.name != 'robot':
            continue
        q = p.orientation
        POSE['yaw'] = math.degrees(
            math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)))
        POSE['pitch'] = math.degrees(
            math.asin(max(-1.0, min(1.0, 2 * (q.w * q.y - q.z * q.x)))))
        POSE['roll'] = math.degrees(
            math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y)))


def on_joint(msg):
    for j in msg.joint:
        if j.name in ('joint3', 'joint4', 'joint5', 'joint6'):
            JOINT[j.name] = j.axis1.velocity


def linfit(pairs):
    """最小二乘拟合 v = a0 + a1*t，返回 a1（斜率，单位 /s）"""
    if len(pairs) < 3:
        return 0.0
    n = len(pairs)
    st = sum(p[0] for p in pairs); sv = sum(p[1] for p in pairs)
    stt = sum(p[0] * p[0] for p in pairs); stv = sum(p[0] * p[1] for p in pairs)
    den = n * stt - st * st
    return 0.0 if abs(den) < 1e-12 else (n * stv - st * sv) / den


def trial(deg, node, pub):
    patch_urdf(deg)
    w0 = hover_speed(deg)
    delta = DELTA_FRAC * w0
    cmd = [w0 - delta, w0 + delta, w0 + delta, w0 - delta]   # [j3, j4, j5, j6]

    proc = subprocess.Popen(["gz", "sim", "-s", "-r", str(TW)], cwd=str(HERE),
                            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
                            preexec_fn=os.setsid)
    try:
        POSE.clear(); JOINT.clear()
        t1 = time.time()
        while time.time() - t1 < 30 and not POSE:
            time.sleep(0.1)
        time.sleep(1.0)

        subprocess.run(
            ["gz", "service", "-s", "/world/tilt/set_pose",
             "--reqtype", "gz.msgs.Pose", "--reptype", "gz.msgs.Boolean",
             "--timeout", "2000", "--req",
             'name: "robot" position {x: 0 y: 0 z: 20.0}'],
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, timeout=10)
        time.sleep(0.3)

        msg = Actuators()
        msg.velocity.extend(cmd)
        rec = []
        t0 = time.time()
        while time.time() - t0 < WINDOW:
            pub.publish(msg)
            rec.append((time.time() - t0, POSE.get('yaw', 0.0),
                        POSE.get('pitch', 0.0), POSE.get('roll', 0.0)))
            time.sleep(0.004)

        sel = [r for r in rec if r[0] > 0.05]
        yaw_rate = linfit([(r[0], r[1]) for r in sel])   # °/s   ← 主指标
        yaws = [r[1] for r in sel]
        pitch = [r[2] for r in sel]
        roll = [r[3] for r in sel]
        speeds = [JOINT.get(k, 0.0) for k in ('joint3', 'joint4', 'joint5', 'joint6')]
        return dict(deg=deg, w0=w0, yaw_rate=yaw_rate,
                    swing=(max(yaws) - min(yaws)) if yaws else 0.0,
                    pitch_exc=(max(pitch) - min(pitch)) if pitch else 0.0,
                    roll_exc=(max(roll) - min(roll)) if roll else 0.0,
                    speeds=speeds)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        time.sleep(1.0)


def main():
    subprocess.run(["pkill", "-9", "-f", "robot_tilt_test"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = Node()
    node.subscribe(Pose_V, T_POSE, on_pose)
    node.subscribe(Model, T_STATE, on_joint)
    pub = node.advertise(T_MOTOR, Actuators)

    print("摆臂倾角 ↔ 偏航权限  实测")
    print(f"差动幅度={DELTA_FRAC*100:.0f}% 悬停转速，采样窗口={WINDOW*1000:.0f} ms，20 m 高空")
    print("预测值 = cosθ + 2.1·sinθ（几何推导）\n")

    res = []
    try:
        for d in ANGLES:
            r = trial(d, node, pub)
            res.append(r)
            sp = ", ".join(f"{v:+.0f}" for v in r['speeds'])
            print(f"【θ={d:>2}°】悬停 {r['w0']:.0f} rad/s | 实测转速 {sp}")
            print(f"          偏航角速度 {r['yaw_rate']:+8.2f} °/s | 摆幅 {r['swing']:6.3f}° "
                  f"| 俯仰扰动 {r['pitch_exc']:5.2f}° | 滚转扰动 {r['roll_exc']:5.2f}°")
    finally:
        for f in (TU, TW):
            if f.exists():
                f.unlink()
        subprocess.run(["pkill", "-9", "-f", "robot_tilt_test"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    base = next((r['yaw_rate'] for r in res if r['deg'] == 0), 0.0)
    print("\n" + "=" * 72)
    print(f"{'摆臂角':>8} {'实测偏航角速度':>16} {'实测相对值':>12} {'理论预测':>10}")
    print("-" * 72)
    for r in res:
        rel = r['yaw_rate'] / base if abs(base) > 1e-9 else float('nan')
        th = r['deg']
        pred = math.cos(math.radians(th)) + 2.1 * math.sin(math.radians(th))
        print(f"{th:>7}° {r['yaw_rate']:>+13.2f} °/s {rel:>12.3f} {pred:>10.3f}")
    print("=" * 72)
    print("\n临时文件已清理，robot.urdf 未被修改。")


if __name__ == "__main__":
    main()
