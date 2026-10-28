#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
================================================================================
 关节实时监视器 —— 显示所有关节的角度与速度
================================================================================
 用法：
     python3 view_joints.py                      # 每 0.5 秒刷新一次
     python3 view_joints.py 0.1                  # 自定义刷新间隔（秒）
     python3 view_joints.py 0.5 /airground/joint_state    # 指定话题
     Ctrl+C 退出

 ⚠️ 两种模式的话题名和关节名【都不一样】，本脚本自动适配：

   模式        世界名      话题                    舵机关节名
   ────────────────────────────────────────────────────────────
   独立 Gazebo  default    /joint_state            joint1 / joint2
   PX4 仿真     airground  /airground/joint_state  servo_0 / servo_1
================================================================================
"""

import math
import sys
import time

from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model

# 候选话题（按顺序尝试；也支持命令行覆盖）
CANDIDATE_TOPICS = ['/airground/joint_state', '/joint_state']

# 关节名 → (用途标签, 顺序)
LABEL = {
    # —— 独立 Gazebo 模式（robot.urdf 原名）——
    'joint1':  ('舵机-左臂', 1),
    'joint2':  ('舵机-右臂', 2),
    'joint3':  ('螺旋桨-1', 3),
    'joint4':  ('螺旋桨-2', 4),
    'joint5':  ('螺旋桨-3', 5),
    'joint6':  ('螺旋桨-4', 6),
    'joint7':  ('轮子-5', 7),
    'joint8':  ('轮子-6', 8),
    'joint9':  ('轮子-7', 9),
    'joint10': ('轮子-8', 10),
    # —— PX4 模式（build_px4_model.py 改名后）——
    'servo_0': ('舵机-左臂', 1),
    'servo_1': ('舵机-右臂', 2),
}

# 每个关节的数据: name -> (position, velocity, limit_upper, limit_velocity)
DATA = {}
# 哪些话题收到过数据
SEEN_TOPICS = set()


def make_handler(topic):
    def on_joint(msg):
        SEEN_TOPICS.add(topic)
        for j in msg.joint:
            DATA[j.name] = (
                j.axis1.position,
                j.axis1.velocity,
                j.axis1.limit_upper,
                j.axis1.limit_velocity,
            )
    return on_joint


def fmt_deg(rad):
    if not math.isfinite(rad):
        return '—'
    return f'{math.degrees(rad):.1f}'


def render():
    out = ['\033[2J\033[H']            # 清屏 + 光标归位
    src = ', '.join(sorted(SEEN_TOPICS)) or '(等待数据...)'
    out.append(f' 数据源: {src}')
    out.append(' 关节          用途        角度(°)    角速度(rad/s)  角度上限')
    out.append('─' * 68)

    # 先按已知顺序列出，未知名字追加在后面
    known = sorted(DATA.keys(), key=lambda n: LABEL.get(n, ('', 99))[1])
    for k in known:
        p, v, up, vl = DATA[k]
        label = LABEL.get(k, ('?', 99))[0]
        running = '▶' if abs(v) > 0.005 else ' '
        out.append(
            f' {running}{k:<12} {label:<11}'
            f'{math.degrees(p):>9.2f}  {v:>12.3f}   {fmt_deg(up):>8}'
        )

    # 两臂差值（只有两个舵机都在时才算）
    for a, b in (('servo_0', 'servo_1'), ('joint1', 'joint2')):
        if a in DATA and b in DATA:
            d = math.degrees(DATA[a][0] - DATA[b][0])
            out.append(f'  两臂差: {d:+.3f}°')
            break

    out.append('─' * 68)
    out.append(' Gazebo 内部单位是【弧度】；▶ 表示该关节正在运动')
    out.append(' 舵机角度上限 52°；轮子/螺旋桨是连续关节（无上限）')
    print('\n'.join(out), flush=True)


def main():
    args = list(sys.argv[1:])
    interval = 0.5
    if args and not args[0].startswith('/'):
        interval = float(args.pop(0))
    topics = args if args else CANDIDATE_TOPICS

    node = Node()
    for t in topics:
        node.subscribe(Model, t, make_handler(t))
    print(f'正在等待 {topics} ...')

    t0 = time.time()
    while not DATA and time.time() - t0 < 6.0:
        time.sleep(0.1)

    if not DATA:
        print()
        print('❌ 收不到数据，请检查：')
        print('   1. Gazebo 是否在运行？   ps -ef | grep "[g]z sim"')
        print('   2. 仿真是否在【暂停】？  看看 GUI 上的 ▶️ 有没有亮')
        print('   3. 话题是否存在？      gz topic -l | grep joint')
        print('   4. 话题名对不对？')
        print('        独立模式 = /joint_state')
        print('        PX4 模式 = /airground/joint_state')
        print('      （也可直接指定：python3 view_joints.py 0.5 /airground/joint_state）')
        return

    try:
        while True:
            render()
            time.sleep(interval)
    except KeyboardInterrupt:
        print('\n已退出。')


if __name__ == '__main__':
    main()
