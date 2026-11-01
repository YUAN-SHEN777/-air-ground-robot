#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""topic_who.py —— 启动一次仿真，检查三个话题的发布者/订阅者数量

目的：确认 MulticopterMotorModel 到底听哪个话题。
      model.sdf 里写的是 command/motor_speed（PX4 直出），
      而 AirGroundMux 发的是 prop_speed —— 若后者无人订阅，
      那么 mux 的桨分支在飞行时就是【空转】的。
"""

import os
import subprocess
import time

os.environ.setdefault("GZ_IP", "127.0.0.1")

import tilt_yaw_px4 as T

TOPICS = [
    "/airground_0/command/motor_speed",
    "/airground_0/command/prop_speed",
    "/airground/joint_state",
]


def info(topic):
    r = subprocess.run(["gz", "topic", "-i", "-t", topic],
                       capture_output=True, text=True, timeout=15)
    txt = (r.stdout + r.stderr).strip()
    pubs = txt.count("tcp://") if "Publishers" not in txt else 0
    # 解析两段
    lines = txt.splitlines()
    n_pub = n_sub = 0
    sect = None
    for ln in lines:
        if "Publisher" in ln:
            sect = "pub"
            continue
        if "Subscriber" in ln:
            sect = "sub"
            continue
        if "tcp://" in ln:
            if sect == "pub":
                n_pub += 1
            elif sect == "sub":
                n_sub += 1
    return n_pub, n_sub, txt


def main():
    T.cleanup()
    if not T.boot():
        return 1
    time.sleep(1.0)
    print("\n话题订阅情况：")
    for t in TOPICS:
        n_pub, n_sub, raw = info(t)
        flag = "✅" if n_pub else "❌"
        print(f"  {flag} {t}")
        print(f"       发布者 {n_pub}   订阅者 {n_sub}")
        if n_pub == 0:
            print("       " + raw.replace("\n", "\n       "))
    T.cleanup()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
