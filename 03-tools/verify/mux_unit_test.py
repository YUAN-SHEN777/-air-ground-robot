#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""mux_unit_test.py —— 对 mux_unit.sdf 的最小 AirGroundMux 门控测试。"""

import math
import time

from gz.msgs10.actuators_pb2 import Actuators
from gz.msgs10.double_pb2 import Double
from gz.msgs10.model_pb2 import Model
from gz.transport13 import Node


class State:
    def __init__(self):
        self.prop = None
        self.joint7 = None
        self.node = Node()
        self.node.subscribe(Actuators, "/airground_0/command/prop_speed", self._prop)
        self.node.subscribe(Model, "/joint_state", self._joint)
        self.mode_pub = self.node.advertise("/model/airground_0/servo_1", Double)
        self.motor_pub = self.node.advertise("/airground_0/command/motor_speed", Actuators)

    def _prop(self, msg):
        self.prop = list(msg.velocity)

    def _joint(self, msg):
        for j in msg.joint:
            if j.name == "joint7":
                self.joint7 = j.axis1.position

    def hold(self, deg, motor, secs=1.5):
        m = Double()
        m.data = math.radians(deg)
        a = Actuators()
        a.velocity.extend([motor] * 4)
        t0 = time.time()
        while time.time() - t0 < secs:
            self.mode_pub.publish(m)
            self.motor_pub.publish(a)
            time.sleep(0.05)


def main():
    s = State()
    time.sleep(0.5)

    s.hold(45.0, 1000.0)
    print(f"[air 45°]   prop={s.prop}  joint7={s.joint7}")
    air_joint = s.joint7

    s.hold(135.0, 1000.0)
    print(f"[ground 135°] prop={s.prop}  joint7={s.joint7}")
    ground_joint = s.joint7

    s.hold(45.0, 1000.0)
    print(f"[air 45°]   prop={s.prop}  joint7={s.joint7}")

    if air_joint is not None and ground_joint is not None:
        print(f"joint7 变化：air→ground {ground_joint - air_joint:+.4f}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
