#!/usr/bin/env python3
"""
摆臂舵机 PID 参数扫描
=====================
目的：找出让「阶跃指令 → 到位」最快的 PID 参数。

做法：
  1. 把 robot.urdf / robot_world.sdf 复制成 robot_tune.urdf / robot_tune_world.sdf
     （话题统一加 /tune/ 前缀、世界名改成 tune，避免和别的仿真进程串话）
     → 你的原文件全程不被触碰
  2. 每个参数组合：改写增益 → 启动仿真 → 发阶跃指令 → 记录角度响应
  3. 输出对比表：超调 / 到位时间(±0.2°) / 残余误差

用法：  cd "/home/yuan/robot air-ground/robot_urdf (4)" && python3 pid_sweep.py
"""
import math
import os
import re
import signal
import statistics
import subprocess
import time
from pathlib import Path

from gz.transport13 import Node
from gz.msgs10.model_pb2 import Model
from gz.msgs10.double_pb2 import Double

HERE = Path(__file__).resolve().parent
# 仓库结构：本脚本在 03-tools/servo/ 下，模型文件在 <repo>/01-simulation/world/
_SIM = HERE.parent.parent / "01-simulation" / "world"
if not (HERE / "robot.urdf").exists() and (_SIM / "robot.urdf").exists():
    HERE = _SIM
SRC_URDF = HERE / "robot.urdf"
SRC_WORLD = HERE / "robot_world.sdf"
TUNE_URDF = HERE / "robot_tune.urdf"
TUNE_WORLD = HERE / "robot_tune_world.sdf"

T_STATE = "/tune/joint_state"
T_ARM = "/tune/cmd/arm_angle"
T_WHEEL = "/tune/cmd/wheel_speed"

STEPS = [48.0, 10.0]   # 阶跃序列：先冲到 48°（最大负载），再压回 10°（反向检验）
STEP_T = 6.0       # 指令发完后观察多久（看超调 / 到位时间）[s]
HOLD_T = 5.0       # 之后再多录多久（专门评估"稳态抖动"）[s]
RAMP_HZ = 50.0     # 斜坡指令的发送频率 [Hz]
STARTUP_T = 25.0   # 仿真启动等待上限 [s]

# 已证实：b>0.2 会让舵机堵转（b=0.5 差 1.4° 到不了，b=1.0 完全冻死）→ 物理阻尼加不得
# 已证实：i_max 是"积分寻的极限环"的振幅天花板
#         i_max=0.30 → 抖动 0.0003°（积分被焊死）  i_max=0.50 → 抖动 0.72°（积分自由游走）
# 结论：i_max 必须锁在 0.30（用户要求"稳态无抖动"优先于静态精度）
# 本轮只扫斜坡时长，把"到位总时间"调成 ~3 秒
# (简称, p_gain, i_gain, i_max, 关节物理阻尼, 斜坡时长 s)
CONFIGS = [
    ("i.3 阶跃",      3.0, 2.0, 0.30, 0.20, 0.0),  # 参考：1.09s，抖动 0.0003°
    ("i.3 坡2s",      3.0, 2.0, 0.30, 0.20, 2.0),
    ("i.3 坡3s",      3.0, 2.0, 0.30, 0.20, 3.0),  # 目标：~3 秒走完
    ("i.3 坡3s Ki3",  3.0, 3.0, 0.30, 0.20, 3.0),  # 斜坡下加 Ki 补跟踪
    ("i.3 坡3s Kp5",  5.0, 3.0, 0.30, 0.20, 3.0),  # 加刚度补跟踪
]

BLOCK = re.compile(
    r'<plugin filename="gz-sim-joint-position-controller-system".*?</plugin>', re.S)


JOINT = re.compile(r'<joint name="joint[12]" type="revolute">.*?</joint>', re.S)


def patch_damping(text, b):
    """只改 joint1/joint2 的物理阻尼，不动其他关节。"""
    def one(m):
        return re.sub(r'<dynamics damping="[^"]*"',
                      f'<dynamics damping="{b}"', m.group(0))
    return JOINT.sub(one, text)


def patch_gains(text, kp, ki, imax):
    """只改位置控制器插件块里的增益，不动其他内容。"""
    def one(m):
        b = m.group(0)
        b = re.sub(r"<p_gain>[^<]*</p_gain>", f"<p_gain>{kp}</p_gain>", b)
        b = re.sub(r"<i_gain>[^<]*</i_gain>", f"<i_gain>{ki}</i_gain>", b)
        b = re.sub(r"<i_max>[^<]*</i_max>", f"<i_max>{imax}</i_max>", b)
        b = re.sub(r"<i_min>[^<]*</i_min>", f"<i_min>{-imax}</i_min>", b)
        return b
    return BLOCK.sub(one, text)


def make_tune_files(kp, ki, imax, bdamp):
    u = SRC_URDF.read_text()
    u = u.replace("<topic>/joint_state</topic>", f"<topic>{T_STATE}</topic>")
    u = u.replace("<topic>/cmd/arm_angle</topic>", f"<topic>{T_ARM}</topic>")
    u = u.replace("<sub_topic>/cmd/wheel_speed</sub_topic>",
                  f"<sub_topic>{T_WHEEL}</sub_topic>")
    u = u.replace("<topic>/cmd/wheel_speed</topic>", f"<topic>{T_WHEEL}</topic>")
    u = patch_gains(u, kp, ki, imax)
    u = patch_damping(u, bdamp)
    TUNE_URDF.write_text(u)

    w = SRC_WORLD.read_text()
    w = w.replace('<world name="default">', '<world name="tune">')
    w = re.sub(r"<uri>file://[^<]*robot\.urdf</uri>",
               f"<uri>file://{TUNE_URDF}</uri>", w)
    TUNE_WORLD.write_text(w)


BUF = []


def on_state(m):
    for j in m.joint:
        if j.name == "joint1":
            BUF.append(math.degrees(j.axis1.position))


def settle_time(samples, tol, target):
    """首次满足「此后一直」落在 tol 内的时间点；未达标返回 None。"""
    n = len(samples)
    for i in range(n - 1, -1, -1):
        if abs(samples[i][1] - target) > tol:
            break
    else:
        i = -1
    if i + 1 >= n:
        return None
    return samples[i + 1][0]


def analyze(samples, target):
    """把一段响应提炼成指标。

    excursion  正=超调，负=还没到位
    mean_ss    最后 HOLD_T 秒的平均值（真正的稳态）
    ripple     最后 HOLD_T 秒的峰峰值 → 就是"稳态抖动"
    """
    if not samples:
        return None
    up = target > samples[0][1]
    vals = [v for _, v in samples]
    ext = max(vals) if up else min(vals)
    t_end = samples[-1][0]
    tail = [v for ts, v in samples if ts > t_end - HOLD_T]
    return {
        "excursion": ext - target,
        "t05": settle_time(samples, 0.5, target),
        "t02": settle_time(samples, 0.2, target),
        "final": samples[-1][1] - target,
        "mean_ss": (statistics.mean(tail) - target) if tail else 0.0,
        "ripple": (max(tail) - min(tail)) if tail else 0.0,
        "tail": tail,
    }


def measure(pub, start, target, ramp, duration):
    """发指令并记录 duration 秒的角度响应。

    ramp <= 0.05 : 直接跳变（阶跃）
    ramp >  0.05 : 用 ramp 秒把指令从 start 平滑推到 target（斜坡）
    """
    BUF.clear()
    t0 = time.time()
    if ramp <= 0.05:
        pub.publish(Double(data=math.radians(target)))
    else:
        n = max(2, int(round(ramp * RAMP_HZ)))
        dt = ramp / n
        for i in range(1, n + 1):
            a = start + (target - start) * i / n
            pub.publish(Double(data=math.radians(a)))
            time.sleep(dt)
    out = []
    while time.time() - t0 < duration:
        if BUF:
            ts = time.time() - t0
            out.extend((ts, v) for v in BUF)
            BUF.clear()
        time.sleep(0.0005)
    return out


def run_one(name, kp, ki, imax, bdamp, ramp, node, pub):
    make_tune_files(kp, ki, imax, bdamp)
    proc = subprocess.Popen(
        ["gz", "sim", "-s", "-r", str(TUNE_WORLD)],
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        cwd=str(HERE), preexec_fn=os.setsid)
    try:
        BUF.clear()
        t0 = time.time()
        while time.time() - t0 < STARTUP_T and not BUF:
            time.sleep(0.1)
        if not BUF:
            return None
        time.sleep(1.5)                      # 让仿真先稳定
        steps = []
        cur = 0.0
        for k, tgt in enumerate(STEPS):
            if k:
                time.sleep(2.0)              # 步与步之间留出静止时间
            smp = measure(pub, cur, tgt, ramp, STEP_T + HOLD_T)
            m = analyze(smp, tgt)
            if m is None:
                return None
            m["target"] = tgt
            m["curve"] = smp
            steps.append(m)
            cur = smp[-1][1]
        return {"name": name, "kp": kp, "ki": ki, "imax": imax,
                "bdamp": bdamp, "ramp": ramp, "steps": steps}
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except Exception:
            pass
        try:
            proc.wait(timeout=5)
        except Exception:
            pass
        subprocess.run(["pkill", "-9", "-f", "robot_tune_world"],
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(0.8)


def main():
    subprocess.run(["pkill", "-9", "-f", "robot_tune_world"],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    node = Node()
    node.subscribe(Model, T_STATE, on_state)
    pub = node.advertise(T_ARM, Double)

    out = []
    for name, kp, ki, imax, bdamp, ramp in CONFIGS:
        print(f"▶ 测试 {name:14s} Kp={kp} Ki={ki} i_max={imax} b={bdamp} "
              f"斜坡={ramp}s ...", flush=True)
        r = run_one(name, kp, ki, imax, bdamp, ramp, node, pub)
        if r is None:
            print("   ⚠ 无数据，跳过")
            continue
        out.append(r)
        for m in r["steps"]:
            t02 = f"{m['t02']:.2f}s" if m["t02"] else "未达标"
            print(f"   阶跃→{m['target']:5.0f}°  超调={m['excursion']:+7.3f}°  "
                  f"到±0.2°={t02:>8s}  稳态偏={m['mean_ss']:+7.4f}°  "
                  f"抖动={m['ripple']:.5f}°")

    print("\n" + "=" * 108)
    print(f"阶跃序列 {STEPS}，每步记录 {STEP_T + HOLD_T:.0f} s"
          f"（最后 {HOLD_T:.0f} s 静止，专门量抖动）      τ_理论 = Kp/Ki")
    print("=" * 108)
    print(f"{'参数':>13} | {'Kp':>4} {'Ki':>4} {'imax':>5} {'b':>4} {'τ':>5} | "
          f"{'阶跃到':>6} | {'超调°':>8} | {'到±0.2°':>8} | "
          f"{'稳态偏差°':>10} | {'稳态抖动°':>10}")
    print("-" * 112)
    for r in out:
        for j, m in enumerate(r["steps"]):
            tau = f"{r['kp'] / r['ki']:.2f}" if j == 0 else ""
            kp = f"{r['kp']:.1f}" if j == 0 else ""
            ki = f"{r['ki']:.1f}" if j == 0 else ""
            ix = f"{r['imax']:.2f}" if j == 0 else ""
            bd = f"{r['bdamp']:.2f}" if j == 0 else ""
            t02 = f"{m['t02']:.2f}s" if m["t02"] else "未达标"
            print(f"{(r['name'] if j == 0 else ''):>13} | {kp:>4} {ki:>4} {ix:>5} "
                  f"{bd:>4} {tau:>5} | {m['target']:>6.0f} | {m['excursion']:>+8.3f} | "
                  f"{t02:>8} | {m['mean_ss']:>+10.4f} | {m['ripple']:>10.5f}")
        print("-" * 112)

    # 画最快那组的响应曲线（ASCII）
    if out:
        def key(r):
            settled = all(m["t02"] is not None for m in r["steps"])
            return (0 if settled else 1, abs(r["steps"][-1]["excursion"]))
        best = min(out, key=key)
        for m in best["steps"]:
            tgt = m["target"]
            print(f"\n最快那组的响应曲线：{best['name']}   阶跃 → {tgt:.0f}°")
            print("-" * 62)
            s = max(1, len(m["curve"]) // 40)
            scale = 40.0 / tgt if tgt else 40.0
            for i in range(0, len(m["curve"]), s):
                ts, v = m["curve"][i]
                n = int(min(max(v * scale, 0), 44))
                print(f"{ts:5.2f}s {v:7.3f}° |{'#' * n}")

    for f in (TUNE_URDF, TUNE_WORLD):
        try:
            f.unlink()
        except OSError:
            pass
    print("\n临时文件已清理，robot.urdf / robot_world.sdf 未被修改。")


if __name__ == "__main__":
    main()
