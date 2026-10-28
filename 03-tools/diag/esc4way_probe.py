#!/usr/bin/env python3
"""
直接从 MSP/4way 底层探测飞控能识别到几个 ESC。

为什么需要：
    AM32 Configurator 卡在 MSP_SET_PASSTHROUGH 时，看不出到底是
    「透传没进」还是「进了但一个 ESC 都没找到」。
    这个脚本直接发 MSP_SET_PASSTHROUGH，并读出 INAV 返回的 ESC 计数。

    INAV 侧实现（src/main/fc/fc_msp.c）：
        case MSP_SET_PASSTHROUGH:
            sbufWriteU8(dst, esc4wayInit());   // ← 这就是 ESC 数量
            *mspPostProcessFn = esc4wayProcess;
    所以返回 0 就是「一个 ESC 都没认到」。

⚠️ 执行后飞控会进入 4way 模式，MSP 不再响应。
   恢复方法：拔掉 USB 再插上（或断电重启）。

用法：
    python3 esc4way_probe.py                    # 自动找端口
    python3 esc4way_probe.py --port /dev/ttyACM2
"""
import argparse
import glob
import sys
import time

import serial

MSP_SET_PASSTHROUGH = 245
MSP_PASSTHROUGH_ESC_4WAY = 0


def msp_v1(ser, cmd, payload=b"", timeout=1.5):
    """发一个 MSP v1 请求，返回 (完整负载, 校验值)。"""
    body = bytes([len(payload), cmd]) + payload
    cs = 0
    for b in body:
        cs ^= b
    ser.reset_input_buffer()
    ser.write(b"$M<" + body + bytes([cs]))
    ser.flush()

    end = time.time() + timeout
    buf = b""
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            end = time.time() + timeout
            i = buf.find(b"$M>")
            if i >= 0 and len(buf) >= i + 5:
                ln = buf[i + 3]
                if len(buf) >= i + 5 + ln + 1:
                    return buf[i + 5:i + 5 + ln], buf[i + 4 + ln]
        time.sleep(0.01)
    return None, None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    args = ap.parse_args()

    port = args.port
    if port is None:
        cands = sorted(glob.glob("/dev/ttyACM*"))
        if not cands:
            print("找不到 /dev/ttyACM*", file=sys.stderr)
            return 1
        port = cands[-1]

    print(f"端口: {port}\n")

    with serial.Serial(port, 115200, timeout=1) as ser:
        # 先确认基本 MSP 通
        p, _ = msp_v1(ser, 1, timeout=1.0)   # MSP_API_VERSION
        print(f"MSP_API_VERSION: {'OK ' + p.hex(' ') if p else '无响应'}")

        if p is None:
            print("\n❌ MSP 本身就不通。先确认飞控在 MSP 模式（不是 CLI / 4way）。")
            return 1

        print("\n发送 MSP_SET_PASSTHROUGH (mode=ESC_4WAY)...")
        p, _ = msp_v1(ser, MSP_SET_PASSTHROUGH,
                      bytes([MSP_PASSTHROUGH_ESC_4WAY, 0, 0]), timeout=6.0)

        if p is None:
            print("❌ 无响应 —— 透传命令没有被处理")
            return 1

        count = p[0] if len(p) >= 1 else -1
        print(f"回复: {p.hex(' ')}")
        print()
        print("=" * 50)
        if count > 0:
            print(f"✅ 找到 {count} 个 ESC —— 电调已通电且信号线连通")
            print("   现在可以回到 AM32 Configurator 正常配置了。")
        elif count == 0:
            print("❌ 找到 0 个 ESC")
            print("   常见原因：")
            print("     1) 四合一电调【没接电池】—— 它的 MCU 靠 VBAT 供电")
            print("     2) 电调信号线没接好 / 接错脚")
            print("     3) 电调未上电或已损坏")
        else:
            print(f"⚠️  异常回复: {count}")
        print("=" * 50)
        print("\n⚠️  飞控现在处于 4way 模式，MSP 不再响应。")
        print("    请【拔掉 USB 再插上】恢复正常。")
        return 0


if __name__ == "__main__":
    sys.exit(main())
