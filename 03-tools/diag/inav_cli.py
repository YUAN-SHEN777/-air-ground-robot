#!/usr/bin/env python3
"""
通用 INAV CLI 命令行工具（走串口）。

用法：
    python3 inav_cli.py "version" "mmix" "help"
    python3 inav_cli.py --port /dev/ttyACM3 "set platform_type = MULTIROTOR" "save"

说明：
    · 自动进入 CLI 模式（发送 '#'）
    · 每条命令打印完整回复
    · 如果命令导致 USB 重新枚举（如 save / exit），会容忍 I/O 错误正常退出
"""
import argparse
import sys
import time

import serial

CLI_HINT = b"Entering CLI Mode"


def drain(ser, timeout=0.8):
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        try:
            n = ser.in_waiting
        except OSError:
            return buf.decode("utf-8", "replace"), False
        if n:
            try:
                buf += ser.read(n)
            except OSError:
                return buf.decode("utf-8", "replace"), False
            end = time.time() + timeout
        else:
            time.sleep(0.02)
    return buf.decode("utf-8", "replace"), True


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default=None)
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("commands", nargs="+")
    args = ap.parse_args()

    port = args.port
    if port is None:
        import glob
        cands = sorted(glob.glob("/dev/ttyACM*"))
        if not cands:
            print("找不到 /dev/ttyACM*，飞控插好了吗？", file=sys.stderr)
            return 1
        port = cands[-1]
    print(f"# 端口 {port}\n")

    ser = serial.Serial(port, args.baud, timeout=1)
    try:
        ser.reset_input_buffer()
        ser.write(b"\n")
        ser.flush()
        time.sleep(0.4)
        pre, _ = drain(ser, 0.5)

        if CLI_HINT not in pre.encode():
            ser.write(b"#")
            ser.flush()
            time.sleep(1.2)
            pre, _ = drain(ser, 1.0)
            if CLI_HINT not in pre.encode():
                print("⚠️  未进入 CLI，原始回复：")
                print(pre.strip()[:500])
                return 1

        for cmd in args.commands:
            print("=" * 64)
            print(f"$ {cmd}")
            print("=" * 64)
            try:
                ser.write((cmd + "\n").encode())
                ser.flush()
            except OSError:
                print("（串口已断开：命令可能已生效并触发重启）")
                return 0
            time.sleep(0.9)
            out, alive = drain(ser, 0.8)
            print(out.strip())
            if not alive:
                print("\n（串口已断开：命令可能已生效并触发重启）")
                return 0
        print()
        return 0
    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
