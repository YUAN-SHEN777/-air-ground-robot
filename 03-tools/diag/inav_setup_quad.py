#!/usr/bin/env python3
"""
通过 INAV CLI（串口）把飞控配置成四旋翼。

为什么需要：
    全新刷入的 INAV 没有配置机型，MSP_MOTOR 返回 MOTOR COUNT = 0。
    AM32 Configurator 的四路透传依赖「电机引脚」来定位 ESC，
    没有电机 → 没有 ESC 设备 → MSP_SET_PASSTHROUGH 卡死。

用法：
    python3 inav_setup_quad.py                # 只查询，不改动
    python3 inav_setup_quad.py --apply        # 查询后设为 QUAD_X 并保存

    python3 inav_setup_quad.py --port /dev/ttyACM1 --apply
"""
import argparse
import sys
import time

import serial

PROMPT = b"# "
CLI_HINT = b"Entering CLI Mode"


def drain(ser, timeout=1.0):
    """读到串口安静为止，返回累积文本。"""
    buf = b""
    end = time.time() + timeout
    while time.time() < end:
        n = ser.in_waiting
        if n:
            buf += ser.read(n)
            end = time.time() + timeout
        else:
            time.sleep(0.02)
    return buf.decode("utf-8", "replace")


def cli_cmd(ser, cmd, wait=1.0):
    ser.write((cmd + "\n").encode())
    ser.flush()
    time.sleep(wait)
    return drain(ser, timeout=0.6)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", default="/dev/ttyACM1")
    ap.add_argument("--baud", type=int, default=115200)
    ap.add_argument("--apply", action="store_true",
                    help="真正写入设置并 save（不指定则只查询）")
    args = ap.parse_args()

    print(f"打开 {args.port} @ {args.baud}")
    try:
        ser = serial.Serial(args.port, args.baud, timeout=1)
    except Exception as e:
        print(f"打开失败：{e}", file=sys.stderr)
        print("请确认：飞控已插 USB、端口名正确、没有别的程序(Chrome/配置器)占用",
              file=sys.stderr)
        return 1

    try:
        ser.reset_input_buffer()

        # 先发个换行，看是不是已经在 CLI 里
        ser.write(b"\n")
        ser.flush()
        time.sleep(0.4)
        pre = drain(ser, 0.5)

        if CLI_HINT not in pre.encode():
            # 不在 CLI，用 '#' 进入
            print("进入 CLI 模式...")
            ser.write(b"#")
            ser.flush()
            time.sleep(1.0)
            pre = drain(ser, 1.0)
            if CLI_HINT not in pre.encode():
                print("⚠️  似乎没有进入 CLI。原始回复：")
                print(pre.strip()[:400])
                print("\n提示：若飞控正在被 Chrome / AM32 配置器占用，先断开。")
                return 1

        print("已在 CLI 模式\n")

        print("=" * 60)
        print("【version】")
        print(cli_cmd(ser, "version").strip())

        print("=" * 60)
        print("【get platform_type】")
        pt = cli_cmd(ser, "get platform_type")
        print(pt.strip())

        print("=" * 60)
        print("【get model_predefined】")
        mp = cli_cmd(ser, "get model_predefined")
        print(mp.strip())

        if not args.apply:
            print("\n未指定 --apply，未做任何改动。")
            cli_cmd(ser, "exit", 0.5)
            return 0

        print("=" * 60)
        print("【设置 MULTIROTOR + QUAD_X】")
        print(cli_cmd(ser, "set platform_type = MULTIROTOR").strip())
        print(cli_cmd(ser, "set model_predefined = QUAD_X").strip())

        # 回读确认
        print("=" * 60)
        print("【回读】")
        print(cli_cmd(ser, "get platform_type").strip())
        print(cli_cmd(ser, "get model_predefined").strip())

        print("=" * 60)
        print("【save】（飞控会重启，串口会断开）")
        ser.write(b"save\n")
        ser.flush()
        time.sleep(2.0)
        print(drain(ser, 1.0).strip() or "（已发送 save，飞控重启中）")
        return 0

    finally:
        try:
            ser.close()
        except Exception:
            pass


if __name__ == "__main__":
    sys.exit(main())
