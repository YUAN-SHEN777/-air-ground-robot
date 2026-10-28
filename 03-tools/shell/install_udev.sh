#!/usr/bin/env bash
# 安装 STM32 / PX4 开发板的 udev 规则
#
# 用法：
#     ./install_udev.sh            # 安装并重载
#     ./install_udev.sh --check    # 只检查当前状态，不改动
#
# 需要管理员权限（脚本内部会用 sudo）。
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SRC="$HERE/50-stm32-dfu.rules"
DST="/etc/udev/rules.d/50-stm32-dfu.rules"

check_only=0
[[ "${1:-}" == "--check" ]] && check_only=1

echo "=== 当前 STM32 设备 ==="
if lsusb 2>/dev/null | grep -qi "0483:"; then
	lsusb | grep -i "0483:"
else
	echo "  （未检测到 STM32 设备；DFU 模式下应是 0483:df11）"
fi

echo
echo "=== 规则文件状态 ==="
if [[ -f "$DST" ]]; then
	echo "  已安装：$DST"
	ls -l "$DST"
else
	echo "  未安装"
fi

if (( check_only )); then
	echo
	echo "--check 模式，未做任何改动。"
	exit 0
fi

[[ -f "$SRC" ]] || { echo "找不到规则源文件：$SRC" >&2; exit 1; }

echo
echo "=== 安装规则 ==="
sudo install -m 0644 "$SRC" "$DST"
echo "  已写入 $DST"

echo
echo "=== 重载 udev ==="
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=usb
echo "  完成"

echo
echo "⚠️  请【重新插拔 USB】后规则才会作用到已连接的设备。"
echo "   验证：按住 BOOT 键插 USB，然后执行"
echo "       lsusb | grep 0483"
echo "   应看到 0483:df11 ... DFU in FS Mode"
