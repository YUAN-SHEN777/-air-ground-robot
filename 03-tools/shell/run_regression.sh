#!/usr/bin/env bash
#
# run_regression.sh —— 一键回归：起仿真 → 冒烟 → 关仿真
#
# 用法：
#   ./run_regression.sh              # 跑 smoke
#   ./run_regression.sh repeat       # 跑任意 verify.py 子命令
#   ./run_regression.sh smoke --keep # 跑完不关仿真（方便接着手工看）
#
# 环境变量：
#   PX4_DIR    PX4-Autopilot 路径（默认 ~/PX4-Autopilot）
#
# 为什么需要它：
#   PX4 SITL 有两个「幽灵」行为会让手工测试白费功夫 ——
#   ① 工具后台启动的 gz sim 在终端关闭后仍然存活，多实例会互相抢同名话题；
#   ② /tmp/px4_lock-0 残留会让新实例直接退出。
#   本脚本在起仿真前一律清理，并且在退出时（含 Ctrl+C）也会清理。

set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PX4="${PX4_DIR:-$HOME/PX4-Autopilot}"
LOG="/tmp/px4_regression.log"

SUBCMD="${1:-smoke}"
KEEP=0
for a in "$@"; do
    [[ "$a" == "--keep" ]] && KEEP=1
done

cleanup() {
    pkill -9 -f "build/px4_sitl_default/bin/px4" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
    sleep 2
    rm -f /tmp/px4_lock-*
}

on_exit() {
    if [[ $KEEP -eq 0 ]]; then
        echo ""
        echo "⑤ 清理仿真进程"
        cleanup
    else
        echo ""
        echo "⑤ --keep：仿真保留运行（手工结束时记得 pkill -9 -f 'gz sim'）"
    fi
}
trap on_exit EXIT INT TERM

echo "=========================================================="
echo " 地面驱动回归  ·  子命令 = $SUBCMD"
echo "=========================================================="

echo ""
echo "① 清理残留进程与锁文件"
cleanup

if [[ ! -d "$PX4" ]]; then
    echo "❌ 找不到 PX4 目录：$PX4（可用 PX4_DIR=... 指定）"
    exit 1
fi

echo ""
echo "② 启动 PX4 SITL + Gazebo（server only，无 GUI）"
echo "   日志：$LOG"
# 注意：PX4 的 gz 固定以 `gz sim -r -s` 启动，HEADLESS 变量对 gz 无效。
#       这里不再传它，避免造成「加了才有用」的误解。想看画面用 ./gui.sh。
( cd "$PX4" && make px4_sitl gz_airground > "$LOG" 2>&1 ) &

echo ""
echo -n "③ 等待就绪"
for i in $(seq 1 60); do
    if grep -q "Ready for takeoff" "$LOG" 2>/dev/null; then
        echo "  ✓ 耗时 ${i}s"
        break
    fi
    echo -n "."
    sleep 1
done

if ! grep -q "Ready for takeoff" "$LOG" 2>/dev/null; then
    echo ""
    echo "❌ 60 s 内未就绪。日志尾部："
    tail -25 "$LOG"
    exit 1
fi

# 顺带把「不该出现」的东西报出来
if grep -q "Disarmed by landing" "$LOG"; then
    echo "   ⚠️ 日志里出现 'Disarmed by landing' —— COM_DISARM_LAND 可能没生效，"
    echo "      这会导致后续数据全部变成 [500,500,500,500]。见文档 §14.6③"
fi
if grep -q "ground_drive: registration FAILED" "$LOG"; then
    echo "   ❌ ground_drive 注册失败"
    tail -15 "$LOG"
    exit 1
fi

echo ""
echo "④ 运行 verify.py $SUBCMD"
cd "$WS" || exit 1
# shellcheck disable=SC2086
python3 verify.py $SUBCMD
RC=$?

echo ""
echo "=========================================================="
if [[ $RC -eq 0 ]]; then
    echo " ✅ 回归完成"
else
    echo " ❌ 回归返回码 $RC"
fi
echo "=========================================================="
exit $RC
