#!/usr/bin/env bash
#
# gui.sh —— 给「已经在跑的」Gazebo server 单独开一个 3D 窗口
#
# 为什么这么用：
#   PX4 的 `make px4_sitl gz_xxx` **永远**以 `gz sim -r -s`（server only）启动，
#   它自己不会开窗口。`HEADLESS` / `PX4_GZ_HEADLESS` 这两个变量对 gz **都无效**
#   （前者只对 jsbsim / gazebo-classic / jmavsim 有效，后者在本版本代码里已不存在）。
#   而 `gz sim -g` 只起图形客户端，连到已存在的 server 上 ——
#   好处是物理仿真不受影响（跑测试时开/关随意），也不会出现两个 server
#   互相抢同名话题导致画面乱闪（那个坑见过）。
#
# 用法：
#   ./gui.sh                 # 贴脸视角（项目内的 gz_gui/airground_gui.config）
#   ./gui.sh --follow        # 同上，并把相机锁在车上（车开远了也看得到）
#   ./gui.sh --default       # 用 Gazebo 默认视角（~/.gz/sim/8/gui.config）
#   ./gui.sh --config FILE   # 指定任意 GUI 配置文件
#
# ⚠️ 不加 --follow 时相机是固定在原点的。车一旦开走就会“看不到车”，
#    因为 PX4 只在启动时发一次 /gui/track，GUI 后开就收不到。
#
# 前置条件：server 已在跑。检查方法：
#   ps -ef | grep "[g]z sim"        # 应看到 `gz sim ... -s .../airground.sdf`
#
# 常见问题：
#   · 窗口里是空的 / 卡住 → 检查 server 是否用的是 GZ_IP=127.0.0.1（PX4 默认是），
#     本脚本已把 GZ_IP 设成一致。
#   · 鼠标滚轮没反应 → 你的会话是 Wayland，Qt 在走 xcb 回退。
#     gz 默认就是这个模式（日志会打 "Ignoring XDG_SESSION_TYPE=wayland"）。
#     若仍无效，试：QT_QPA_PLATFORM=wayland ./gui.sh
#   · 车跑远了看不到了 → 在场景里双击车辆，或右键 → Follow
#
# 相机位姿（配置文件里的 <camera_pose>）格式：
#   x y z roll pitch yaw     （米 / 弧度）
#   项目配置用的是 -0.45 -0.35 0.28 0 0.46 0.66
#   ≈ 离车 0.64 m、高 0.28 m 的斜俯视，刚好把 0.165×0.21 m 的整车装进画面。
#   想更近就把 x/y/z 同时乘 0.7。

set -uo pipefail

WS="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEFAULT_CFG="$WS/gz_gui/airground_gui.config"
CFG="$DEFAULT_CFG"
FOLLOW=0

while [[ $# -gt 0 ]]; do
    case "$1" in
        --default)
            CFG=""
            shift
            ;;
        --follow)
            FOLLOW=1
            shift
            ;;
        --config)
            CFG="${2:-}"
            shift 2
            ;;
        -h|--help)
            sed -n '2,40p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *)
            echo "未知参数：$1（用 --help 看用法）"
            exit 1
            ;;
    esac
done

# ── 先确认 server 在跑 ──
if ! pgrep -f "gz sim.*-s" > /dev/null 2>&1; then
    echo "❌ 没找到运行中的 Gazebo server（gz sim ... -s ...）"
    echo ""
    echo "   请先启动仿真，例如："
    echo "     cd ~/PX4-Autopilot && make px4_sitl gz_airground"
    echo "   或直接："
    echo "     ./run_regression.sh smoke --keep"
    exit 1
fi

# ── 检查是否有幽灵 server（多实例会让画面在两个位姿间闪）──
N_SERVER=$(pgrep -fc "gz sim.*-s" || true)
if [[ "${N_SERVER:-1}" -gt 1 ]]; then
    echo "⚠️ 发现 ${N_SERVER} 个 Gazebo server 同时在跑！"
    echo "   多实例会各自注册同名话题，GUI 收到矛盾位姿 → 画面疯狂闪烁。"
    echo "   清理：pkill -9 -f 'gz sim'; sleep 2; rm -f /tmp/px4_lock-*"
    echo ""
    read -r -p "   仍要继续吗？[y/N] " ans
    [[ "$ans" =~ ^[Yy]$ ]] || exit 1
fi

# ── 起 GUI ──
echo "连接已运行的 server（GZ_IP=127.0.0.1）…"

# --follow：等 GUI 起来后发一条 CameraTrack，把相机锁到车上。
# 必须在 exec 之前启动，因为 exec 会替换掉当前 shell。
if [[ $FOLLOW -eq 1 ]]; then
    echo "相机跟随：已启用（跟随 airground_0）"
    (
        sleep 4
        gz topic -t /gui/track -m gz.msgs.CameraTrack \
            -p "track_mode: FOLLOW, follow_target: {name: 'airground_0'}, follow_offset: {x: -0.45, y: -0.35, z: 0.28}, follow_pgain: 1.0, track_pgain: 1.0" \
            > /dev/null 2>&1
    ) &
fi

if [[ -n "$CFG" && -f "$CFG" ]]; then
    echo "GUI 配置：$CFG"
    exec env GZ_IP=127.0.0.1 gz sim -g --gui-config "$CFG"
elif [[ -n "$CFG" ]]; then
    echo "⚠️ 配置文件不存在：$CFG，回落到默认视角"
    exec env GZ_IP=127.0.0.1 gz sim -g
else
    echo "GUI 配置：（Gazebo 默认）"
    exec env GZ_IP=127.0.0.1 gz sim -g
fi
