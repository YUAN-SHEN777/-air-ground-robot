#!/usr/bin/env bash
#
# stop_all.sh —— 一键关闭仿真并清理残留
#
# 为什么需要它：
#   PX4 SITL + Gazebo 是一组多进程（px4 + gz server + gz GUI）。用终端/工具
#   后台启动时，**关掉终端并不会杀掉子进程**，会留下「幽灵实例」：
#     · 多个 gz sim 同时跑 → 各自注册同名话题 → GUI 收到互相矛盾的位姿
#       → 机器人在多个位置之间疯狂闪烁
#     · /tmp/px4_lock-* 残留 → 下一次启动直接退出
#     · **幽灵世界话题**（最隐蔽）：残留的 px4 会订阅 /world/xxx/clock，
#       而 gz topic -l 会把「只有订阅者、没有发布者」的话题也列出来。
#       于是新启动的 PX4 看到它 → 误判「gazebo already running」→ 不再拉起
#       server → 永远卡在 `INFO [init] Waiting for Gazebo world...`。
#       注意：这种情况跟 HEADLESS 无关，加不加都会卡。
#
# ⚠️ 顺序很重要：**必须先杀进程，再删锁**。
#    反过来的话，进程退出时还会再写一次锁文件。
#
# 用法：
#   ./stop_all.sh              # 关 Gazebo + 飞控 + 清锁
#   ./stop_all.sh --gui-only   # 只关 GUI，保留仿真
#   ./stop_all.sh --quiet      # 静默（只输出最终结果）

set -uo pipefail

GUI_ONLY=0
QUIET=0
for a in "$@"; do
    case "$a" in
        --gui-only) GUI_ONLY=1 ;;
        --quiet|-q) QUIET=1 ;;
        -h|--help)
            sed -n '2,22p' "${BASH_SOURCE[0]}" | sed 's/^# \{0,1\}//'
            exit 0
            ;;
        *) echo "未知参数：$a（用 --help 看用法）"; exit 1 ;;
    esac
done

say() { [[ $QUIET -eq 1 ]] || echo "$@"; }
step() { [[ $QUIET -eq 1 ]] || echo ""; say "$@"; }

# 杀匹配的进程；返回 0 表示这次真的杀掉了东西
kill_pat() {
    local pat="$1" label="$2"
    if pgrep -f "$pat" > /dev/null 2>&1; then
        pkill -9 -f "$pat" 2>/dev/null
        say "   ✓ 已终止 $label"
        return 0
    fi
    say "   · 没有在跑的 $label"
    return 1
}

if [[ $QUIET -eq 0 ]]; then
    echo "=========================================================="
    echo " 关闭仿真并清理残留$([[ $GUI_ONLY -eq 1 ]] && echo "（仅 GUI）")"
    echo "=========================================================="
fi

# ── ① 只关 GUI ──
step "① 关闭 Gazebo GUI"
kill_pat "gz sim -g" "Gazebo GUI"

if [[ $GUI_ONLY -eq 1 ]]; then
    step "   （--gui-only：物理仿真保留运行）"
    exit 0
fi

# ── ② 关飞控 ──
# ★ 要连 make/cmake 的包装进程一起杀：
#     /bin/sh -c cd .../px4_sitl_default/src/modules/simulation/gz
#     /usr/bin/cmake -E env PX4_SIM_MODEL=...
#   只杀 bin/px4 的话，包装进程可能把子进程重新拉起来。
step "② 关闭飞控（px4）"
kill_pat "build/px4_sitl_default/bin/px4" "px4 主进程"
kill_pat "cmake -E env PX4_SIM_MODEL" "px4 包装进程（cmake）"
kill_pat "px4_sitl_default/src/modules/simulation/gz" "px4 包装进程（sh）"

# ── ③ 关 Gazebo server ──
step "③ 关闭 Gazebo server"
kill_pat "gz sim" "Gazebo server"

# ── ④ 等进程真的退出（最多 5 s）──
step "④ 等待进程退出"
waited=0
for i in 1 2 3 4 5; do
    sleep 1
    waited=$i
    n=$(pgrep -fc "gz sim" 2>/dev/null || true)
    m=$(pgrep -fc "build/px4_sitl_default/bin/px4" 2>/dev/null || true)
    [[ "${n:-0}" -eq 0 && "${m:-0}" -eq 0 ]] && break
done
say "   ✓ 已等待 ${waited}s"

# ── ⑤ 删锁文件（必须在杀进程之后）──
step "⑤ 删除锁文件"
if compgen -G "/tmp/px4_lock-*" > /dev/null; then
    rm -f /tmp/px4_lock-*
    say "   ✓ 已删除 /tmp/px4_lock-*"
else
    say "   · 没有锁文件"
fi

# ── ⑥ 确认 ──
left_gz=$(pgrep -fc "gz sim" 2>/dev/null || true)
left_px4=$(pgrep -fc "build/px4_sitl_default/bin/px4" 2>/dev/null || true)
left_lock=$(compgen -G "/tmp/px4_lock-*" 2>/dev/null | wc -l)

step "⑥ 确认"
say "   Gazebo 进程 : ${left_gz:-0}"
say "   px4   进程  : ${left_px4:-0}"
say "   锁文件      : ${left_lock}"

# 幽灵世界话题检查：只要还有进程订阅 /world/*/clock，新启动就会卡住。
ghost_worlds=$(timeout 8 gz topic -l 2>/dev/null \
    | sed -n 's|^/world/\([^/]*\)/clock$|\1|p' | sort -u | tr '\n' ' ')
ghost_count=0
[[ -n "$ghost_worlds" ]] && ghost_count=$(echo "$ghost_worlds" | wc -w)
if [[ "$ghost_count" -gt 0 ]]; then
    say "   ⚠️ 幽灵世界话题  : $ghost_worlds"
    say "      还有进程在订阅 /world/*/clock。若不处理，新 PX4 会误判"
    say "      「gazebo already running」并卡在 Waiting for Gazebo world。"
else
    say "   幽灵世界话题: 无"
fi

if [[ "${left_gz:-0}" -eq 0 && "${left_px4:-0}" -eq 0 && "${left_lock}" -eq 0 \
      && "$ghost_count" -eq 0 ]]; then
    [[ $QUIET -eq 1 ]] || echo ""
    say " ✅ 环境已清空，可以重新启动仿真"
    exit 0
else
    [[ $QUIET -eq 1 ]] || echo ""
    say " ⚠️ 仍有残留。先重跑本脚本；仍不行就手动："
    say "    pkill -9 -f 'px4_sitl_default/bin/px4'; pkill -9 -f 'gz sim'; sleep 2"
    say "    rm -f /tmp/px4_lock-*"
    say "    gz topic -l | grep '/world/.*/clock'   # 确认幽灵话题已消失"
    exit 1
fi
