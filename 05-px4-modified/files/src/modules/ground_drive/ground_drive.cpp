/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

/**
 * @file ground_drive.cpp
 *
 * 地面驱动（轮式）外部模式。
 *
 * 功能全貌：
 *   · 50 Hz 调度；注册外部模式 GROUND_DRIVE（NAVIGATION_STATE_EXTERNAL1+N）
 *   · 激活时发 setpoint_config{TYPE_DIRECT_ACTUATORS} → control_allocator 停发
 *     → 本模块成为 actuator_motors 的唯一发布者
 *   · arming check 门控：激活时看摆臂角度；未激活时只拦「armed + 空中」
 *   · 摆臂角度限速外推器（不读真实角度，见 updateServoEstimate）
 *   · 差速运动学 → 4 路归一化指令
 *   · 补发 vehicle_thrust_setpoint{0}，喂饱 land detector（见 publishThrustZero）
 *
 * 详细设计、源码链路与 P4 排除的三个系统性陷阱：
 *   《PX4地面驱动方案.md》第 4、12、13 章。
 *
 * ⚠️ 已知限制（不要再试软件调参，详见文档第 14 章）
 *   四轮滑移转向原地搓转的力矩比是纯几何量：
 *       驱动力矩 = 4·mu·N·(W/2) = 2·mu·N·W
 *       擦阻力矩 = 4·mu·N·(L/2) = 2·mu·N·L
 *       比值 = W/L = 0.165/0.21 = 0.79     ← mu 被约掉
 *   ⇒ ω=0 是被静摩擦焊死的稳定解，结果呈双稳态（要么 0°，要么 70~150°）。
 *   实测无效的手段：转向伴速、纵向抖动、GD_YAW_GAIN 1.5→5.0、
 *   转向时 v_cmd 归零、借动量后收油 —— 成功率都在 17~40% 徘徊。
 *   唯一有效：把轮距加宽到 ≥0.30 m（W/L≥1.43）→ 8/8。
 */

#include "ground_drive.hpp"

#include <math.h>
#include <string.h>

#include <drivers/drv_hrt.h>
#include <mathlib/math/Functions.hpp>

ModuleBase::Descriptor GroundDrive::desc{task_spawn, custom_command, print_usage};

namespace
{
//! 摆臂角度换算：SIM_GZ_SV_MINA1 = 0°, MAXA1 = 52°
//!   ⇒ angle[deg] = 26 * (value + 1)，value ∈ [-1, 1]
constexpr float kArmAnglePerUnit{26.f};

//! 未注册时的注册请求重发周期
constexpr hrt_abstime kRegisterRetryInterval{1_s};

//! 激活期间 setpoint_config 的重发周期（防丢）
constexpr hrt_abstime kSetpointConfigInterval{1_s};

//! 舵机功能号（对应 airframe 里 SIM_GZ_SV_FUNC1 = 301）
constexpr float kServoFunction{301.f};

//! 消息队列一次最多消费的条数（不依赖各 msg 是否定义 ORB_QUEUE_LENGTH）
constexpr int kMaxMsgPerCycle{4};

//! 外推器单周期步长的下限/上限（秒）。
//! 用真实周期而不是硬编码 20 ms —— nav_and_controllers 队列会被其他任务挤占，
//! 周期一抖外推就偏，而这个角度直接喂 arming 门控（安全相关）。
constexpr float kServoDtMin{0.01f};
constexpr float kServoDtMax{0.10f};

//! 摇杆死区（含等比放大：dz..1 → 0..1，满量程不损失）。
//! 为什么需要：QGC 虚拟摇杆 / 实体摇杆回中后并不精确为 0。
//! 实测回中时 thr = +0.12 → v = 0.144 m/s，而轮子在前有 64:1 减速器，
//! 电机实际要转到 ~2200 rpm（桨同轴，声音很大），看上去就像“没动杆却在转了”。
//! 死区内强制归零，死区外把剩余行程放大到 0..1，满杆仍然满权限。
float applyStickDeadzone(float v, float dz)
{
	if (dz <= 0.f) {
		return v;
	}

	const float a = fabsf(v);

	if (a <= dz) {
		return 0.f;
	}

	return copysignf((a - dz) / (1.f - dz), v);
}
} // namespace

GroundDrive::GroundDrive() :
	ModuleParams(nullptr),
	ScheduledWorkItem(MODULE_NAME, px4::wq_configurations::nav_and_controllers),
	_cycle_perf(perf_alloc(PC_ELAPSED, MODULE_NAME ": cycle"))
{
	updateParams();
}

GroundDrive::~GroundDrive()
{
	perf_free(_cycle_perf);
}

bool GroundDrive::init()
{
	// 50 Hz：与文档 §4.2 一致，外推器的 dt 也按 1/50 s 算
	ScheduleOnInterval(20_ms);
	return true;
}

void GroundDrive::Run()
{
	if (should_exit()) {
		if (_reg_request_sent) {
			unregisterExternalMode();
		}

		exit_and_cleanup(desc);
		return;
	}

	perf_begin(_cycle_perf);

	if (_parameter_update_sub.updated()) {
		updateParams();
	}

	// ── ⓪ 刷新落地状态（arming check 门控要用）──
	// 必须放在 ② arming check 之前：commander 随时可能来问。
	vehicle_land_detected_s land{};

	if (_vehicle_land_detected_sub.update(&land)) {
		_landed = land.landed;
	}

	// ── ① 注册（未成功则 1 s 重试）──
	if (!_registered) {
		tryRegister();
		handleRegistrationReply();
	}

	// ── ② arming check（commander 定期询问，必须每次回复）──
	if (_arming_check_id >= 0) {
		arming_check_request_s req;

		for (int i = 0; i < kMaxMsgPerCycle; i++) {
			if (!_arming_check_req_sub.update(&req)) {
				break;
			}

			replyToArmingCheck(req.request_id);
		}
	}

	// ── ③ 摆臂角度外推（每周期）──
	updateServoEstimate();

	// ── ④ 激活 / 退出检测 ──
	vehicle_status_s vs;

	if (_vehicle_status_sub.update(&vs)) {
		_vehicle_status = vs;

		const bool active = _registered && (_mode_id >= 0) && (vs.nav_state == (uint8_t)_mode_id);

		if (active && !_mode_active) {
			activateMode();

		} else if (!active && _mode_active) {
			deactivateMode();
		}
	}

	const bool active_now = _registered && (_mode_id >= 0)
				&& (_vehicle_status.nav_state == (uint8_t)_mode_id);

	// ── 危险状态监测：摆臂在轮位，但当前不是本模式却已解锁 ──
	// 这时 control_allocator 已在驱动 actuator_motors，而 AirGroundMux 又把桨
	// 置 0、把所有输出送给了轮子 ⇒ **飞控的姿态混控直接推着车跑**，
	// 而且推力只有正值（永远不反转）、姿态环持续配平（不动摇杆也在动）。
	// 实测（2026-10-18）：操作者误点 QGC 的 Takeoff → 切 AUTO_TAKEOFF/Position
	// → 就是这个状态，车在地面上自己跑，还以为是轮子坏了。
	if ((_servo_angle_est_deg >= _param_arm_thresh.get()) && !_mode_active && active_now == false
	    && (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED)) {
		if (hrt_elapsed_time(&_last_hazard_warn) > 2_s) {
			PX4_WARN("ground_drive: 摆臂在轮位但飞控在驾驶！切回 GROUND_DRIVE 或先 Disarm");
			_last_hazard_warn = hrt_absolute_time();
		}
	}

	// ── ⑤ 激活时：周期重发 setpoint_config + 输出零 ──
	if (_mode_active) {
		const hrt_abstime now = hrt_absolute_time();

		if ((_last_sp_config_pub == 0) || (hrt_elapsed_time(&_last_sp_config_pub) > kSetpointConfigInterval)) {
			publishSetpointConfig();
		}

		checkSetpointConfigReply();

		// ── ⑥ 补齐 land detector 缺失的输入（详见 publishThrustZero 注释）──
		// 条件：本模式激活（_mode_active 已满足）+ 分配器已让位（_sp_confirmed）
		//
		// ⚠️ 千万不要加「已解锁」条件！实测教训（2026-10-17）：
		//    加了 armed 后，解锁瞬间的第一帧里 armed=true 而 valid 还是 false，
		//    land detector 抖动一下 → commander 打 "Takeoff detected" →
		//    置 _have_taken_off_since_arming=true → COM_DISARM_LAND(2 s) 立刻自动上锁。
		//    改成未解锁也发（未解锁时 landed 靠 !_armed 本来就是 true，无害），
		//    就能保证 armed 变 true 的那一刻 valid 已经是 true，不抖。
		if (_sp_confirmed) {
			publishThrustZero(now);
		}

		manualDrive(now);
	}

	perf_end(_cycle_perf);
}

// ──────────────────────────────────────────────────────────── ① 注册

void GroundDrive::tryRegister()
{
	if (_reg_request_sent && (hrt_elapsed_time(&_reg_request_time) < kRegisterRetryInterval)) {
		return;
	}

	registerExternalMode();
}

void GroundDrive::registerExternalMode()
{
	register_ext_component_request_s req{};
	req.timestamp = hrt_absolute_time();
	strncpy(req.name, kModeName, sizeof(req.name) - 1);
	req.request_id = kRequestId;
	req.px4_ros2_api_version = register_ext_component_request_s::LATEST_PX4_ROS2_API_VERSION;
	req.register_arming_check = true;            // ★ register_mode 依赖它
	req.register_mode = true;
	req.register_mode_executor = false;
	req.enable_replace_internal_mode = false;    // 不替换任何内置模式
	req.activate_mode_immediately = false;
	req.not_user_selectable = false;             // 用户可以从 QGC 切进来
	req.request_offboard_setpoints = false;

	_reg_req_pub.publish(req);

	_reg_request_sent = true;
	_reg_request_time = req.timestamp;
	_register_attempts++;

	if (_register_attempts == 1) {
		PX4_INFO("ground_drive: registration request sent (%s)", kModeName);

	} else if ((_register_attempts % 10) == 0) {
		PX4_WARN("ground_drive: still not registered (%u attempts)", (unsigned)_register_attempts);
	}
}

void GroundDrive::handleRegistrationReply()
{
	register_ext_component_reply_s rep;

	for (int i = 0; i < kMaxMsgPerCycle; i++) {
		if (!_reg_reply_sub.update(&rep)) {
			break;
		}

		if (rep.request_id != kRequestId) {
			continue;
		}

		if (!rep.success) {
			PX4_ERR("ground_drive: registration FAILED");
			return;
		}

		_arming_check_id = rep.arming_check_id;
		_mode_id = rep.mode_id;
		_registered = true;

		PX4_INFO("ground_drive: registered — mode_id=%d arming_check_id=%d",
			 (int)_mode_id, (int)_arming_check_id);
		PX4_INFO("ground_drive: enter with `commander mode ext%d` (QGC 里也能看到 %s)",
			 (int)_mode_id - (int)vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 + 1, kModeName);
		return;
	}
}

void GroundDrive::unregisterExternalMode()
{
	unregister_ext_component_s req{};
	req.timestamp = hrt_absolute_time();
	strncpy(req.name, kModeName, sizeof(req.name) - 1);
	req.arming_check_id = _arming_check_id;
	req.mode_id = _mode_id;
	req.mode_executor_id = -1;

	_unreg_pub.publish(req);

	PX4_INFO("ground_drive: unregistered");
}

// ────────────────────────────────────────────── ② arming check 门控

//! 摆臂角度门控开关（参数 GD_SERVO_GATE，默认 1 = 启用）。
//!   · 激活中         ：摆臂 >= GD_ARM_MIN_DEG 才允许 arm / 继续 run
//!   · 未激活 + armed ：仅当【已落地】才放行 → 清 can_run 位 → 拒绝空中误切
//!   · GD_SERVO_GATE=0：门控彻底失效（含 manualDrive 的输出门控点 ②）
//!     —— 仅供舵机未接线时的台架调试，必须拆桨 + 整车架空！
//!
//! 为什么用参数而不是编译期常量：
//!   舵机没接/没标定时，把 GD_ARM_MIN_DEG 降到 0 也能绕过门控，但那会连带
//!   打开 manualDrive() 的输出门控，而且改完必须记得改回去。用独立开关可以
//!   保持已标定的阈值（48°）不变，重新启用就恢复真实安全限。
bool GroundDrive::servoGateEnabled()
{
	return _param_servo_gate.get() != 0;
}

void GroundDrive::replyToArmingCheck(uint8_t request_id)
{
	arming_check_reply_s rep{};
	rep.timestamp = hrt_absolute_time();
	rep.request_id = request_id;
	rep.registration_id = (uint8_t)_arming_check_id;
	rep.health_component_index = arming_check_reply_s::HEALTH_COMPONENT_INDEX_NONE;
	rep.num_events = 0;

	// 三段式门控（文档 §4.4）：
	//   激活中  ：摆臂到位才允许 arm / 继续 run
	//   未激活  ：摆臂到位 且（未解锁 或 已落地）才允许切入
	//
	// ⚠️ 2026-10-18 修订：未激活分支也【必须】要求 servo_ok。
	//    原先只查 `!armed || _landed`，于是在摆臂还是 0° 时就能切进地面模式：
	//    此时 AirGroundMux 仍按飞行位把电机指令送给桨，轮子一动不动，
	//    操作者看到的是“进了地面模式但车不动”（实测踩过）。
	//    要求 servo_ok 后，摆臂没到位就切不进来，commander 会直接提示模式被拒。
	//
	// ⚠️ 绝不能写成 `_mode_active ? servo_ok : !armed`（旧版）：
	//    `_mode_active` 本身就是「nav_state == mode_id」，于是“能不能进去”
	//    依赖“已经在里面” —— 自指死锁。一旦有任何外部原因把模式顶掉
	//    （failsafe → Return 等），就永远回不来。
	//    详见 PX4 侧链路：externalChecks.cpp:163 置 mode_req_other →
	//    modeCheck.cpp:182 clearCanRunBits → UserModeIntention.cpp:57 canRun()==false。
	//
	// 用 landed 而不是只用 armed 的原因：
	//   armed 时“降落 → 切地面模式”是正常工作流，必须允许；
	//   只有“armed 且在空中”才应该拒绝（防空中误切）。
	//   未解锁时 landed 恒为 true（land detector 的 `!_armed ||` 分支）。
	const bool armed = (_vehicle_status.arming_state == vehicle_status_s::ARMING_STATE_ARMED);
	const bool servo_ok = (_servo_angle_est_deg >= _param_arm_thresh.get());

	if (servoGateEnabled()) {
		rep.can_arm_and_run = _mode_active ? servo_ok : (servo_ok && (!armed || _landed));

	} else {
		rep.can_arm_and_run = true;
	}

	// 地面模式不需要任何状态估算，只要摇杆
	rep.mode_req_angular_velocity        = false;
	rep.mode_req_attitude                = false;
	rep.mode_req_local_alt               = false;
	rep.mode_req_local_position          = false;
	rep.mode_req_local_position_relaxed  = false;
	rep.mode_req_global_position         = false;
	rep.mode_req_global_position_relaxed = false;
	rep.mode_req_mission                 = false;
	rep.mode_req_home_position           = false;
	rep.mode_req_prevent_arming          = false;
	rep.mode_req_manual_control          = true;

	_arming_check_reply_pub.publish(rep);
}

// ──────────────────────────────────────────────────────── ③ 模式激活

void GroundDrive::activateMode()
{
	_mode_active = true;
	_sp_confirmed = false;
	_last_sp_config_pub = 0;

	publishSetpointConfig();

	PX4_INFO("ground_drive: mode ACTIVE — requesting TYPE_DIRECT_ACTUATORS");

	// 防御性提示：正常路径下门控已经拦住“摆臂不到位就切进来”，
	// 但 activate_mode_immediately / 强制切模式仍可能绕过。
	// 摆臂不到位时 AirGroundMux 会把输出送给桨而不是轮子 —— 车不会动。
	if (servoGateEnabled() && (_servo_angle_est_deg < _param_arm_thresh.get())) {
		PX4_WARN("ground_drive: 摆臂仅 %.0f° < %.0f° — 轮子不会转！先摆到 52°",
			 (double)_servo_angle_est_deg, (double)_param_arm_thresh.get());
	}
}

void GroundDrive::deactivateMode()
{
	_mode_active = false;
	_sp_confirmed = false;

	// 不需要手动恢复 setpoint type：
	// ModeManagement::getSetpointType() 在模式切换时会把「被停用的模式」
	// 的 current_setpoint_type 复位成 Trajectory → control_allocator 自动恢复发布。
	PX4_INFO("ground_drive: mode INACTIVE — control_allocator 重新接管");
}

void GroundDrive::publishSetpointConfig()
{
	setpoint_config_s sc{};
	sc.timestamp = hrt_absolute_time();
	sc.source_id = (uint8_t)_mode_id;
	sc.type = setpoint_config_s::TYPE_DIRECT_ACTUATORS;
	sc.should_apply = true;
	// 0 = 不启用 setpoint 超时；失联由 arming check 的「未响应」路径兜底。
	// TODO(P6)：接上真实输出后改为 500 ms，让模块卡死时能触发 failsafe。
	sc.timeout_ms = 0;

	_setpoint_config_pub.publish(sc);

	_last_sp_config_pub = sc.timestamp;
}

void GroundDrive::checkSetpointConfigReply()
{
	setpoint_config_reply_s rep;

	for (int i = 0; i < kMaxMsgPerCycle; i++) {
		if (!_sp_reply_sub.update(&rep)) {
			break;
		}

		if (rep.source_id != (uint8_t)_mode_id) {
			continue;
		}

		_sp_have_result = true;
		_sp_last_result = rep.result;

		// ⚠️ 必须双向同步：分配器一旦被别的模块抢回去，这里要立即失效。
		//    否则会继续和 mc_rate_control / control_allocator 抢同一批话题
		//    （uORB 无仲裁，最后写入者赢），正是本模块设计要避免的静默冲突。
		const bool ok = (rep.result == setpoint_config_reply_s::RESULT_SUCCESS);

		if (!ok) {
			if (_sp_confirmed) {
				PX4_ERR("ground_drive: setpoint_config 已失效 (result=%u) → 停止驱动",
					(unsigned)rep.result);

			} else {
				PX4_ERR("ground_drive: setpoint_config REJECTED (result=%u)", (unsigned)rep.result);
			}

		} else if (!_sp_confirmed) {
			PX4_INFO("ground_drive: setpoint_config accepted → control_allocator 应已停发");
		}

		_sp_confirmed = ok;
	}
}

// ──────────────────────────────────────────────────── ④ 摆臂角度外推

void GroundDrive::updateServoEstimate()
{
	// ── 抓舵机指令 ──
	vehicle_command_s cmd;

	for (int i = 0; i < kMaxMsgPerCycle; i++) {
		if (!_vehicle_command_sub.update(&cmd)) {
			break;
		}

		float new_value = NAN;
		hrt_abstime expire_us = 0;

		if ((cmd.command == vehicle_command_s::VEHICLE_CMD_DO_SET_ACTUATOR)
		    && (lroundf(cmd.param7) == 0)) {
			// 187：param1 = 值，param7 = 索引（0 基）。**锁存**，无超时
			new_value = cmd.param1;
			expire_us = 0;

		} else if ((cmd.command == 310 /* MAV_CMD_ACTUATOR_TEST */)
			   && (lroundf(cmd.param5) == (long)(1000.f + kServoFunction))) {
			// 310：param1 = 值，param2 = **超时秒数**，param5 = 1000 + 功能号
			new_value = cmd.param1;
			// PX4 把超时钳到 3 s 上限；param2 <= 0 时也按上限算
			const float tmo_s = (cmd.param2 > 0.f) ? fminf(cmd.param2, 3.f) : 3.f;
			expire_us = (hrt_abstime)(tmo_s * 1e6f);
		}

		if (PX4_ISFINITE(new_value)) {
			_servo_cmd_value = math::constrain(new_value, -1.f, 1.f);
			_servo_cmd_time = hrt_absolute_time();
			_servo_cmd_expire_us = expire_us;
		}
	}

	// ── 310 超时回落 ──
	// 实测（2026-10-18）：QGC 的 Actuators 滑条用 310 驱动舵机，param2 = 1 s。
	// 一旦离开 Vehicle Setup → Actuators 页面，QGC 就不再重发，**1 秒后**
	// mixer_module 里的 _actuator_test 超时 → outputs 回落 DIS1 = 0 → 摆臂回 0°。
	// 而本估计器如果只跟“收到的指令”走，就会一直以为摆臂还在 52° ——
	// 于是门控放行、实际 mux 却在飞行位，表现为“进了地面模式但车不动”。
	// 所以这里必须按同样的超时规则把估计值拉回 DIS(-1 → 0°)。
	float effective_cmd = _servo_cmd_value;

	if ((_servo_cmd_expire_us > 0)
	    && (hrt_elapsed_time(&_servo_cmd_time) > (hrt_abstime)_servo_cmd_expire_us)) {
		effective_cmd = -1.f;              // = SIM_GZ_SV_DIS1 → 0°
	}

	// ── 一阶限速逼近（朝目标角匀速走，模拟舵机转速上限）──
	const float target_deg = kArmAnglePerUnit * (effective_cmd + 1.f);

	// 用真实周期而不是硬编码 20 ms：nav_and_controllers 队列会被其他任务挤占，
	// 周期一抖外推速率就错，而这个角度直接决定 arming 门控是否放行（安全相关）。
	// clamp 到 [10, 100] ms，是为了防首帧/长阻塞把角度一步跳到位。
	const hrt_abstime now = hrt_absolute_time();
	float dt_s = kServoDtMin;

	if (_last_servo_update != 0) {
		dt_s = math::constrain(hrt_elapsed_time(&_last_servo_update) * 1e-6f,
				       kServoDtMin, kServoDtMax);
	}

	_last_servo_update = now;

	const float rate = _param_servo_rate.get();
	const float step = (rate > 0.f ? rate : 1.f) * dt_s;        // deg per cycle

	const float diff = target_deg - _servo_angle_est_deg;

	if (fabsf(diff) <= step) {
		_servo_angle_est_deg = target_deg;

	} else {
		_servo_angle_est_deg += copysignf(step, diff);
	}
}

// ──────────────────────────────────────────────────────────── ⑤ 输出

void GroundDrive::manualDrive(hrt_abstime now)
{
	// ★ 门控点 ②：摆臂没到位 → 输出 0（与门控点 ① 双保险）
	//   GD_SERVO_GATE=0 时此门控也失效（台架调试），此时摆臂 0° 也会驱动电机
	if (servoGateEnabled() && (_servo_angle_est_deg < _param_arm_thresh.get())) {
		_last_stick_valid = false;
		publishZeroMotors(now);
		return;
	}

	manual_control_setpoint_s mcs;

	if (!_manual_control_sub.copy(&mcs) || !mcs.valid) {
		_last_stick_valid = false;
		publishZeroMotors(now);
		return;
	}

	_last_stick_valid = true;

	// ── 前后轴：throttle 与 pitch 饱和求和 ──
	// QGC 虚拟摇杆是双摇杆，两个纵轴分属不同摇杆：
	//     左摇杆上下 = MANUAL_CONTROL.z → manual_control_setpoint.throttle
	//     右摇杆上下 = MANUAL_CONTROL.x → manual_control_setpoint.pitch
	// 玩家习惯不统一（有人推左、有人推右），两个都接受。
	//
	// ⚠️ 早期用的是 `|pitch| > |throttle| ? pitch : throttle`（二选一），
	//    实测问题是：两轴幅值接近时摇杆噪声会让 fwd 在两者间来回跳，
	//    车身速度跟着抖（日志里 pitch −0.24~+1.00、throttle 0~0.10，交叉点明显）。
	//    改成饱和求和：同向叠加、反向抵消、越界截断，连续且无跳变。
	// 转向只认 roll（右摇杆左右）—— 已验证可用，不改。
	// ★ 摇杆死区（GD_STICK_DZ）：回中偏置会让车自己爬行，必须先压掉
	const float dz  = _param_stick_dz.get();
	const float thr = applyStickDeadzone(mcs.throttle, dz);
	const float pit = applyStickDeadzone(mcs.pitch, dz);
	const float rol = applyStickDeadzone(mcs.roll, dz);

	const float fwd = math::constrain(pit + thr, -1.f, 1.f);

	// 摇杆 → 底盘线速度 / 角速度（轴权与官方 rover 一致：throttle 前进、roll 转向）
	//   fwd：0 = 摇杆居中 = 停车，+1 前进，−1 后退
	const float w_cmd = _param_yaw_gain.get() * rol;             // [rad/s]

	// ★ 转弯自动降速（滑移转向的通用做法，GD_TURN_SLOW=0 则关闭）
	//   本项目 W=0.165 m < L=0.21 m（W/L=0.79），强不足转向：满油门时纵向摩擦
	//   把摩擦圆占满，差速根本使不上力。实测同差速下：
	//       v=0.60 m/s → 0/5 能转；v=0.00 m/s → 2/10（最好）
	//   ⇒ 默认 GD_TURN_SLOW=1.0，满舵时纵向指令**精确归零**，退化为纯原地搓转。
	//   |roll|=0 时因子为 1，直线完全不受影响。
	//   额外好处：运动中打满舵会带来一次巨大的差速阶跃，而日志显示 9 次
	//   「开始转」的瞬间都有这种阶跃 —— 这是主动破锁的天然脉冲。
	const float slow = _param_turn_slow.get();
	float v_cmd = _param_max_speed.get() * fwd
		      * (1.f - slow * fabsf(rol));                  // [m/s]

	// 差速运动学：
	//   a 侧 = joint3 / joint5（URDF y = −0.0825）→ control[0], control[2]
	//   b 侧 = joint4 / joint6（URDF y = +0.0825）→ control[1], control[3]
	const float half_track = 0.5f * _param_wheel_track.get();
	const float v_a = v_cmd - w_cmd * half_track;
	const float v_b = v_cmd + w_cmd * half_track;

	// 轮线速 → 电机轴角速度 → 归一化
	//   ctrl = v * (齿轮比 / (轮半径 * max_omega))
	const float gear = _param_gear_ratio.get();
	const float radius = _param_wheel_r.get();
	const float max_omega = _param_motor_max.get();

	float ctrl_a = 0.f;
	float ctrl_b = 0.f;

	if ((radius > 0.f) && (max_omega > 0.f)) {
		const float k = gear / (radius * max_omega);   // [1/(m/s)]
		ctrl_a = v_a * k;
		ctrl_b = v_b * k;

		// ★ 饱和时【保转向】（照抄 DifferentialActControl 的思路）：
		//   任一侧超限就整体等比缩放，保持 ctrl_a : ctrl_b 比例不变
		//   ⇒ 转弯半径不变，只牺牲绝对车速。
		//   反例（旧版）：两侧各自 clamp → 外侧被削、内侧不动，
		//   表现为「满油门时弯转不动、车往外甩」。
		//   注意这里必须是「除同一个数」而不是分别 clamp。
		const float peak = fmaxf(fabsf(ctrl_a), fabsf(ctrl_b));

		if (peak > 1.f) {
			ctrl_a /= peak;
			ctrl_b /= peak;
		}
	}

	_last_v_cmd = v_cmd;
	_last_w_cmd = w_cmd;
	_last_ctrl_a = ctrl_a;
	_last_ctrl_b = ctrl_b;

	publishMotors(now, ctrl_a, ctrl_b);
}

void GroundDrive::publishZeroMotors(hrt_abstime now)
{
	_last_v_cmd = 0.f;
	_last_w_cmd = 0.f;
	_last_ctrl_a = 0.f;
	_last_ctrl_b = 0.f;

	publishMotors(now, 0.f, 0.f);
}

void GroundDrive::publishMotors(hrt_abstime now, float ctrl_a, float ctrl_b)
{
	actuator_motors_s am{};
	am.timestamp = now;
	am.timestamp_sample = now;
	am.reversible_flags = (uint16_t)_param_r_rev.get();

	// 只驱动前 4 路（SIM_GZ_EC_FUNC1..4），其余置 NaN = 未使用。
	// 同一个电机通道在摆臂 52° 时由 Gazebo 侧 AirGroundMux 转给轮子。
	am.control[0] = ctrl_a;   // 电机1：joint3（桨）/ joint7（轮）
	am.control[1] = ctrl_b;   // 电机2：joint4（桨）/ joint8（轮）
	am.control[2] = ctrl_a;   // 电机3：joint5（桨）/ joint9（轮）
	am.control[3] = ctrl_b;   // 电机4：joint6（桨）/ joint10（轮）

	for (int i = 4; i < actuator_motors_s::NUM_CONTROLS; i++) {
		am.control[i] = NAN;
	}

	_actuator_motors_pub.publish(am);
}

// ───────────────────────────── ⑥ land detector 输入补偿

/**
 * 发布一条「我请求 0 推力」的 vehicle_thrust_setpoint。
 *
 * 为什么需要它（2026-10-17 P4 排障结论）
 * ------------------------------------
 * 本模块用 TYPE_DIRECT_ACTUATORS，getControlMode() 什么都不启用：
 *     flag_control_rates_enabled   = false
 *     flag_control_allocation_enabled = false
 * ⇒ mc_rate_control 不运行（MulticopterRateControl.cpp:189 门控）
 * ⇒ vehicle_thrust_setpoint 从来没人发布
 *
 * 而 MulticopterLandDetector 判定「落地」的第一级判据就依赖这条消息：
 *     _vehicle_thrust_setpoint_valid = (最后接收时间 < 1 s)   // :221
 *     _has_low_throttle  = valid && (throttle <= 阈值)          // :232
 *     ground_contact     = _has_low_throttle                    // :233
 *     maybe_landed       = minimum_thrust_now && ...            // :285/307
 *     landed             = !_armed || maybe_landed              // :313
 *
 * 话题从未发布 ⇒ valid=false（时间戳停在初值 0）⇒ ground_contact=false
 *            ⇒ maybe_landed=false ⇒ **armed 时 landed 永远为 false**
 *
 * 后果实测：一解锁 commander 就报 "Takeoff detected"、普通 disarm 被拒
 *          （"Disarming denied: not landed"）、所有 failsafe 按「飞行中」处理
 *          （日志里真的出现了 "RTL: start return at 13 m"）。
 *
 * 这里补发 throttle=0，把 land detector 缺的那个输入补回去。
 * 语义上也如实：地面模式确实不需要任何螺旋桨推力。
 *
 * ⚠️ 四条铁律（破坏任意一条都会引入新问题，排查记录见文档）
 * ----------------------------------------------------------------
 * 1) 只在 _mode_active 时发布。
 *    mc_rate_control 的发布门控是 flag_control_rates_enabled，恰好与本模式
 *    互斥；但 uORB 没有仲裁（最后写入者赢），越界发布就会和它抢同一个话题。
 * 2) xyz 必须填 0，绝不能填 NaN。
 *    NaN 会被 control_allocator 解释成「停止该轴所有电机」
 *    （ActuatorEffectiveness::stopMotorsBasedOnThrustSetpoint，:230）。
 * 3) 只在 _sp_confirmed 之后发布，避免在分配器还没让位时插入假指令。
 * 4) 【不能】只在已解锁时发布 —— 必须未解锁也发。
 *    否则解锁瞬间的第一帧 armed=true、valid=false，land detector 抖一下
 *    就会触发 "Takeoff detected" → _have_taken_off_since_arming=true →
 *    COM_DISARM_LAND(2 s) 立刻把车自动上锁（实测过）。
 *    未解锁时多发一条是无害的：landed 靠 !_armed 本来就是 true。
 *
 * ⚠️ 已排查的副作用（均为无风险）：
 *   · control_allocator：只在 _publish_controls=false 时不写输出；xyz=0 也不会
 *     触发停轴（只有 NaN 才会）
 *   · mc_hover_thrust_estimator：要求 _in_air，而 _in_air 只在
 *     dist_bottom > 1 m 时才置 true ⇒ 地面跑时 EKF 不运行，不会污染 MPC_THR_HOVER
 *   · vehicle_magnetometer / VFR_HUD：收到 0 与「收不到」在本场景下等价
 */
void GroundDrive::publishThrustZero(hrt_abstime now)
{
	vehicle_thrust_setpoint_s sp{};
	sp.timestamp = now;
	sp.timestamp_sample = now;

	// 机体 Z 轴向下为正，所以「不要推力」写 0。
	// 显式写 0 而不是 NAN，见铁律 2。
	sp.xyz[0] = 0.f;
	sp.xyz[1] = 0.f;
	sp.xyz[2] = 0.f;

	_vehicle_thrust_sp_pub.publish(sp);
	_thrust_sp_count++;
}

// ────────────────────────────────────────────────────────── 模块样板

int GroundDrive::print_status()
{
	PX4_INFO("registered      : %s", _registered ? "yes" : "no");

	if (_registered) {
		PX4_INFO("mode_id         : %d  (`commander mode ext%d`)",
			 (int)_mode_id, (int)_mode_id - (int)vehicle_status_s::NAVIGATION_STATE_EXTERNAL1 + 1);
		PX4_INFO("arming_check_id : %d", (int)_arming_check_id);
		PX4_INFO("setpoint_config : current=%s  last_reply=%s",
			 _sp_confirmed ? "confirmed" : "none",
			 !_sp_have_result ? "n/a"
			 : (_sp_last_result == setpoint_config_reply_s::RESULT_SUCCESS ? "SUCCESS" : "FAIL"));
	}

	PX4_INFO("mode active     : %s", _mode_active ? "yes" : "no");
	PX4_INFO("servo gate      : %s (threshold %.1f deg)",
		 servoGateEnabled() ? "ENABLED" : "DISABLED (bench debug!)", (double)_param_arm_thresh.get());
	PX4_INFO("servo cmd       : %+.3f  →  est angle %.1f deg (rate %.1f deg/s)",
		 (double)_servo_cmd_value, (double)_servo_angle_est_deg, (double)_param_servo_rate.get());

	// 原始摇杆四轴：排查「某个方向没反应」时先看这里，别看下面算出来的值
	{
		manual_control_setpoint_s mcs{};

		if (_manual_control_sub.copy(&mcs)) {
			PX4_INFO("sticks raw      : thr=%+.2f  pitch=%+.2f  roll=%+.2f  yaw=%+.2f  valid=%s",
				 (double)mcs.throttle, (double)mcs.pitch, (double)mcs.roll, (double)mcs.yaw,
				 mcs.valid ? "yes" : "no");

			const float dzv = _param_stick_dz.get();
			PX4_INFO("sticks dz       : thr=%+.3f  pitch=%+.3f  roll=%+.3f   (GD_STICK_DZ=%.2f)",
				 (double)applyStickDeadzone(mcs.throttle, dzv),
				 (double)applyStickDeadzone(mcs.pitch, dzv),
				 (double)applyStickDeadzone(mcs.roll, dzv), (double)dzv);
		}
	}

	PX4_INFO("drive           : stick_valid=%s  v=%.3f m/s  w=%.3f rad/s",
		 _last_stick_valid ? "yes" : "no", (double)_last_v_cmd, (double)_last_w_cmd);

	// 标定参数全量打印：现场排查「车不走 / 转不动」时先看这里 ——
	// 十有八九是 parameters.bson 里的旧值把 airframe/module.yaml 的默认值顶掉了。
	PX4_INFO("calib           : v_max=%.2f m/s  yaw_gain=%.2f rad/s  turn_slow=%.2f",
		 (double)_param_max_speed.get(), (double)_param_yaw_gain.get(),
		 (double)_param_turn_slow.get());
	PX4_INFO("calib           : omega_max=%.0f  gear=%.1f  R=%.4f m  track=%.3f m",
		 (double)_param_motor_max.get(), (double)_param_gear_ratio.get(),
		 (double)_param_wheel_r.get(), (double)_param_wheel_track.get());
	PX4_INFO("ctrl[a,b]       : %+.4f / %+.4f   (a→control[0,2]  b→control[1,3])",
		 (double)_last_ctrl_a, (double)_last_ctrl_b);
	PX4_INFO("vehicle         : arming_state=%u nav_state=%u",
		 (unsigned)_vehicle_status.arming_state, (unsigned)_vehicle_status.nav_state);
	PX4_INFO("thrust sp       : %llu 条已发（land detector 输入补偿）",
		 (unsigned long long)_thrust_sp_count);
	perf_print_counter(_cycle_perf);

	return 0;
}

int GroundDrive::task_spawn(int argc, char *argv[])
{
	GroundDrive *instance = new GroundDrive();

	if (instance) {
		desc.object.store(instance);
		desc.task_id = task_id_is_work_queue;

		if (instance->init()) {
			return PX4_OK;
		}

	} else {
		PX4_ERR("alloc failed");
	}

	delete instance;
	desc.object.store(nullptr);
	desc.task_id = -1;

	return PX4_ERROR;
}

int GroundDrive::custom_command(int argc, char *argv[])
{
	return print_usage("unknown command");
}

int GroundDrive::print_usage(const char *reason)
{
	if (reason) {
		PX4_WARN("%s\n", reason);
	}

	PRINT_MODULE_DESCRIPTION(
		R"DESCR_STR(
### Description
Ground drive (wheeled) external mode.

Registers the external mode GROUND_DRIVE. While active it publishes
setpoint_config{TYPE_DIRECT_ACTUATORS}, which makes the control allocator
relinquish actuator_motors, so this module becomes the only writer.

Fly mode is untouched: while inactive the module never touches actuator_motors.
)DESCR_STR");

	PRINT_MODULE_USAGE_NAME("ground_drive", "controller");
	PRINT_MODULE_USAGE_COMMAND("start");
	PRINT_MODULE_USAGE_DEFAULT_COMMANDS();

	return 0;
}

extern "C" __EXPORT int ground_drive_main(int argc, char *argv[])
{
	return ModuleBase::main(GroundDrive::desc, argc, argv);
}
