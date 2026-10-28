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
 * @file ground_drive.hpp
 *
 * 地面驱动（轮式）外部模式。
 *
 * 架构（详见工作区文档 `PX4地面驱动方案.md`）：
 *   本模块注册一个 PX4 外部模式（NAVIGATION_STATE_EXTERNAL1+N），
 *   激活时向 commander 发布 setpoint_config{TYPE_DIRECT_ACTUATORS}，
 *   使 `flag_control_allocation_enabled = false`
 *   → control_allocator 在 publish_actuator_controls() 早退、不再发布
 *   → 本模块成为 actuator_motors 的唯一发布者。
 *
 * 飞行模式下本模块常驻但不写电机（`_mode_active == false`）。
 */

#pragma once

#include <perf/perf_counter.h>
#include <px4_platform_common/defines.h>
#include <px4_platform_common/log.h>
#include <px4_platform_common/module.h>
#include <px4_platform_common/module_params.h>
#include <px4_platform_common/px4_work_queue/ScheduledWorkItem.hpp>

#include <uORB/Publication.hpp>
#include <uORB/Subscription.hpp>
#include <uORB/SubscriptionInterval.hpp>

#include <uORB/topics/actuator_motors.h>
#include <uORB/topics/arming_check_reply.h>
#include <uORB/topics/arming_check_request.h>
#include <uORB/topics/manual_control_setpoint.h>
#include <uORB/topics/parameter_update.h>
#include <uORB/topics/register_ext_component_reply.h>
#include <uORB/topics/register_ext_component_request.h>
#include <uORB/topics/setpoint_config.h>
#include <uORB/topics/setpoint_config_reply.h>
#include <uORB/topics/unregister_ext_component.h>
#include <uORB/topics/vehicle_command.h>
#include <uORB/topics/vehicle_land_detected.h>
#include <uORB/topics/vehicle_status.h>
#include <uORB/topics/vehicle_thrust_setpoint.h>

using namespace time_literals;

class GroundDrive : public ModuleBase, public ModuleParams, public px4::ScheduledWorkItem
{
public:
	GroundDrive();
	~GroundDrive() override;

	static Descriptor desc;

	/** @see ModuleBase */
	static int task_spawn(int argc, char *argv[]);

	/** @see ModuleBase */
	static int custom_command(int argc, char *argv[]);

	/** @see ModuleBase */
	static int print_usage(const char *reason = nullptr);

	/** @see ModuleBase */
	int print_status() override;

	bool init();

private:
	void Run() override;

	// ── ① 注册 / 注销 ─────────────────────────────────────────
	void tryRegister();                 //!< 未注册时按 1 s 周期重发注册请求
	void registerExternalMode();        //!< 发 register_ext_component_request
	void handleRegistrationReply();     //!< 收 register_ext_component_reply
	void unregisterExternalMode();      //!< 退出时发 unregister_ext_component

	// ── ② arming check（门控点 ①）────────────────────────────
	void replyToArmingCheck(uint8_t request_id);
	//! GD_SERVO_GATE 是否为 1。false = 门控彻底失效（仅台架调试）
	bool servoGateEnabled();

	// ── ③ 模式激活 ────────────────────────────────────────────
	void activateMode();
	void deactivateMode();
	void publishSetpointConfig();       //!< 发 setpoint_config{TYPE_DIRECT_ACTUATORS}
	void checkSetpointConfigReply();    //!< 读 setpoint_config_reply 并打印结果

	// ── ④ 摆臂角度外推器 ──────────────────────────────────────
	void updateServoEstimate();

	// ── ⑤ 输出 ───────────────────────────────────────────────
	void manualDrive(hrt_abstime now);
	void publishZeroMotors(hrt_abstime now);
	void publishMotors(hrt_abstime now, float ctrl_a, float ctrl_b);

	// ── ⑥ land detector 输入补偿 ─────────────────────────────
	//! 补发 vehicle_thrust_setpoint{0}，让 MulticopterLandDetector 能正常判“落地”。
	//! 详见 .cpp 里的实现注释（为什么必须有它，以及四条铁律）。
	void publishThrustZero(hrt_abstime now);

	static constexpr uint64_t kRequestId{0x67726F756E645F31ULL};   //!< 'ground_1'
	static constexpr char kModeName[]{"GROUND_DRIVE"};

	// 注册状态
	bool         _reg_request_sent{false};
	bool         _registered{false};
	int8_t       _arming_check_id{-1};
	int8_t       _mode_id{-1};
	uint32_t     _register_attempts{0};
	hrt_abstime  _reg_request_time{0};

	// 模式状态
	bool         _mode_active{false};
	bool         _sp_confirmed{false};      //!< 当前是否已确认（激活期间有效）
	bool         _sp_have_result{false};    //!< 是否收到过 setpoint_config_reply
	uint8_t      _sp_last_result{0};        //!< 最近一次 reply 的 result
	hrt_abstime  _last_sp_config_pub{0};
	//! 危险状态报警限速（“摆臂在轮位但飞控在驾驶”，见 Run()）
	hrt_abstime  _last_hazard_warn{0};

	// 状态缓存
	vehicle_status_s _vehicle_status{};     //!< Run() 内刷新，供 arming check 读 armed
	bool         _landed{true};             //!< vehicle_land_detected.landed 缓存（门控用）

	// 摆臂角度外推
	float        _servo_cmd_value{-1.f};    //!< 最近一次舵机指令 [-1, 1] → 0°
	float        _servo_angle_est_deg{0.f};
	//! 最近一次舵机指令的时间
	hrt_abstime  _servo_cmd_time{0};
	//! 该指令的有效期（us）。0 = 无超时（187 锁存）；310 由 param2 指定，最长 3 s
	uint32_t     _servo_cmd_expire_us{0};
	//! 上一次执行外推的时刻（用于算真实 dt，而不是硬编码 20 ms）
	hrt_abstime  _last_servo_update{0};

	// 上一次的控制量（仅用于 status 显示 / 调试）
	float        _last_v_cmd{0.f};
	float        _last_w_cmd{0.f};
	float        _last_ctrl_a{0.f};
	float        _last_ctrl_b{0.f};
	bool         _last_stick_valid{false};
	uint64_t     _thrust_sp_count{0};       //!< 发了多少条补偿的 thrust setpoint（仅调试用）

	perf_counter_t _cycle_perf{nullptr};

	// Subscriptions
	uORB::SubscriptionInterval _parameter_update_sub{ORB_ID(parameter_update), 1_s};
	uORB::Subscription _reg_reply_sub{ORB_ID(register_ext_component_reply)};
	uORB::Subscription _arming_check_req_sub{ORB_ID(arming_check_request)};
	uORB::Subscription _vehicle_status_sub{ORB_ID(vehicle_status)};
	uORB::Subscription _vehicle_land_detected_sub{ORB_ID(vehicle_land_detected)};
	uORB::Subscription _vehicle_command_sub{ORB_ID(vehicle_command)};
	uORB::Subscription _manual_control_sub{ORB_ID(manual_control_setpoint)};
	uORB::Subscription _sp_reply_sub{ORB_ID(setpoint_config_reply)};

	// Publications
	uORB::Publication<register_ext_component_request_s> _reg_req_pub{ORB_ID(register_ext_component_request)};
	uORB::Publication<unregister_ext_component_s>       _unreg_pub{ORB_ID(unregister_ext_component)};
	uORB::Publication<arming_check_reply_s>             _arming_check_reply_pub{ORB_ID(arming_check_reply)};
	uORB::Publication<setpoint_config_s>                _setpoint_config_pub{ORB_ID(setpoint_config)};
	uORB::Publication<actuator_motors_s>                _actuator_motors_pub{ORB_ID(actuator_motors)};
	uORB::Publication<vehicle_thrust_setpoint_s>       _vehicle_thrust_sp_pub{ORB_ID(vehicle_thrust_setpoint)};

	DEFINE_PARAMETERS(
		(ParamFloat<px4::params::GD_SERVO_RATE>)   _param_servo_rate,
		(ParamFloat<px4::params::GD_STICK_DZ>)     _param_stick_dz,    // 摇杆死区
		(ParamFloat<px4::params::GD_ARM_MIN_DEG>)  _param_arm_thresh,
		(ParamInt<px4::params::GD_SERVO_GATE>)     _param_servo_gate,  // 门控总开关（0=台架禁用）
		(ParamFloat<px4::params::GD_MAX_SPEED>)    _param_max_speed,
		(ParamFloat<px4::params::GD_WHEEL_TRACK>)  _param_wheel_track,
		(ParamFloat<px4::params::GD_YAW_GAIN>)     _param_yaw_gain,
		(ParamFloat<px4::params::GD_MAX_OMEGA>)    _param_motor_max,
		(ParamFloat<px4::params::GD_GEAR_RATIO>)   _param_gear_ratio,   // 64:1 行星减速器
		(ParamFloat<px4::params::GD_WHEEL_R>)      _param_wheel_r,      // 轮半径 [m]
		(ParamFloat<px4::params::GD_TURN_SLOW>)    _param_turn_slow,    // 转弯降速系数
		(ParamInt<px4::params::CA_R_REV>)          _param_r_rev
	)
};
