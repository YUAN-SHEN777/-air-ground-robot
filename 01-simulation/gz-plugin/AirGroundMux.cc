/*
 * AirGroundMux.cc —— "空-地机械切换"的 Gazebo 系统插件（仅仿真用）
 *
 * 为什么需要它
 * ------------
 * 真机上桨和轮是【同一台电机】，靠摆臂机构切换：
 *     摆臂 0°  → 电机连桨
 *     摆臂 52° → 电机连轮
 * 仿真里桨和轮是两个【独立实体】，PX4 只有一路 actuator_motors。
 * 如果没有本插件，地面模式下桨会跟着转（52° 姿态下产生斜向推力把车推跑/掀翻）。
 *
 * 它做什么
 * --------
 *   输入  A: gz.msgs.Actuators  "/<model>/command/motor_speed"   ← PX4 gz_bridge ESC 接口
 *   输入  B: 摆臂关节 servo_0 的实际角度（从 ECS 组件读，不是外推！）
 *
 *   if 摆臂 >= 阈值（地面位）:
 *       桨  ← 发 0（多旋翼电机模型停止）
 *       轮  ← joint7..10 速度 = signed(velocity[i]) * (motor_max_omega/gear_ratio) * sign[i]
 *   else（飞行位）:
 *       桨  ← max(0, signed) * motor_max_omega，轮 ← 速度 0
 *
 *   注意【不能两者都原样转发】：这里的 velocity 是「中位 = 停」编码，
 *   500 表示停而不是 500 rad/s 的怡速。
 *
 * 数值约定（务必与 PX4 侧一致）—— 「DShot 3D / reversible」语义
 * --------------------------------------------------------------
 *   PX4 MixingOutput: outputs = interpolate(control[-1,1], SIM_GZ_EC_MIN..MAX)
 *   默认 SIM_GZ_EC_MIN=0 / MAX=1000，airframe 里再把 SIM_GZ_EC_DIS1..4 设成 500：
 *
 *       状态               control   velocity   signed   含义
 *       未解锁              ——        500         0      停
 *       armed, control= 0    0        500         0      停
 *       armed, control=+1   +1       1000        +1      满前进 / 满油门
 *       armed, control=-1   -1          0        -1      满反转
 *
 *   ⇒ signed = (velocity - zero_offset) / half_span ∈ [-1, +1]
 *
 *   ⚠️ 关键：DIS 必须是 500（中位），不能是默认的 0。
 *      DIS=0 时「未解锁」和「满反转」是同一个数字，无法区分 —— 这正是
 *      早期版本「未解锁却满速倒车」的根因。
 *
 *   该语义与实车 DShot 3D 一致：PX4 的 DShot 驱动对 reversible 通道把
 *   [48, 2047] 的中位对到 DSHOT_3D_SPLIT(=1048) 作为「停」，未解锁发 DISARM 值。
 *   所以仿真里验证过的标定数据可以直接搬到真机。
 *
 * SDF 用法（写在 model.sdf 的 <model> 内）
 * ---------------------------------------
 *   <plugin filename="libairground_mux.so" name="airground::AirGroundMux">
 *     <servo_joint>servo_0</servo_joint>
 *     <arm_threshold_deg>48</arm_threshold_deg>
 *     <wheel_joints>joint7 joint8 joint9 joint10</wheel_joints>
 *     <wheel_signs>-1 1 -1 1</wheel_signs>
 *     <gear_ratio>64</gear_ratio>
 *     <motor_max_omega>1000</motor_max_omega>
 *     <!-- 轮侧 ω 上限，不写 = 跟 motor_max_omega 一致。
 *          想让她面跑快点就只改这个，不要动 motor_max_omega（桨的推力标定） -->
 *     <wheel_max_omega>2400</wheel_max_omega>
 *     <zero_offset>500</zero_offset>
 *     <half_span>500</half_span>
 *     <!-- 下面两个一般不用写，默认按模型名自动推导 -->
 *     <esc_topic>/airground_0/command/motor_speed</esc_topic>
 *     <prop_topic>/airground_0/command/prop_speed</prop_topic>
 *   </plugin>
 *
 * ⚠️ 本插件只影响仿真。真机没有这个需求（机械切换是物理存在的）。
 */

#include <algorithm>
#include <chrono>
#include <cmath>
#include <mutex>
#include <sstream>
#include <string>
#include <vector>

#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/JointPosition.hh>
#include <gz/sim/components/JointVelocityCmd.hh>
#include <gz/sim/components/Name.hh>
#include <gz/transport/Node.hh>

namespace airground
{

/// 桨指令心跳：即使内容未变，也每 100 ms 重发一次（防下游订阅晚于首次发布）
constexpr int64_t kPropHeartbeatNs{100'000'000};

class AirGroundMux : public gz::sim::System,
		     public gz::sim::ISystemConfigure,
		     public gz::sim::ISystemPreUpdate
{
public:
	void Configure(const gz::sim::Entity &_entity,
		       const std::shared_ptr<const sdf::Element> &_sdf,
		       gz::sim::EntityComponentManager &_ecm,
		       gz::sim::EventManager &_eventMgr) override;

	void PreUpdate(const gz::sim::UpdateInfo &_info,
		       gz::sim::EntityComponentManager &_ecm) override;

private:
	void OnActuators(const gz::msgs::Actuators &_msg);

	/// velocity → 有符号归一化 [-1, +1]（_zeroOffset / _halfSpan 决定中位与满量程）
	/// NaN → 0（未使用的通道可能是 NaN；NaN*0 仍是 NaN，必须在此拦住）
	double SignedNorm(double _vel) const
	{
		const double s = (_vel - _zeroOffset) / _halfSpan;
		return std::isnan(s) ? 0.0 : std::clamp(s, -1.0, 1.0);
	}

	gz::transport::Node _node;
	gz::transport::Node::Publisher _propPub;

	gz::sim::Entity _servoJoint{gz::sim::kNullEntity};
	std::vector<gz::sim::Entity> _wheelJoints;

	std::string _escTopic;
	std::string _propTopic;

	double _armThresholdDeg{48.0};
	double _gearRatio{64.0};
	//! 桨侧最大角速度（由推力标定决定，不能改）
	double _motorMaxOmega{1000.0};
	//! 轮侧最大角速度。0 = 跟随 _motorMaxOmega。
	//!
	//! 为什么要单独开一个：桨和轮**共用同一个电机通道**，而桨的 ω_max 由
	//! 推力标定（T = kT·w²）锁定。想让它上跑快点就必须只改轮侧，
	//! 否则起飞推力直接翻四倍。
	//! ⚠️ 必须与 PX4 的 GD_MAX_OMEGA 保持一致，否则「指令速度 ≠ 实际速度」。
	double _wheelMaxOmega{0.0};
	//! PX4 侧的中位值：500 = 停（对应 airframe 的 SIM_GZ_EC_DIS1..4 = 500）
	double _zeroOffset{500.0};
	//! 半量程：|velocity - zero_offset| = half_span 时对应 signed = ±1
	double _halfSpan{500.0};
	//! 桨侧「零推力」对应的 velocity。
	//!
	//! 桨和轮共用同一批电机通道，但**物理需求相反**：
	//!   桨：单向   0..1 推力          → 需要 velocity 与推力线性，不能有中位
	//!   轮：双向  -1..+1 速度         → 需要 500 = 停 的中位编码
	//! PX4 的 CA_R_REV / SIM_GZ_EC_MIN-MAX 只能二选一，所以必须在 mux 里分模式解码。
	//!
	//!   500 = PX4 可逆编码（CA_R_REV 置位，thrust -1..+1 → 0..1000）
	//!     0 = PX4 单向编码（CA_R_REV 清零，thrust  0..1  → 0..1000）
	//!
	//! ⚠️ 用 500 时，负推力（配平/偏航差动）会被截断成 0 —— 这正是
	//!    「悬停时某几个桨转速恒为 0、偏航没权限」的根因，飞行请用 0。
	double _propZeroOffset{500.0};
	std::vector<double> _wheelSigns{1.0, 1.0, 1.0, 1.0};

	// 来自 transport 线程的最新指令
	std::mutex _mutex;
	std::vector<double> _cmdVel;
	bool _cmdDirty{true};

	// 上一次真正发出去的桨指令（用于"变化或超时才发"）
	std::vector<double> _lastPropVel;
	int64_t _lastPropSendNs{-1};
};

//////////////////////////////////////////////////

/// 按空白切词
static std::vector<std::string> SplitWords(const std::string &_s)
{
	std::vector<std::string> out;
	std::istringstream iss(_s);
	std::string w;

	while (iss >> w) {
		out.push_back(w);
	}

	return out;
}

/// 按名字找 joint 实体，并确保 JointPosition / JointVelocityCmd 组件存在
static gz::sim::Entity FindJoint(gz::sim::EntityComponentManager &_ecm, const std::string &_name)
{
	const auto joint = _ecm.EntityByComponents(gz::sim::components::Joint(),
						   gz::sim::components::Name(_name));

	if (joint == gz::sim::kNullEntity) {
		return gz::sim::kNullEntity;
	}

	if (!_ecm.Component<gz::sim::components::JointPosition>(joint)) {
		_ecm.CreateComponent(joint, gz::sim::components::JointPosition({0.0}));
	}

	if (!_ecm.Component<gz::sim::components::JointVelocityCmd>(joint)) {
		_ecm.CreateComponent(joint, gz::sim::components::JointVelocityCmd({0.0}));
	}

	return joint;
}

//////////////////////////////////////////////////

void AirGroundMux::Configure(const gz::sim::Entity &_entity,
			     const std::shared_ptr<const sdf::Element> &_sdf,
			     gz::sim::EntityComponentManager &_ecm,
			     gz::sim::EventManager &)
{
	gz::sim::Model model(_entity);
	const std::string modelName = model.Name(_ecm);

	// ── 参数（用 HasElement 判断，避开不同 sdformat 版本 Get(key,default) 签名差异）──
	const auto getStr = [&](const char *_k, const std::string &_def) -> std::string {
		return _sdf->HasElement(_k) ? _sdf->Get<std::string>(_k) : _def;
	};
	const auto getDbl = [&](const char *_k, double _def) -> double {
		return _sdf->HasElement(_k) ? _sdf->Get<double>(_k) : _def;
	};

	const std::string servoJointName = getStr("servo_joint", "servo_0");
	_armThresholdDeg = getDbl("arm_threshold_deg", 48.0);
	_gearRatio = getDbl("gear_ratio", 64.0);
	_motorMaxOmega = getDbl("motor_max_omega", 1000.0);
	_wheelMaxOmega = getDbl("wheel_max_omega", 0.0);
	_zeroOffset = getDbl("zero_offset", 500.0);
	_halfSpan = getDbl("half_span", 500.0);
	_propZeroOffset = getDbl("prop_zero_offset", 500.0);

	if (_halfSpan == 0.0) {
		_halfSpan = 500.0;
	}

	if (_wheelMaxOmega <= 0.0) {
		_wheelMaxOmega = _motorMaxOmega;      // 向后兼容：不写就跟桨一致
	}

	const std::string wheelJointsStr = getStr("wheel_joints", "joint7 joint8 joint9 joint10");
	const std::string wheelSignsStr = getStr("wheel_signs", "1 1 1 1");

	// 默认话题按 PX4 gz_bridge 的命名推导：/<模型实例名>/command/motor_speed
	_escTopic = getStr("esc_topic", "/" + modelName + "/command/motor_speed");
	_propTopic = getStr("prop_topic", "/" + modelName + "/command/prop_speed");

	// ── 摆臂关节 ──
	_servoJoint = FindJoint(_ecm, servoJointName);

	if (_servoJoint == gz::sim::kNullEntity) {
		gzerr << "[AirGroundMux] 找不到摆臂关节 '" << servoJointName
		      << "'，插件不工作（模型保持原样）" << std::endl;

	} else {
		gzmsg << "[AirGroundMux] 摆臂关节 '" << servoJointName
		      << "' 阈值 " << _armThresholdDeg << " deg" << std::endl;
	}

	// ── 轮子关节 ──
	const std::vector<std::string> wheelNames = SplitWords(wheelJointsStr);
	const std::vector<std::string> signStrs = SplitWords(wheelSignsStr);

	for (size_t i = 0; i < wheelNames.size(); ++i) {
		const auto joint = FindJoint(_ecm, wheelNames[i]);

		if (joint == gz::sim::kNullEntity) {
			gzerr << "[AirGroundMux] 找不到轮子关节 '" << wheelNames[i] << "'" << std::endl;
			continue;
		}

		_wheelJoints.push_back(joint);

		double sign = 1.0;

		if (i < signStrs.size()) {
			try {
				sign = std::stod(signStrs[i]);

			} catch (...) {
				sign = 1.0;
			}
		}

		_wheelSigns.push_back(sign);
	}

	gzmsg << "[AirGroundMux] 轮子关节 " << _wheelJoints.size() << " 个；esc=" << _escTopic
	      << " prop=" << _propTopic << std::endl;
	gzmsg << "[AirGroundMux] 换算：wheel_signed = (vel - " << _zeroOffset << ") / " << _halfSpan
	      << "；wheel_omega = signed * (" << _wheelMaxOmega << " / " << _gearRatio << ")"
	      << "；prop_omega = clamp((vel - " << _propZeroOffset << ") / "
	      << ((_propZeroOffset <= 0.0) ? 1000.0 : (1000.0 - _propZeroOffset)) << ", 0, 1) * "
	      << _motorMaxOmega
	      << "；轮顶速 = " << (_wheelMaxOmega / _gearRatio * 0.04) << " m/s(R=0.04 估)" << std::endl;

	// ── 话题 ──
	_propPub = _node.Advertise<gz::msgs::Actuators>(_propTopic);

	if (!_propPub.Valid()) {
		gzerr << "[AirGroundMux] 无法发布 " << _propTopic << std::endl;
	}

	if (!_node.Subscribe(_escTopic, &AirGroundMux::OnActuators, this)) {
		gzerr << "[AirGroundMux] 无法订阅 " << _escTopic << std::endl;
	}
}

//////////////////////////////////////////////////

void AirGroundMux::OnActuators(const gz::msgs::Actuators &_msg)
{
	std::lock_guard<std::mutex> lock(_mutex);

	_cmdVel.clear();

	for (int i = 0; i < _msg.velocity_size(); ++i) {
		_cmdVel.push_back(_msg.velocity(i));
	}

	_cmdDirty = true;
}

//////////////////////////////////////////////////

void AirGroundMux::PreUpdate(const gz::sim::UpdateInfo &_info,
			     gz::sim::EntityComponentManager &_ecm)
{
	if (_info.paused) {
		return;
	}

	// ── 读真实摆臂角度（rad → deg）──
	double armDeg = 0.0;

	if (_servoJoint != gz::sim::kNullEntity) {
		const auto pos = _ecm.Component<gz::sim::components::JointPosition>(_servoJoint);

		if (pos && !pos->Data().empty()) {
			armDeg = pos->Data()[0] * 180.0 / M_PI;
		}
	}

	const bool groundMode = (armDeg >= _armThresholdDeg);

	// ── 取最新指令 ──
	std::vector<double> cmd;
	bool dirty = false;

	{
		std::lock_guard<std::mutex> lock(_mutex);
		cmd = _cmdVel;
		dirty = _cmdDirty;
		_cmdDirty = false;
	}

	if (cmd.empty()) {
		cmd.assign(_wheelJoints.size() > 0 ? _wheelJoints.size() : 4, _zeroOffset);
	}

	// ── 轮子 ──
	// signed ∈ [-1, +1]：-1 = 满反转，0 = 停，+1 = 满前进
	// 飞行位把 scale 置 0，等于把轮速拉 0（不再需要额外的 groundMode 三元）
	const double wheelScale = (groundMode && _gearRatio > 0.0)
				  ? (_wheelMaxOmega / _gearRatio)
				  : 0.0;

	for (size_t i = 0; i < _wheelJoints.size(); ++i) {
		const double vel = (i < cmd.size()) ? cmd[i] : _zeroOffset;
		const double omega = SignedNorm(vel) * wheelScale * _wheelSigns[i];

		auto *cmdComp = _ecm.Component<gz::sim::components::JointVelocityCmd>(_wheelJoints[i]);

		if (cmdComp) {
			cmdComp->Data() = {omega};

		} else {
			_ecm.CreateComponent(_wheelJoints[i],
					     gz::sim::components::JointVelocityCmd({omega}));
		}
	}

	// ── 桨：地面模式发 0；飞行模式把推力 ∈ [0,1] 还原成 [0, motor_max_omega] ──
	// 注意：不能再「原样转发」velocity —— PX4 给的是编码值，不是转速。
	// 零推力点由 prop_zero_offset 决定（见成员注释），不能用 SignedNorm()：
	// 那个是给轮子的中位编码用的，套在桨上会把负推力悄悄吃掉。
	std::vector<double> propVel(cmd.size(), 0.0);

	if (!groundMode) {
		const double propSpan =
			(_propZeroOffset <= 0.0) ? 1000.0 : (1000.0 - _propZeroOffset);

		for (size_t i = 0; i < cmd.size(); ++i) {
			const double raw = (cmd[i] - _propZeroOffset) / propSpan;
			const double s = std::isnan(raw) ? 0.0 : std::clamp(raw, 0.0, 1.0);
			propVel[i] = s * _motorMaxOmega;
		}
	}

	const bool changed = (propVel != _lastPropVel);
	const int64_t nowNs =
		std::chrono::duration_cast<std::chrono::nanoseconds>(_info.simTime).count();
	const bool heartbeat = (_lastPropSendNs < 0) || ((nowNs - _lastPropSendNs) > kPropHeartbeatNs);

	if (dirty || changed || heartbeat) {
		gz::msgs::Actuators msg;
		auto *vel = msg.mutable_velocity();

		for (const double v : propVel) {
			vel->Add(v);
		}

		_propPub.Publish(msg);

		_lastPropVel = propVel;
		_lastPropSendNs = nowNs;
	}
}

} // namespace airground

GZ_ADD_PLUGIN(airground::AirGroundMux,
	      gz::sim::System,
	      airground::AirGroundMux::ISystemConfigure,
	      airground::AirGroundMux::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(airground::AirGroundMux, "airground::AirGroundMux")
