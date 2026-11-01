/*
 * ReverseThrust.cc —— 给 Gazebo 的电机模型补上「反转推力」（仅仿真用）
 *
 * 问题
 * ----
 * gz::sim::systems::MulticopterMotorModel 的推力公式是
 *
 *      F = k_T * omega^2        沿 link 的 +Z
 *
 * 平方运算把符号吃掉了。于是 omega = -300 仍然产生向上的 0.09 * T_max，
 * 而真实螺旋桨反转时推力方向是反的（迎角反号）。
 *
 * PX4 的 CA_REV_THR_FRAC（默认 0.4）就是在描述这个现象：
 * 反转只产生正向推力的约 40%。地面双向驱动需要 CA_R_REV=15，
 * PX4 会让分配器输出负推力来构成偏航差动 —— 仿真做不到，
 * 实测摆臂 0° 的偏航峰值因此从 40 °/s 掉到 26 °/s（-35%）。
 *
 * 为什么是「修正插件」而不是换掉整个电机模型
 * ------------------------------------------
 * 本项目的 gz 版本里，自己施加反扭矩会让 dartsim 的 ODE 碰撞检测
 * 在约 45 s 后崩掉（ODE INTERNAL ERROR 1，collision_space.cpp:460）；
 * 把 momentConstant 置 0 就不崩 —— 所以力矩那一半交还给原厂插件。
 *
 * 本插件【只补力，不碰力矩，也不碰关节速度】：
 *      原厂给的：      +k_T * omega^2
 *      本插件在 omega<0 时再补：  -k_T * omega^2 * (1 + reverse_fraction)
 *      合计：          -k_T * omega^2 * reverse_fraction      ← 反向推力
 *
 * ⚠️ omega >= 0 时本插件【什么都不做】，所以正推力路径与原厂完全一致，
 *    不会影响任何已有的正推力标定结果。
 *
 * SDF 用法（每个桨一个，写在对应 MulticopterMotorModel 旁边）
 * -----------------------------------------------------------
 *   <plugin filename="libairground_revthrust.so" name="airground::ReverseThrust">
 *     <linkName>link1</linkName>
 *     <motorConstant>5.002070e-06</motorConstant>
 *     <maxRotVelocity>1000</maxRotVelocity>
 *     <timeConstantUp>0.0125</timeConstantUp>
 *     <timeConstantDown>0.025</timeConstantDown>
 *     <commandSubTopic>command/motor_speed</commandSubTopic>
 *     <actuator_number>0</actuator_number>
 *     <reverse_thrust_fraction>0.4</reverse_thrust_fraction>
 *   </plugin>
 */

#include <cmath>
#include <mutex>
#include <string>

#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/msgs/actuators.pb.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Util.hh>
#include <gz/transport/Node.hh>

namespace airground
{

class ReverseThrust : public gz::sim::System,
                      public gz::sim::ISystemConfigure,
                      public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    gz::sim::Model model(_entity);
    const std::string modelName = model.Name(_ecm);

    auto getStr = [&](const char *_k, const std::string &_d) {
      return _sdf->HasElement(_k) ? _sdf->Get<std::string>(_k) : _d;
    };
    auto getDbl = [&](const char *_k, double _d) {
      return _sdf->HasElement(_k) ? _sdf->Get<double>(_k) : _d;
    };

    const std::string linkName = getStr("linkName", "");

    if (linkName.empty()) {
      gzerr << "[ReverseThrust] 缺少 linkName，插件不工作" << std::endl;
      return;
    }

    this->_linkEntity = model.LinkByName(_ecm, linkName);

    if (this->_linkEntity == gz::sim::kNullEntity) {
      gzerr << "[ReverseThrust] 找不到 link '" << linkName << "'" << std::endl;
      return;
    }

    this->_motorK = getDbl("motorConstant", 0.0);
    this->_maxRot = getDbl("maxRotVelocity", 1000.0);
    this->_timeUp = getDbl("timeConstantUp", 0.0125);
    this->_timeDown = getDbl("timeConstantDown", 0.025);
    this->_revFrac = getDbl("reverse_thrust_fraction", 0.4);
    this->_actIdx = static_cast<int>(getDbl("actuator_number", 0.0));

    // 指令中位与量程：CA_R_REV=15 时 PX4 把控制量映射到 500±500，
    // 所以“零推力”对应 500，而不是 0。
    this->_zero = getDbl("prop_zero_offset", 500.0);
    this->_span = getDbl("prop_span", (this->_zero > 0.0) ? (1000.0 - this->_zero) : 1000.0);

    if (this->_span <= 1e-9) {
      this->_span = 1000.0;
    }

    if (this->_revFrac < 0.0) {
      this->_revFrac = 0.0;
    }

    if (this->_revFrac > 1.0) {
      this->_revFrac = 1.0;
    }

    std::string sub = getStr("commandSubTopic", "command/motor_speed");

    if (sub.empty() || sub.front() != '/') {
      sub = "/" + modelName + "/" + sub;
    }

    if (!this->_node.Subscribe(sub, &ReverseThrust::OnCommand, this)) {
      gzerr << "[ReverseThrust] 无法订阅 " << sub << std::endl;
      return;
    }

    this->_ready = true;

    gzmsg << "[ReverseThrust] " << linkName << "  kT=" << this->_motorK
          << "  maxRot=" << this->_maxRot
          << "  反转推力比例=" << this->_revFrac
          << "  订阅=" << sub << "  idx=" << this->_actIdx << std::endl;
  }

  void PreUpdate(const gz::sim::UpdateInfo &_info,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (!this->_ready || _info.paused) {
      return;
    }

    const double dt = std::chrono::duration<double>(_info.dt).count();

    double cmd = 0.0;

    {
      std::lock_guard<std::mutex> lock(this->_mutex);
      cmd = this->_cmd;
    }

    const double desired = this->_maxRot * ((cmd - this->_zero) / this->_span);

    // 与原厂插件相同的转速一阶惯性，保证两边估算的是同一个 omega
    const double tau = (desired > this->_rotorVel) ? this->_timeUp : this->_timeDown;

    if (tau > 1e-9 && dt > 0.0) {
      this->_rotorVel = desired + (this->_rotorVel - desired) * std::exp(-dt / tau);

    } else {
      this->_rotorVel = desired;
    }

    // ω ≥ 0：正推力路径已由原厂插件（读 prop_speed）处理，这里什么都不做
    if (!(this->_rotorVel < 0.0)) {
      return;
    }

    // 负半轴：原厂插件收到的是 clamp 后的 0，完全不出力；
    // 这里直接补上反向推力 −revFrac·kT·ω²
    const double w2 = this->_rotorVel * this->_rotorVel;
    const double extra = -this->_motorK * w2 * this->_revFrac;

    if (!std::isfinite(extra)) {
      return;
    }

    const gz::math::Pose3d pose = gz::sim::worldPose(this->_linkEntity, _ecm);
    const gz::math::Vector3d fWorld =
        pose.Rot().RotateVector(gz::math::Vector3d(0.0, 0.0, extra));

    gz::sim::Link link(this->_linkEntity);
    link.AddWorldForce(_ecm, fWorld);
  }

private:
  void OnCommand(const gz::msgs::Actuators &_msg)
  {
    if (this->_actIdx < 0 || this->_actIdx >= _msg.velocity_size()) {
      return;
    }

    std::lock_guard<std::mutex> lock(this->_mutex);
    this->_cmd = _msg.velocity(this->_actIdx);
  }

  gz::transport::Node _node;
  std::mutex _mutex;

  gz::sim::Entity _linkEntity{gz::sim::kNullEntity};

  double _cmd{0.0};
  double _rotorVel{0.0};
  double _timeUp{0.0125};
  double _timeDown{0.025};
  double _maxRot{1000.0};
  double _motorK{0.0};
  double _revFrac{0.4};
  double _zero{500.0};
  double _span{500.0};
  int _actIdx{0};

  bool _ready{false};
};

}  // namespace airground

GZ_ADD_PLUGIN(airground::ReverseThrust,
              gz::sim::System,
              airground::ReverseThrust::ISystemConfigure,
              airground::ReverseThrust::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(airground::ReverseThrust, "airground::ReverseThrust")
