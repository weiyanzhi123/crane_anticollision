#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "crane_anticollision/anti_collision_core.hpp"
#include "crane_msg/JointPoseMsg.h"
#include "crane_msg/LoadCollisionAvoidCtrlLimit.h"

namespace {

crane_anticollision::AntiCollisionCore::Observation toObservation(const crane_msg::JointPoseMsg::ConstPtr& msg) {
  crane_anticollision::AntiCollisionCore::Observation obs;
  obs.valid = !msg->SensorMsgError;

  ros::Time stamp = msg->header.stamp;
  if (stamp.isZero()) stamp = ros::Time::now();
  obs.t_sec = stamp.toSec();

  obs.theta_deg = msg->PolarAngle;
  obs.r = msg->PolarRadius;
  obs.z = msg->PolarHeight;
  return obs;
}

void publishLimit(ros::Publisher& pub, bool having_risk, bool need_stop_now, double limit,
                  bool high_risk, bool low_risk) {
  crane_msg::LoadCollisionAvoidCtrlLimit msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.having_collision_risk = having_risk;
  msg.need_stop_now = need_stop_now;
  msg.flag_load_detected = false;

  msg.positive_limit.slew = limit;
  msg.positive_limit.luff = limit;
  msg.positive_limit.hoist = limit;
  msg.negative_limit.slew = -limit;
  msg.negative_limit.luff = -limit;
  msg.negative_limit.hoist = -limit;

  // 利用现有字段传递风险等级信息（用于可视化）
  // 使用load_pose_x传递风险等级：0=无风险, 1=低风险, 2=高风险
  if (high_risk) msg.load_pose_x = 2.0;
  else if (low_risk) msg.load_pose_x = 1.0;
  else msg.load_pose_x = 0.0;

  pub.publish(msg);
}

}  // namespace

class MultiCraneAntiCollisionRosNode {
public:
  MultiCraneAntiCollisionRosNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("crane_a_state_topic", crane_a_topic_, "/crane_teleop/crane_a_states");
    pnh_.param<std::string>("crane_b_state_topic", crane_b_topic_, "/crane_teleop/crane_b_states");
    pnh_.param<std::string>("crane_c_state_topic", crane_c_topic_, "/crane_teleop/crane_c_states");

    // 结构/策略/阈值参数 -> core config
    crane_anticollision::AntiCollisionCore::CraneParams params_a;
    crane_anticollision::AntiCollisionCore::CraneParams params_b;
    crane_anticollision::AntiCollisionCore::CraneParams params_c;
    loadCraneParams("crane_a", params_a);
    loadCraneParams("crane_b", params_b);
    loadCraneParams("crane_c", params_c);

    crane_anticollision::AntiCollisionCore::Config cfg_common;
    pnh_.param<double>("D_safe", cfg_common.D_safe, cfg_common.D_safe);
    pnh_.param<double>("D_warn", cfg_common.D_warn, cfg_common.D_warn);
    pnh_.param<double>("D_stop", cfg_common.D_stop, cfg_common.D_stop);
    pnh_.param<double>("D_low_risk", cfg_common.D_low_risk, cfg_common.D_low_risk);
    pnh_.param<double>("D_high_risk", cfg_common.D_high_risk, cfg_common.D_high_risk);

    pnh_.param<double>("prediction_time", cfg_common.prediction_time, cfg_common.prediction_time);
    pnh_.param<double>("prediction_step", cfg_common.prediction_step, cfg_common.prediction_step);
    double n_vel_double = static_cast<double>(cfg_common.velocity_window_size);
    pnh_.param<double>("velocity_window_size", n_vel_double, n_vel_double);
    if (n_vel_double < 2.0) n_vel_double = 2.0;
    cfg_common.velocity_window_size = static_cast<std::size_t>(n_vel_double);

    pnh_.param<double>("t_latency", cfg_common.t_latency, cfg_common.t_latency);
    pnh_.param<double>("t_brake", cfg_common.t_brake, cfg_common.t_brake);
    pnh_.param<double>("t_margin", cfg_common.t_margin, cfg_common.t_margin);

    pnh_.param<double>("max_theta_dot_deg_per_sec", cfg_common.max_theta_dot_deg_per_sec, cfg_common.max_theta_dot_deg_per_sec);
    pnh_.param<double>("max_r_dot_m_per_sec", cfg_common.max_r_dot_m_per_sec, cfg_common.max_r_dot_m_per_sec);
    pnh_.param<double>("max_z_dot_m_per_sec", cfg_common.max_z_dot_m_per_sec, cfg_common.max_z_dot_m_per_sec);

    double priority_a = cfg_common.crane_a_priority;
    double priority_b = cfg_common.crane_b_priority;
    double priority_c = 0.3;
    pnh_.param<double>("crane_a_priority", priority_a, priority_a);
    pnh_.param<double>("crane_b_priority", priority_b, priority_b);
    pnh_.param<double>("crane_c_priority", priority_c, priority_c);

    // 预测减速度（原来写死在节点内，这里参数化；默认值保持不变）
    pnh_.param<double>("prediction_decel_theta_deg_per_sec2",
                       cfg_common.prediction_decel_theta_deg_per_sec2,
                       cfg_common.prediction_decel_theta_deg_per_sec2);
    pnh_.param<double>("prediction_decel_r_m_per_sec2",
                       cfg_common.prediction_decel_r_m_per_sec2,
                       cfg_common.prediction_decel_r_m_per_sec2);
    pnh_.param<double>("prediction_decel_z_m_per_sec2",
                       cfg_common.prediction_decel_z_m_per_sec2,
                       cfg_common.prediction_decel_z_m_per_sec2);

    // timer rate（仍由 ROS 层控制）
    pnh_.param<double>("update_hz", update_hz_, 20.0);
    if (update_hz_ <= 0.0) update_hz_ = 20.0;

    crane_anticollision::AntiCollisionCore::Config cfg_ab = cfg_common;
    cfg_ab.crane_a = params_a;
    cfg_ab.crane_b = params_b;
    cfg_ab.crane_a_priority = priority_a;
    cfg_ab.crane_b_priority = priority_b;

    crane_anticollision::AntiCollisionCore::Config cfg_ac = cfg_common;
    cfg_ac.crane_a = params_a;
    cfg_ac.crane_b = params_c;
    cfg_ac.crane_a_priority = priority_a;
    cfg_ac.crane_b_priority = priority_c;

    crane_anticollision::AntiCollisionCore::Config cfg_bc = cfg_common;
    cfg_bc.crane_a = params_b;
    cfg_bc.crane_b = params_c;
    cfg_bc.crane_a_priority = priority_b;
    cfg_bc.crane_b_priority = priority_c;

    core_ab_.init(cfg_ab);
    core_ac_.init(cfg_ac);
    core_bc_.init(cfg_bc);

    sub_a_ = nh_.subscribe(crane_a_topic_, 10, &MultiCraneAntiCollisionRosNode::onCraneA, this);
    sub_b_ = nh_.subscribe(crane_b_topic_, 10, &MultiCraneAntiCollisionRosNode::onCraneB, this);
    sub_c_ = nh_.subscribe(crane_c_topic_, 10, &MultiCraneAntiCollisionRosNode::onCraneC, this);

    pub_limit_a_ = nh_.advertise<crane_msg::LoadCollisionAvoidCtrlLimit>("/planner/collision_avoid_info_a", 1);
    pub_limit_b_ = nh_.advertise<crane_msg::LoadCollisionAvoidCtrlLimit>("/planner/collision_avoid_info_b", 1);
    pub_limit_c_ = nh_.advertise<crane_msg::LoadCollisionAvoidCtrlLimit>("/planner/collision_avoid_info_c", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / update_hz_), &MultiCraneAntiCollisionRosNode::onTimer, this);

    ROS_INFO("Multi-Crane Anti-Collision Node initialized (core extracted, ROS wrapper mode)");
  }

private:
  void loadCraneParams(const std::string& prefix, crane_anticollision::AntiCollisionCore::CraneParams& params) {
    pnh_.param<double>(prefix + "/base_x", params.base_xyz.x(), params.base_xyz.x());
    pnh_.param<double>(prefix + "/base_y", params.base_xyz.y(), params.base_xyz.y());
    pnh_.param<double>(prefix + "/base_z", params.base_xyz.z(), params.base_xyz.z());
    pnh_.param<double>(prefix + "/mast_height", params.mast_height, params.mast_height);
    pnh_.param<double>(prefix + "/theta_offset_deg", params.theta_offset_deg, params.theta_offset_deg);
    pnh_.param<double>(prefix + "/arm_length", params.arm_length, params.arm_length);

    double arm_width = 1.0, arm_thickness = 0.5;
    pnh_.param<double>(prefix + "/arm_width", arm_width, arm_width);
    pnh_.param<double>(prefix + "/arm_thickness", arm_thickness, arm_thickness);
    params.arm_radius = 0.5 * std::max(arm_width, arm_thickness);

    pnh_.param<double>(prefix + "/rope_radius", params.rope_radius, params.rope_radius);
    pnh_.param<double>(prefix + "/hook_radius", params.hook_radius, params.hook_radius);
  }

  void onCraneA(const crane_msg::JointPoseMsg::ConstPtr& msg) {
    const auto obs = toObservation(msg);
    core_ab_.updateCraneA(obs);
    core_ac_.updateCraneA(obs);
  }

  void onCraneB(const crane_msg::JointPoseMsg::ConstPtr& msg) {
    const auto obs = toObservation(msg);
    core_ab_.updateCraneB(obs);
    core_bc_.updateCraneA(obs);
  }

  void onCraneC(const crane_msg::JointPoseMsg::ConstPtr& msg) {
    const auto obs = toObservation(msg);
    core_ac_.updateCraneB(obs);
    core_bc_.updateCraneB(obs);
  }

  void onTimer(const ros::TimerEvent&) {
    const double now = ros::Time::now().toSec();
    const auto res_ab = core_ab_.step(now);
    const auto res_ac = core_ac_.step(now);
    const auto res_bc = core_bc_.step(now);

    auto mergeDecision = [](const crane_anticollision::AntiCollisionCore::PerCraneDecision& d1,
                            const crane_anticollision::AntiCollisionCore::PerCraneDecision& d2) {
      crane_anticollision::AntiCollisionCore::PerCraneDecision out;
      out.having_risk = d1.having_risk || d2.having_risk;
      out.high_risk = d1.high_risk || d2.high_risk;
      out.low_risk = d1.low_risk || d2.low_risk;
      out.force_stop = d1.force_stop || d2.force_stop;
      out.need_stop_now = d1.need_stop_now || d2.need_stop_now || out.force_stop;
      out.limit = std::min(d1.limit, d2.limit);
      if (out.force_stop) out.limit = 0.0;
      return out;
    };

    const auto dec_a = mergeDecision(res_ab.a, res_ac.a);
    const auto dec_b = mergeDecision(res_ab.b, res_bc.a);
    const auto dec_c = mergeDecision(res_ac.b, res_bc.b);

    publishLimit(pub_limit_a_, dec_a.having_risk, dec_a.need_stop_now, dec_a.limit, dec_a.high_risk, dec_a.low_risk);
    publishLimit(pub_limit_b_, dec_b.having_risk, dec_b.need_stop_now, dec_b.limit, dec_b.high_risk, dec_b.low_risk);
    publishLimit(pub_limit_c_, dec_c.having_risk, dec_c.need_stop_now, dec_c.limit, dec_c.high_risk, dec_c.low_risk);

    static int count = 0;
    if (++count % 20 == 0) {
      const double d_min = std::min(res_ab.d_min, std::min(res_ac.d_min, res_bc.d_min));
      const double d_min_pred = std::min(res_ab.d_min_pred, std::min(res_ac.d_min_pred, res_bc.d_min_pred));
      const double ttc = std::min(res_ab.ttc, std::min(res_ac.ttc, res_bc.ttc));
      ROS_INFO_THROTTLE(1.0,
                        "Anti-Collision(3): d_min=%.2f, d_min_pred=%.2f, stop_a=%d, stop_b=%d, stop_c=%d, TTC=%.2f",
                        d_min, d_min_pred,
                        dec_a.need_stop_now ? 1 : 0, dec_b.need_stop_now ? 1 : 0, dec_c.need_stop_now ? 1 : 0,
                        std::isinf(ttc) ? -1.0 : ttc);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_a_;
  ros::Subscriber sub_b_;
  ros::Subscriber sub_c_;
  ros::Publisher pub_limit_a_;
  ros::Publisher pub_limit_b_;
  ros::Publisher pub_limit_c_;
  ros::Timer timer_;

  std::string crane_a_topic_;
  std::string crane_b_topic_;
  std::string crane_c_topic_;
  double update_hz_{20.0};

  crane_anticollision::AntiCollisionCore core_ab_;
  crane_anticollision::AntiCollisionCore core_ac_;
  crane_anticollision::AntiCollisionCore core_bc_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_crane_anticollision_node");
  MultiCraneAntiCollisionRosNode node;
  ros::spin();
  return 0;
}
