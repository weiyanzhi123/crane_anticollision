#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <string>

#include "crane_anticollision/anti_collision_core.hpp"
#include "crane_msg/JointPoseMsg.h"
#include "crane_msg/LoadCollisionAvoidCtrlLimit.h"
#include "crane_anticollision/CraneGearLimit.h"

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

crane_anticollision::AntiCollisionCore::Observation toObservation(const crane_msg::JointPoseMsg::ConstPtr& msg) {
  crane_anticollision::AntiCollisionCore::Observation obs;
  obs.valid = !msg->SensorMsgError;

  ros::Time stamp = msg->header.stamp;
  if (stamp.isZero()) stamp = ros::Time::now();
  obs.t_sec = stamp.toSec();

  // Core uses radians internally; convert from message degrees.
  obs.theta_rad = msg->PolarAngle * kDeg2Rad;
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

void publishGearLimit(ros::Publisher& pub, bool having_risk, bool need_stop_now,
                      bool high_risk, bool low_risk,
                      int gear_theta, int gear_r, int gear_z) {
  crane_anticollision::CraneGearLimit msg;
  msg.header.stamp = ros::Time::now();
  msg.header.frame_id = "world";
  msg.having_risk = having_risk;
  msg.need_stop_now = need_stop_now;
  msg.high_risk = high_risk;
  msg.low_risk = low_risk;
  msg.gear_theta_limit = gear_theta;
  msg.gear_r_limit = gear_r;
  msg.gear_z_limit = gear_z;
  pub.publish(msg);
}

}  // namespace

class MultiCraneAntiCollisionRosNode {
public:
  MultiCraneAntiCollisionRosNode() : nh_(), pnh_("~") {
    pnh_.param<std::string>("crane_a_state_topic", crane_a_topic_, "/crane_teleop/crane_a_states");
    pnh_.param<std::string>("crane_b_state_topic", crane_b_topic_, "/crane_teleop/crane_b_states");
    // Only A/B in two-crane mode

    // 结构/策略/阈值参数 -> core config
    crane_anticollision::AntiCollisionCore::CraneParams params_a;
    crane_anticollision::AntiCollisionCore::CraneParams params_b;
    loadCraneParams("crane_a", params_a);
    loadCraneParams("crane_b", params_b);

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

    pnh_.param<double>("max_theta_dot_rad_per_sec", cfg_common.max_theta_dot_rad_per_sec, cfg_common.max_theta_dot_rad_per_sec);
    pnh_.param<double>("max_r_dot_m_per_sec", cfg_common.max_r_dot_m_per_sec, cfg_common.max_r_dot_m_per_sec);
    pnh_.param<double>("max_z_dot_m_per_sec", cfg_common.max_z_dot_m_per_sec, cfg_common.max_z_dot_m_per_sec);

    pnh_.param<double>("speed_weight_slew", cfg_common.speed_weight_slew, cfg_common.speed_weight_slew);
    pnh_.param<double>("speed_weight_trolley", cfg_common.speed_weight_trolley, cfg_common.speed_weight_trolley);
    pnh_.param<double>("speed_weight_hoist", cfg_common.speed_weight_hoist, cfg_common.speed_weight_hoist);

    pnh_.param<bool>("command_fusion_enabled", cfg_common.command_fusion_enabled, cfg_common.command_fusion_enabled);
    pnh_.param<double>("command_fusion_alpha", cfg_common.command_fusion_alpha, cfg_common.command_fusion_alpha);
    pnh_.param<double>("command_fusion_lead_time", cfg_common.command_fusion_lead_time, cfg_common.command_fusion_lead_time);

    pnh_.param<bool>("latency_compensation_enabled", cfg_common.latency_compensation_enabled,
                     cfg_common.latency_compensation_enabled);
    pnh_.param<double>("latency_compensation_time", cfg_common.latency_compensation_time,
                       cfg_common.latency_compensation_time);
    pnh_.param<bool>("braking_distance_guard_enabled", cfg_common.braking_distance_guard_enabled,
                     cfg_common.braking_distance_guard_enabled);
    pnh_.param<double>("braking_distance_margin", cfg_common.braking_distance_margin,
                       cfg_common.braking_distance_margin);

    int gear_count_theta = 4;
    int gear_count_r = 3;
    int gear_count_z = 5;
    pnh_.param<int>("gear_count_theta", gear_count_theta, gear_count_theta);
    pnh_.param<int>("gear_count_r", gear_count_r, gear_count_r);
    pnh_.param<int>("gear_count_z", gear_count_z, gear_count_z);
    if (gear_count_theta < 1) gear_count_theta = 1;
    if (gear_count_r < 1) gear_count_r = 1;
    if (gear_count_z < 1) gear_count_z = 1;

    double priority_a = cfg_common.crane_a_priority;
    double priority_b = cfg_common.crane_b_priority;
    pnh_.param<double>("crane_a_priority", priority_a, priority_a);
    pnh_.param<double>("crane_b_priority", priority_b, priority_b);

    // 预测减速度（原来写死在节点内，这里参数化；默认值保持不变）
    pnh_.param<double>("prediction_decel_theta_rad_per_sec2",
                       cfg_common.prediction_decel_theta_rad_per_sec2,
                       cfg_common.prediction_decel_theta_rad_per_sec2);
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

    core_ab_.init(cfg_ab);

    sub_a_ = nh_.subscribe(crane_a_topic_, 10, &MultiCraneAntiCollisionRosNode::onCraneA, this);
    sub_b_ = nh_.subscribe(crane_b_topic_, 10, &MultiCraneAntiCollisionRosNode::onCraneB, this);

    pub_limit_a_ = nh_.advertise<crane_msg::LoadCollisionAvoidCtrlLimit>("/planner/collision_avoid_info_a", 1);
    pub_limit_b_ = nh_.advertise<crane_msg::LoadCollisionAvoidCtrlLimit>("/planner/collision_avoid_info_b", 1);
    pub_gear_limit_a_ = nh_.advertise<crane_anticollision::CraneGearLimit>("/planner/gear_limit_a", 1);
    pub_gear_limit_b_ = nh_.advertise<crane_anticollision::CraneGearLimit>("/planner/gear_limit_b", 1);

    timer_ = nh_.createTimer(ros::Duration(1.0 / update_hz_), &MultiCraneAntiCollisionRosNode::onTimer, this);

    ROS_INFO("Multi-Crane Anti-Collision Node initialized (core extracted, ROS wrapper mode)");

    gear_count_theta_ = gear_count_theta;
    gear_count_r_ = gear_count_r;
    gear_count_z_ = gear_count_z;
  }

private:
  void loadCraneParams(const std::string& prefix, crane_anticollision::AntiCollisionCore::CraneParams& params) {
    pnh_.param<double>(prefix + "/base_x", params.base_xyz.x(), params.base_xyz.x());
    pnh_.param<double>(prefix + "/base_y", params.base_xyz.y(), params.base_xyz.y());
    pnh_.param<double>(prefix + "/base_z", params.base_xyz.z(), params.base_xyz.z());
    pnh_.param<double>(prefix + "/mast_height", params.mast_height, params.mast_height);
    pnh_.param<double>(prefix + "/theta_offset_rad", params.theta_offset_rad, params.theta_offset_rad);
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
  }

  void onCraneB(const crane_msg::JointPoseMsg::ConstPtr& msg) {
    const auto obs = toObservation(msg);
    core_ab_.updateCraneB(obs);
  }

  void onTimer(const ros::TimerEvent&) {
    const double now = ros::Time::now().toSec();
    const auto res_ab = core_ab_.step(now);
    publishLimit(pub_limit_a_, res_ab.a.having_risk, res_ab.a.need_stop_now, res_ab.a.limit, res_ab.a.high_risk, res_ab.a.low_risk);
    publishLimit(pub_limit_b_, res_ab.b.having_risk, res_ab.b.need_stop_now, res_ab.b.limit, res_ab.b.high_risk, res_ab.b.low_risk);

    auto computeGearLimit = [](double limit, bool force_stop, int max_gear) -> int {
      if (force_stop || max_gear <= 0) return 0;
      if (limit >= 0.999) return max_gear;
      if (limit <= 0.0) return 0;
      int gear = static_cast<int>(std::floor(limit * static_cast<double>(max_gear)));
      if (gear < 1) gear = 1;
      if (gear > max_gear) gear = max_gear;
      return gear;
    };

    const int gear_theta_a = computeGearLimit(res_ab.a.limit, res_ab.a.force_stop, gear_count_theta_);
    const int gear_r_a = computeGearLimit(res_ab.a.limit, res_ab.a.force_stop, gear_count_r_);
    const int gear_z_a = computeGearLimit(res_ab.a.limit, res_ab.a.force_stop, gear_count_z_);
    const int gear_theta_b = computeGearLimit(res_ab.b.limit, res_ab.b.force_stop, gear_count_theta_);
    const int gear_r_b = computeGearLimit(res_ab.b.limit, res_ab.b.force_stop, gear_count_r_);
    const int gear_z_b = computeGearLimit(res_ab.b.limit, res_ab.b.force_stop, gear_count_z_);

    publishGearLimit(pub_gear_limit_a_, res_ab.a.having_risk, res_ab.a.need_stop_now, res_ab.a.high_risk, res_ab.a.low_risk,
                     gear_theta_a, gear_r_a, gear_z_a);
    publishGearLimit(pub_gear_limit_b_, res_ab.b.having_risk, res_ab.b.need_stop_now, res_ab.b.high_risk, res_ab.b.low_risk,
                     gear_theta_b, gear_r_b, gear_z_b);

    static int count = 0;
    if (++count % 20 == 0) {
      const double d_min = res_ab.d_min;
      const double d_min_pred = res_ab.d_min_pred;
      const double ttc = res_ab.ttc;
      ROS_INFO_THROTTLE(1.0,
                        "Anti-Collision(2): d_min=%.2f, d_min_pred=%.2f, stop_a=%d, stop_b=%d, TTC=%.2f",
                        d_min, d_min_pred,
                        res_ab.a.need_stop_now ? 1 : 0, res_ab.b.need_stop_now ? 1 : 0,
                        std::isinf(ttc) ? -1.0 : ttc);
    }
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;
  ros::Subscriber sub_a_;
  ros::Subscriber sub_b_;
  ros::Publisher pub_limit_a_;
  ros::Publisher pub_limit_b_;
  ros::Publisher pub_gear_limit_a_;
  ros::Publisher pub_gear_limit_b_;
  ros::Timer timer_;

  std::string crane_a_topic_;
  std::string crane_b_topic_;
  double update_hz_{20.0};
  int gear_count_theta_{4};
  int gear_count_r_{3};
  int gear_count_z_{5};

  crane_anticollision::AntiCollisionCore core_ab_;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_crane_anticollision_node");
  MultiCraneAntiCollisionRosNode node;
  ros::spin();
  return 0;
}
