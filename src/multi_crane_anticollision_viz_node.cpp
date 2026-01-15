#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <limits>
#include <string>

#include <Eigen/Dense>

#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <visualization_msgs/MarkerArray.h>

#include "crane_msg/JointPoseMsg.h"
#include "crane_msg/LoadCollisionAvoidCtrlLimit.h"

namespace {

constexpr double kDeg2Rad = M_PI / 180.0;

double clampDouble(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

double pointSegmentDistance(const Eigen::Vector3d& p,
                            const Eigen::Vector3d& a,
                            const Eigen::Vector3d& b) {
  const Eigen::Vector3d ab = b - a;
  const double ab2 = ab.squaredNorm();
  if (ab2 <= 1e-12) return (p - a).norm();
  const double t = clampDouble((p - a).dot(ab) / ab2, 0.0, 1.0);
  const Eigen::Vector3d q = a + t * ab;
  return (p - q).norm();
}

double segmentSegmentDistance(const Eigen::Vector3d& p1,
                              const Eigen::Vector3d& q1,
                              const Eigen::Vector3d& p2,
                              const Eigen::Vector3d& q2) {
  const Eigen::Vector3d d1 = q1 - p1;
  const Eigen::Vector3d d2 = q2 - p2;
  const Eigen::Vector3d r = p1 - p2;
  const double a = d1.dot(d1);
  const double e = d2.dot(d2);
  const double f = d2.dot(r);

  double s = 0.0, t = 0.0;

  if (a <= 1e-12 && e <= 1e-12) {
    return (p1 - p2).norm();
  }
  if (a <= 1e-12) {
    s = 0.0;
    t = clampDouble(f / e, 0.0, 1.0);
  } else {
    const double c = d1.dot(r);
    if (e <= 1e-12) {
      t = 0.0;
      s = clampDouble(-c / a, 0.0, 1.0);
    } else {
      const double b = d1.dot(d2);
      const double denom = a * e - b * b;
      if (denom != 0.0) {
        s = clampDouble((b * f - c * e) / denom, 0.0, 1.0);
      } else {
        s = 0.0;
      }
      const double tnom = b * s + f;
      if (tnom < 0.0) {
        t = 0.0;
        s = clampDouble(-c / a, 0.0, 1.0);
      } else if (tnom > e) {
        t = 1.0;
        s = clampDouble((b - c) / a, 0.0, 1.0);
      } else {
        t = tnom / e;
      }
    }
  }

  const Eigen::Vector3d c1 = p1 + s * d1;
  const Eigen::Vector3d c2 = p2 + t * d2;
  return (c1 - c2).norm();
}

struct CraneParams {
  Eigen::Vector3d base_xyz{0.0, 0.0, 0.0};
  double theta_offset_deg{0.0};
  double mast_height{45.0};     // 塔身高度（m）
  double mast_radius{1.2};      // 塔身半径（m）
  double arm_length{50.0};
  double arm_width{1.0};        // 大臂宽（m）
  double arm_thickness{0.5};    // 大臂厚（m）
  // 平衡臂/后臂（反方向延伸），更像真实塔吊
  double counter_arm_length{15.0};
  double counter_arm_width{1.0};
  double counter_arm_thickness{0.5};
  double trolley_length{1.5};   // 小车尺寸（m）
  double trolley_width{1.0};
  double trolley_height{0.8};
  double rope_radius{0.05};
  double hook_radius{0.35};
};

struct CraneState {
  bool valid{false};
  double theta_deg{0.0};
  double r{0.0};
  double z{0.0};
  double theta_vel_deg_per_sec{0.0};
  double r_vel_m_per_sec{0.0};
  double z_vel_m_per_sec{0.0};
};

struct DistResult {
  double d_min{std::numeric_limits<double>::infinity()};
  std::string pair;
};

}  // namespace

class MultiCraneAntiCollisionVizNode {
public:
  MultiCraneAntiCollisionVizNode() : nh_(), pnh_("~") {
    // Topics
    pnh_.param<std::string>("crane_a_state_topic", crane_a_state_topic_,
                            std::string("/craneA/crane_control/crane_states"));
    pnh_.param<std::string>("crane_b_state_topic", crane_b_state_topic_,
                            std::string("/craneB/crane_control/crane_states"));
    pnh_.param<std::string>("crane_c_state_topic", crane_c_state_topic_,
                            std::string("/craneC/crane_control/crane_states"));
    pnh_.param<std::string>("gear_limit_topic", gear_limit_topic_,
                            std::string("/planner/collision_avoid_info"));
    pnh_.param<std::string>("marker_topic", marker_topic_,
                            std::string("markers"));  // private by default

    // Static viz mode (no state topics required)
    // use_state_topics_ is kept as a global default, but A/B can override independently.
    pnh_.param<bool>("use_state_topics", use_state_topics_, true);
    pnh_.param<bool>("use_gear_limit_topic", use_gear_limit_topic_, true);
    pnh_.param<double>("static_crane_a_theta_deg", static_a_theta_deg_, 0.0);
    pnh_.param<double>("static_crane_a_r", static_a_r_, 10.0);
    pnh_.param<double>("static_crane_a_z", static_a_z_, 10.0);
    pnh_.param<double>("static_crane_b_theta_deg", static_b_theta_deg_, 180.0);
    pnh_.param<double>("static_crane_b_r", static_b_r_, 10.0);
    pnh_.param<double>("static_crane_b_z", static_b_z_, 10.0);
    pnh_.param<double>("static_crane_c_theta_deg", static_c_theta_deg_, 90.0);
    pnh_.param<double>("static_crane_c_r", static_c_r_, 12.0);
    pnh_.param<double>("static_crane_c_z", static_c_z_, 10.0);

    // Per-crane override: allow only the "main crane" to be controlled / subscribed.
    pnh_.param<bool>("crane_a/use_state_topic", use_state_topic_a_, use_state_topics_);
    pnh_.param<bool>("crane_b/use_state_topic", use_state_topic_b_, use_state_topics_);
    pnh_.param<bool>("crane_c/use_state_topic", use_state_topic_c_, use_state_topics_);

    // Frames
    pnh_.param<std::string>("world_frame", world_frame_, std::string("world"));
    pnh_.param<std::string>("crane_a_frame_prefix", crane_a_prefix_, std::string("crane_a"));
    pnh_.param<std::string>("crane_b_frame_prefix", crane_b_prefix_, std::string("crane_b"));
    pnh_.param<std::string>("crane_c_frame_prefix", crane_c_prefix_, std::string("crane_c"));

    // Rate
    pnh_.param<double>("viz_hz", viz_hz_, 20.0);

    // Params for drawing
    loadCraneParams("crane_a", crane_a_params_);
    loadCraneParams("crane_b", crane_b_params_);
    loadCraneParams("crane_c", crane_c_params_);

    // Always seed with static pose so visualization appears immediately.
    a_.valid = true;
    a_.theta_deg = static_a_theta_deg_;
    a_.r = static_a_r_;
    a_.z = static_a_z_;

    b_.valid = true;
    b_.theta_deg = static_b_theta_deg_;
    b_.r = static_b_r_;
    b_.z = static_b_z_;

    c_.valid = true;
    c_.theta_deg = static_c_theta_deg_;
    c_.r = static_c_r_;
    c_.z = static_c_z_;

    // Subscribe per crane if enabled.
    if (use_state_topic_a_) {
      sub_a_ = nh_.subscribe(crane_a_state_topic_, 10, &MultiCraneAntiCollisionVizNode::onCraneA, this);
    }
    if (use_state_topic_b_) {
      sub_b_ = nh_.subscribe(crane_b_state_topic_, 10, &MultiCraneAntiCollisionVizNode::onCraneB, this);
    }
    if (use_state_topic_c_) {
      sub_c_ = nh_.subscribe(crane_c_state_topic_, 10, &MultiCraneAntiCollisionVizNode::onCraneC, this);
    }

    if (use_gear_limit_topic_) {
      // 订阅两路限制信息（A和B分别）
      sub_limit_a_ = nh_.subscribe("/planner/collision_avoid_info_a", 10, &MultiCraneAntiCollisionVizNode::onLimitA, this);
      sub_limit_b_ = nh_.subscribe("/planner/collision_avoid_info_b", 10, &MultiCraneAntiCollisionVizNode::onLimitB, this);
      sub_limit_c_ = nh_.subscribe("/planner/collision_avoid_info_c", 10, &MultiCraneAntiCollisionVizNode::onLimitC, this);
      // 也保留旧的单一topic订阅（向后兼容）
      sub_limit_ = nh_.subscribe(gear_limit_topic_, 10, &MultiCraneAntiCollisionVizNode::onLimit, this);
    } else {
      having_risk_ = false;
      need_stop_ = false;
      stop_a_ = false;
      stop_b_ = false;
      stop_c_ = false;
    }
    
    // 红色闪烁持续时间（秒）
    pnh_.param<double>("stop_red_duration", stop_red_duration_, 1.0);
    
    // 闪烁频率参数
    pnh_.param<double>("low_risk_flash_freq", low_risk_flash_freq_, 1.0);   // 低风险闪烁频率（Hz）
    pnh_.param<double>("high_risk_flash_freq", high_risk_flash_freq_, 5.0);  // 高风险闪烁频率（Hz）

    // 运动方向箭头（大臂/小车/吊钩 各自运动时显示；各自静止时不显示）
    pnh_.param<bool>("motion_arrows/enabled", motion_arrows_enabled_, true);
    pnh_.param<double>("motion_arrows/theta_vel_threshold_deg_per_sec", arrow_theta_vel_thresh_deg_per_sec_, 0.05);
    pnh_.param<double>("motion_arrows/r_vel_threshold_m_per_sec", arrow_r_vel_thresh_m_per_sec_, 0.01);
    pnh_.param<double>("motion_arrows/z_vel_threshold_m_per_sec", arrow_z_vel_thresh_m_per_sec_, 0.01);
    pnh_.param<double>("motion_arrows/arm_arrow_length_m", arm_arrow_length_m_, 4.0);
    pnh_.param<double>("motion_arrows/trolley_arrow_length_m", trolley_arrow_length_m_, 3.0);
    pnh_.param<double>("motion_arrows/hook_arrow_length_m", hook_arrow_length_m_, 3.0);
    pnh_.param<double>("motion_arrows/shaft_diameter_m", arrow_shaft_diameter_m_, 0.25);
    pnh_.param<double>("motion_arrows/head_diameter_m", arrow_head_diameter_m_, 0.45);
    pnh_.param<double>("motion_arrows/head_length_m", arrow_head_length_m_, 0.7);

    pub_markers_ = pnh_.advertise<visualization_msgs::MarkerArray>(marker_topic_, 1);

    const double period = (viz_hz_ > 0.0) ? (1.0 / viz_hz_) : 0.05;
    timer_ = nh_.createTimer(ros::Duration(period), &MultiCraneAntiCollisionVizNode::onTimer, this);
  }

private:
  void loadCraneParams(const std::string& ns, CraneParams& p) {
    pnh_.param<double>(ns + "/base_x", p.base_xyz.x(), p.base_xyz.x());
    pnh_.param<double>(ns + "/base_y", p.base_xyz.y(), p.base_xyz.y());
    pnh_.param<double>(ns + "/base_z", p.base_xyz.z(), p.base_xyz.z());
    pnh_.param<double>(ns + "/theta_offset_deg", p.theta_offset_deg, p.theta_offset_deg);
    pnh_.param<double>(ns + "/mast_height", p.mast_height, p.mast_height);
    pnh_.param<double>(ns + "/mast_radius", p.mast_radius, p.mast_radius);
    pnh_.param<double>(ns + "/arm_length", p.arm_length, p.arm_length);
    pnh_.param<double>(ns + "/arm_width", p.arm_width, p.arm_width);
    pnh_.param<double>(ns + "/arm_thickness", p.arm_thickness, p.arm_thickness);
    pnh_.param<double>(ns + "/counter_arm_length", p.counter_arm_length, p.counter_arm_length);
    pnh_.param<double>(ns + "/counter_arm_width", p.counter_arm_width, p.counter_arm_width);
    pnh_.param<double>(ns + "/counter_arm_thickness", p.counter_arm_thickness, p.counter_arm_thickness);
    pnh_.param<double>(ns + "/trolley_length", p.trolley_length, p.trolley_length);
    pnh_.param<double>(ns + "/trolley_width", p.trolley_width, p.trolley_width);
    pnh_.param<double>(ns + "/trolley_height", p.trolley_height, p.trolley_height);
    pnh_.param<double>(ns + "/rope_radius", p.rope_radius, p.rope_radius);
    pnh_.param<double>(ns + "/hook_radius", p.hook_radius, p.hook_radius);
  }

  void onCraneA(const crane_msg::JointPoseMsg::ConstPtr& msg) {
    a_.valid = !msg->SensorMsgError;
    a_.theta_deg = msg->PolarAngle + crane_a_params_.theta_offset_deg;
    a_.r = msg->PolarRadius;
    a_.z = msg->PolarHeight;
    a_.theta_vel_deg_per_sec = msg->PolarAngleVel;
    a_.r_vel_m_per_sec = msg->PolarRadiusVel;
    a_.z_vel_m_per_sec = msg->PolarHeightVel;
  }

  void onCraneB(const crane_msg::JointPoseMsg::ConstPtr& msg) {
    b_.valid = !msg->SensorMsgError;
    b_.theta_deg = msg->PolarAngle + crane_b_params_.theta_offset_deg;
    b_.r = msg->PolarRadius;
    b_.z = msg->PolarHeight;
    b_.theta_vel_deg_per_sec = msg->PolarAngleVel;
    b_.r_vel_m_per_sec = msg->PolarRadiusVel;
    b_.z_vel_m_per_sec = msg->PolarHeightVel;
  }

  void onCraneC(const crane_msg::JointPoseMsg::ConstPtr& msg) {
    c_.valid = !msg->SensorMsgError;
    c_.theta_deg = msg->PolarAngle + crane_c_params_.theta_offset_deg;
    c_.r = msg->PolarRadius;
    c_.z = msg->PolarHeight;
    c_.theta_vel_deg_per_sec = msg->PolarAngleVel;
    c_.r_vel_m_per_sec = msg->PolarRadiusVel;
    c_.z_vel_m_per_sec = msg->PolarHeightVel;
  }

  void onLimit(const crane_msg::LoadCollisionAvoidCtrlLimit::ConstPtr& msg) {
    having_risk_ = msg->having_collision_risk;
    need_stop_ = msg->need_stop_now;
    // 向后兼容：如果只订阅单一topic，默认应用到A
    // 只有当真正需要刹停时才触发红色闪烁
    if (msg->need_stop_now && !stop_a_) {
      stop_a_ = true;
      stop_time_a_ = ros::Time::now();
    } else if (!msg->need_stop_now && stop_a_) {
      stop_a_ = false;
      stop_time_a_ = ros::Time();  // 重置为无效时间
    }
  }

  void onLimitA(const crane_msg::LoadCollisionAvoidCtrlLimit::ConstPtr& msg) {
    having_risk_ = msg->having_collision_risk || having_risk_;  // 任一有风险就标记
    risk_level_a_ = msg->load_pose_x;  // 从load_pose_x获取风险等级：0=无风险, 1=低风险, 2=高风险
    
    if (msg->need_stop_now && !stop_a_) {
      // 刚触发刹停（高风险）
      stop_a_ = true;
      stop_time_a_ = ros::Time::now();
    } else if (!msg->need_stop_now && stop_a_) {
      // 停止后恢复
      stop_a_ = false;
      stop_time_a_ = ros::Time();  // 重置为无效时间
    }
  }

  void onLimitB(const crane_msg::LoadCollisionAvoidCtrlLimit::ConstPtr& msg) {
    having_risk_ = msg->having_collision_risk || having_risk_;  // 任一有风险就标记
    risk_level_b_ = msg->load_pose_x;  // 从load_pose_x获取风险等级：0=无风险, 1=低风险, 2=高风险
    
    if (msg->need_stop_now && !stop_b_) {
      // 刚触发刹停（高风险）
      stop_b_ = true;
      stop_time_b_ = ros::Time::now();
    } else if (!msg->need_stop_now && stop_b_) {
      // 停止后恢复
      stop_b_ = false;
      stop_time_b_ = ros::Time();  // 重置为无效时间
    }
  }

  void onLimitC(const crane_msg::LoadCollisionAvoidCtrlLimit::ConstPtr& msg) {
    having_risk_ = msg->having_collision_risk || having_risk_;  // 任一有风险就标记
    risk_level_c_ = msg->load_pose_x;  // 从load_pose_x获取风险等级：0=无风险, 1=低风险, 2=高风险

    if (msg->need_stop_now && !stop_c_) {
      stop_c_ = true;
      stop_time_c_ = ros::Time::now();
    } else if (!msg->need_stop_now && stop_c_) {
      stop_c_ = false;
      stop_time_c_ = ros::Time();
    }
  }

  DistResult computeDistance(const Eigen::Vector3d& ca, const Eigen::Vector3d& ta,
                             const Eigen::Vector3d& cb, const Eigen::Vector3d& tb,
                             const Eigen::Vector3d& pa_rope, const Eigen::Vector3d& ha,
                             const Eigen::Vector3d& pb_rope, const Eigen::Vector3d& hb) const {
    DistResult out;

    // Arm-arm
    {
      // 这里把“臂的宽度/厚度”简化为胶囊半径：取较大的半宽作为安全半径（可视化用途）
      const double ra = 0.5 * std::max(crane_a_params_.arm_width, crane_a_params_.arm_thickness);
      const double rb = 0.5 * std::max(crane_b_params_.arm_width, crane_b_params_.arm_thickness);
      const double d = segmentSegmentDistance(ca, ta, cb, tb) - (ra + rb);
      if (d < out.d_min) { out.d_min = d; out.pair = "arm-arm"; }
    }
    // A arm vs B rope
    {
      const double ra = 0.5 * std::max(crane_a_params_.arm_width, crane_a_params_.arm_thickness);
      const double d = segmentSegmentDistance(ca, ta, pb_rope, hb) - (ra + crane_b_params_.rope_radius);
      if (d < out.d_min) { out.d_min = d; out.pair = "A_arm-B_rope"; }
    }
    // B arm vs A rope
    {
      const double rb = 0.5 * std::max(crane_b_params_.arm_width, crane_b_params_.arm_thickness);
      const double d = segmentSegmentDistance(cb, tb, pa_rope, ha) - (rb + crane_a_params_.rope_radius);
      if (d < out.d_min) { out.d_min = d; out.pair = "B_arm-A_rope"; }
    }
    // A arm vs B hook
    {
      const double ra = 0.5 * std::max(crane_a_params_.arm_width, crane_a_params_.arm_thickness);
      const double d = pointSegmentDistance(hb, ca, ta) - (ra + crane_b_params_.hook_radius);
      if (d < out.d_min) { out.d_min = d; out.pair = "A_arm-B_hook"; }
    }
    // B arm vs A hook
    {
      const double rb = 0.5 * std::max(crane_b_params_.arm_width, crane_b_params_.arm_thickness);
      const double d = pointSegmentDistance(ha, cb, tb) - (rb + crane_a_params_.hook_radius);
      if (d < out.d_min) { out.d_min = d; out.pair = "B_arm-A_hook"; }
    }
    return out;
  }

  void publishTF(const std::string& child, const Eigen::Vector3d& t) {
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros::Time::now();
    tf.header.frame_id = world_frame_;
    tf.child_frame_id = child;
    tf.transform.translation.x = t.x();
    tf.transform.translation.y = t.y();
    tf.transform.translation.z = t.z();
    tf.transform.rotation.x = 0.0;
    tf.transform.rotation.y = 0.0;
    tf.transform.rotation.z = 0.0;
    tf.transform.rotation.w = 1.0;
    tf_br_.sendTransform(tf);
  }

  static geometry_msgs::Quaternion yawToQuat(double yaw_rad) {
    geometry_msgs::Quaternion q;
    q.x = 0.0;
    q.y = 0.0;
    q.z = std::sin(yaw_rad * 0.5);
    q.w = std::cos(yaw_rad * 0.5);
    return q;
  }

  visualization_msgs::Marker makeLineMarker(int id, const std::string& ns,
                                            const Eigen::Vector3d& p0,
                                            const Eigen::Vector3d& p1,
                                            double width,
                                            float r, float g, float b, float a) const {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::LINE_LIST;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    m.scale.x = width;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    geometry_msgs::Point gp0; gp0.x = p0.x(); gp0.y = p0.y(); gp0.z = p0.z();
    geometry_msgs::Point gp1; gp1.x = p1.x(); gp1.y = p1.y(); gp1.z = p1.z();
    m.points.push_back(gp0);
    m.points.push_back(gp1);
    return m;
  }

  visualization_msgs::Marker makeCubeMarker(int id, const std::string& ns,
                                            const Eigen::Vector3d& p,
                                            double yaw_rad,
                                            double sx, double sy, double sz,
                                            float r, float g, float b, float a) const {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::CUBE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation = yawToQuat(yaw_rad);
    m.scale.x = sx;
    m.scale.y = sy;
    m.scale.z = sz;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    return m;
  }

  visualization_msgs::Marker makeCylinderMarker(int id, const std::string& ns,
                                                const Eigen::Vector3d& p,
                                                double radius, double height,
                                                float r, float g, float b, float a) const {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::CYLINDER;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = radius * 2.0;
    m.scale.y = radius * 2.0;
    m.scale.z = height;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    return m;
  }

  visualization_msgs::Marker makeSphereMarker(int id, const std::string& ns,
                                              const Eigen::Vector3d& p,
                                              double radius,
                                              float r, float g, float b, float a) const {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::SPHERE;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    m.scale.x = radius * 2.0;
    m.scale.y = radius * 2.0;
    m.scale.z = radius * 2.0;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    return m;
  }

  visualization_msgs::Marker makeTextMarker(int id, const std::string& ns,
                                            const Eigen::Vector3d& p,
                                            const std::string& text,
                                            double size,
                                            float r, float g, float b, float a) const {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.position.x = p.x();
    m.pose.position.y = p.y();
    m.pose.position.z = p.z();
    m.pose.orientation.w = 1.0;
    m.scale.z = size;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    m.text = text;
    return m;
  }

  visualization_msgs::Marker makeArrowMarker(int id, const std::string& ns,
                                             const Eigen::Vector3d& p0,
                                             const Eigen::Vector3d& p1,
                                             float r, float g, float b, float a) const {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = visualization_msgs::Marker::ARROW;
    m.action = visualization_msgs::Marker::ADD;
    m.pose.orientation.w = 1.0;
    // For ARROW: x=shaft diameter, y=head diameter, z=head length
    m.scale.x = arrow_shaft_diameter_m_;
    m.scale.y = arrow_head_diameter_m_;
    m.scale.z = arrow_head_length_m_;
    m.color.r = r; m.color.g = g; m.color.b = b; m.color.a = a;
    geometry_msgs::Point gp0; gp0.x = p0.x(); gp0.y = p0.y(); gp0.z = p0.z();
    geometry_msgs::Point gp1; gp1.x = p1.x(); gp1.y = p1.y(); gp1.z = p1.z();
    m.points.push_back(gp0);
    m.points.push_back(gp1);
    return m;
  }

  visualization_msgs::Marker makeDeleteMarker(int id, const std::string& ns, int type) const {
    visualization_msgs::Marker m;
    m.header.frame_id = world_frame_;
    m.header.stamp = ros::Time::now();
    m.ns = ns;
    m.id = id;
    m.type = type;
    m.action = visualization_msgs::Marker::DELETE;
    return m;
  }

  void onTimer(const ros::TimerEvent&) {
    if (!a_.valid || !b_.valid || !c_.valid) {
      // In static mode we should always be valid; in dynamic mode, wait for messages.
      return;
    }

    // Build geometry in world frame
    const double th_a = a_.theta_deg * kDeg2Rad;
    const double th_b = b_.theta_deg * kDeg2Rad;
    const double th_c = c_.theta_deg * kDeg2Rad;
    const Eigen::Vector3d base_a = crane_a_params_.base_xyz;
    const Eigen::Vector3d base_b = crane_b_params_.base_xyz;
    const Eigen::Vector3d base_c = crane_c_params_.base_xyz;

    // 回转中心（大臂/小车所在高度）= 塔身顶部
    const Eigen::Vector3d ca = base_a + Eigen::Vector3d(0.0, 0.0, crane_a_params_.mast_height);
    const Eigen::Vector3d cb = base_b + Eigen::Vector3d(0.0, 0.0, crane_b_params_.mast_height);
    const Eigen::Vector3d cc = base_c + Eigen::Vector3d(0.0, 0.0, crane_c_params_.mast_height);

    const Eigen::Vector3d dir_a(std::cos(th_a), std::sin(th_a), 0.0);
    const Eigen::Vector3d dir_b(std::cos(th_b), std::sin(th_b), 0.0);
    const Eigen::Vector3d dir_c(std::cos(th_c), std::sin(th_c), 0.0);
    const Eigen::Vector3d ta = ca + crane_a_params_.arm_length * dir_a;
    const Eigen::Vector3d tb = cb + crane_b_params_.arm_length * dir_b;
    const Eigen::Vector3d tc = cc + crane_c_params_.arm_length * dir_c;

    const double ra = clampDouble(a_.r, 0.0, crane_a_params_.arm_length);
    const double rb = clampDouble(b_.r, 0.0, crane_b_params_.arm_length);
    const double rc = clampDouble(c_.r, 0.0, crane_c_params_.arm_length);
    const Eigen::Vector3d trolley_a = ca + ra * dir_a;
    const Eigen::Vector3d trolley_b = cb + rb * dir_b;
    const Eigen::Vector3d trolley_c = cc + rc * dir_c;
    const Eigen::Vector3d ha(trolley_a.x(), trolley_a.y(), a_.z);
    const Eigen::Vector3d hb(trolley_b.x(), trolley_b.y(), b_.z);
    const Eigen::Vector3d hc(trolley_c.x(), trolley_c.y(), c_.z);

    // TF: base + slew center + trolley + hook
    publishTF(crane_a_prefix_ + "/base", base_a);
    publishTF(crane_b_prefix_ + "/base", base_b);
    publishTF(crane_c_prefix_ + "/base", base_c);
    publishTF(crane_a_prefix_ + "/slew_center", ca);
    publishTF(crane_b_prefix_ + "/slew_center", cb);
    publishTF(crane_c_prefix_ + "/slew_center", cc);
    publishTF(crane_a_prefix_ + "/trolley", trolley_a);
    publishTF(crane_b_prefix_ + "/trolley", trolley_b);
    publishTF(crane_c_prefix_ + "/trolley", trolley_c);
    publishTF(crane_a_prefix_ + "/hook", ha);
    publishTF(crane_b_prefix_ + "/hook", hb);
    publishTF(crane_c_prefix_ + "/hook", hc);

    // Color by risk state (from gear limit topic)
    float cr = 0.2f, cg = 0.9f, cbg = 0.2f;  // SAFE green
    std::string level = "SAFE";
    if (use_gear_limit_topic_) {
      if (having_risk_) { cr = 0.95f; cg = 0.85f; cbg = 0.1f; level = "WARN"; }
      if (need_stop_) { cr = 0.95f; cg = 0.1f; cbg = 0.1f; level = "STOP"; }
    }
    
    // 检查A和B的风险闪烁状态（根据风险等级使用不同频率）
    ros::Time now = ros::Time::now();
    bool show_red_a = false, show_red_b = false, show_red_c = false;
    
    // A塔闪烁逻辑
    if (risk_level_a_ >= 1.0) {  // 有风险（低风险或高风险）
      if (risk_level_a_ >= 2.0) {
        // 高风险：高频闪烁（强制刹停）
        double period = 1.0 / high_risk_flash_freq_;
        double phase = std::fmod(now.toSec(), period);
        show_red_a = (phase < period * 0.5);  // 50%占空比，持续高频闪烁
      } else {
        // 低风险：低频闪烁（不刹停）
        double period = 1.0 / low_risk_flash_freq_;
        double phase = std::fmod(now.toSec(), period);
        show_red_a = (phase < period * 0.5);  // 50%占空比
      }
    }
    
    // B塔闪烁逻辑
    if (risk_level_b_ >= 1.0) {  // 有风险（低风险或高风险）
      if (risk_level_b_ >= 2.0) {
        // 高风险：高频闪烁（强制刹停）
        double period = 1.0 / high_risk_flash_freq_;
        double phase = std::fmod(now.toSec(), period);
        show_red_b = (phase < period * 0.5);  // 50%占空比，持续高频闪烁
      } else {
        // 低风险：低频闪烁（不刹停）
        double period = 1.0 / low_risk_flash_freq_;
        double phase = std::fmod(now.toSec(), period);
        show_red_b = (phase < period * 0.5);  // 50%占空比
      }
    }

    // C塔闪烁逻辑
    if (risk_level_c_ >= 1.0) {  // 有风险（低风险或高风险）
      if (risk_level_c_ >= 2.0) {
        double period = 1.0 / high_risk_flash_freq_;
        double phase = std::fmod(now.toSec(), period);
        show_red_c = (phase < period * 0.5);
      } else {
        double period = 1.0 / low_risk_flash_freq_;
        double phase = std::fmod(now.toSec(), period);
        show_red_c = (phase < period * 0.5);
      }
    }
    
    // A和B各自的颜色（根据风险等级闪烁）
    // 默认绿色（安全状态），只有在真正有风险时才改变颜色
    float cr_a = 0.2f, cg_a = 0.9f, cbg_a = 0.2f;  // 默认绿色
    float cr_b = 0.2f, cg_b = 0.9f, cbg_b = 0.2f;  // 默认绿色
    float cr_c = 0.2f, cg_c = 0.9f, cbg_c = 0.2f;  // 默认绿色
    
    // 根据风险等级设置颜色
    if (use_gear_limit_topic_) {
      // A的颜色
      if (risk_level_a_ >= 2.0) {
        // 高风险：红色（闪烁）
        if (show_red_a) {
          cr_a = 1.0f; cg_a = 0.0f; cbg_a = 0.0f;  // 红色
        } else {
          cr_a = 0.95f; cg_a = 0.1f; cbg_a = 0.1f;  // 暗红色
        }
      } else if (risk_level_a_ >= 1.0) {
        // 低风险：黄色（闪烁）
        if (show_red_a) {
          cr_a = 1.0f; cg_a = 0.0f; cbg_a = 0.0f;  // 红色（闪烁时）
        } else {
          cr_a = 0.95f; cg_a = 0.85f; cbg_a = 0.1f;  // 黄色
        }
      }
      
      // B的颜色
      if (risk_level_b_ >= 2.0) {
        // 高风险：红色（闪烁）
        if (show_red_b) {
          cr_b = 1.0f; cg_b = 0.0f; cbg_b = 0.0f;  // 红色
        } else {
          cr_b = 0.95f; cg_b = 0.1f; cbg_b = 0.1f;  // 暗红色
        }
      } else if (risk_level_b_ >= 1.0) {
        // 低风险：黄色（闪烁）
        if (show_red_b) {
          cr_b = 1.0f; cg_b = 0.0f; cbg_b = 0.0f;  // 红色（闪烁时）
        } else {
          cr_b = 0.95f; cg_b = 0.85f; cbg_b = 0.1f;  // 黄色
        }
      }

      // C的颜色
      if (risk_level_c_ >= 2.0) {
        if (show_red_c) {
          cr_c = 1.0f; cg_c = 0.0f; cbg_c = 0.0f;
        } else {
          cr_c = 0.95f; cg_c = 0.1f; cbg_c = 0.1f;
        }
      } else if (risk_level_c_ >= 1.0) {
        if (show_red_c) {
          cr_c = 1.0f; cg_c = 0.0f; cbg_c = 0.0f;
        } else {
          cr_c = 0.95f; cg_c = 0.85f; cbg_c = 0.1f;
        }
      }
    }

    visualization_msgs::MarkerArray arr;

    // Crane A: mast + arm + trolley + rope + hook
    {
      const Eigen::Vector3d mast_center = base_a + Eigen::Vector3d(0.0, 0.0, 0.5 * crane_a_params_.mast_height);
      arr.markers.push_back(makeCylinderMarker(1, "crane_a/mast", mast_center,
                                               crane_a_params_.mast_radius, crane_a_params_.mast_height,
                                               0.75f, 0.75f, 0.75f, 0.9f));

      const Eigen::Vector3d arm_center = ca + 0.5 * crane_a_params_.arm_length * dir_a;
      arr.markers.push_back(makeCubeMarker(2, "crane_a/arm", arm_center, th_a,
                                           crane_a_params_.arm_length, crane_a_params_.arm_width, crane_a_params_.arm_thickness,
                                           cr_a, cg_a, cbg_a, 0.9f));

      // Counter-jib (back arm): extend to opposite direction
      const Eigen::Vector3d counter_center = ca - 0.5 * crane_a_params_.counter_arm_length * dir_a;
      arr.markers.push_back(makeCubeMarker(6, "crane_a/counter_arm", counter_center, th_a,
                                           crane_a_params_.counter_arm_length, crane_a_params_.counter_arm_width, crane_a_params_.counter_arm_thickness,
                                           cr_a, cg_a, cbg_a, 0.9f));

      arr.markers.push_back(makeCubeMarker(3, "crane_a/trolley", trolley_a, th_a,
                                           crane_a_params_.trolley_length, crane_a_params_.trolley_width, crane_a_params_.trolley_height,
                                           0.9f, 0.2f, 0.9f, 0.95f));

      arr.markers.push_back(makeLineMarker(4, "crane_a/rope", trolley_a, ha, crane_a_params_.rope_radius * 2.0,
                                           0.2f, 0.6f, 1.0f, 0.9f));
      // Hook: 刹停时变红，否则保持亮橙红
      if (show_red_a) {
        arr.markers.push_back(makeSphereMarker(5, "crane_a/hook", ha, crane_a_params_.hook_radius, 1.0f, 0.0f, 0.0f, 1.0f));
      } else {
        arr.markers.push_back(makeSphereMarker(5, "crane_a/hook", ha, crane_a_params_.hook_radius, 1.0f, 0.25f, 0.05f, 1.0f));
      }

      // 运动方向箭头：大臂/小车/吊钩 各自运动才显示；各自静止则删除箭头
      if (motion_arrows_enabled_) {
        const float ar = 0.95f, ag = 0.95f, ab = 0.95f, aa = 0.95f;
        // 大臂回转：切向箭头（由 theta 速度符号决定方向）
        if (std::abs(a_.theta_vel_deg_per_sec) > arrow_theta_vel_thresh_deg_per_sec_) {
          const double sign = (a_.theta_vel_deg_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d p = ca + 0.8 * crane_a_params_.arm_length * dir_a;
          const Eigen::Vector3d tang = sign * Eigen::Vector3d(-dir_a.y(), dir_a.x(), 0.0);
          arr.markers.push_back(makeArrowMarker(7, "crane_a/arrow_arm", p, p + arm_arrow_length_m_ * tang, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(7, "crane_a/arrow_arm", visualization_msgs::Marker::ARROW));
        }
        // 小车变幅：沿大臂方向箭头（由 r 速度符号决定方向）
        if (std::abs(a_.r_vel_m_per_sec) > arrow_r_vel_thresh_m_per_sec_) {
          const double sign = (a_.r_vel_m_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d d = sign * dir_a;
          arr.markers.push_back(makeArrowMarker(8, "crane_a/arrow_trolley", trolley_a, trolley_a + trolley_arrow_length_m_ * d, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(8, "crane_a/arrow_trolley", visualization_msgs::Marker::ARROW));
        }
        // 吊钩起升：竖直箭头（由 z 速度符号决定方向）
        if (std::abs(a_.z_vel_m_per_sec) > arrow_z_vel_thresh_m_per_sec_) {
          const double sign = (a_.z_vel_m_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d d(0.0, 0.0, sign);
          arr.markers.push_back(makeArrowMarker(9, "crane_a/arrow_hook", ha, ha + hook_arrow_length_m_ * d, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(9, "crane_a/arrow_hook", visualization_msgs::Marker::ARROW));
        }
      }
    }

    // Crane B: mast + arm + trolley + rope + hook
    {
      const Eigen::Vector3d mast_center = base_b + Eigen::Vector3d(0.0, 0.0, 0.5 * crane_b_params_.mast_height);
      arr.markers.push_back(makeCylinderMarker(11, "crane_b/mast", mast_center,
                                               crane_b_params_.mast_radius, crane_b_params_.mast_height,
                                               0.75f, 0.75f, 0.75f, 0.9f));

      const Eigen::Vector3d arm_center = cb + 0.5 * crane_b_params_.arm_length * dir_b;
      arr.markers.push_back(makeCubeMarker(12, "crane_b/arm", arm_center, th_b,
                                           crane_b_params_.arm_length, crane_b_params_.arm_width, crane_b_params_.arm_thickness,
                                           cr_b, cg_b, cbg_b, 0.9f));

      // Counter-jib (back arm): extend to opposite direction
      const Eigen::Vector3d counter_center = cb - 0.5 * crane_b_params_.counter_arm_length * dir_b;
      arr.markers.push_back(makeCubeMarker(16, "crane_b/counter_arm", counter_center, th_b,
                                           crane_b_params_.counter_arm_length, crane_b_params_.counter_arm_width, crane_b_params_.counter_arm_thickness,
                                           cr_b, cg_b, cbg_b, 0.9f));

      arr.markers.push_back(makeCubeMarker(13, "crane_b/trolley", trolley_b, th_b,
                                           crane_b_params_.trolley_length, crane_b_params_.trolley_width, crane_b_params_.trolley_height,
                                           0.9f, 0.2f, 0.9f, 0.95f));

      arr.markers.push_back(makeLineMarker(14, "crane_b/rope", trolley_b, hb, crane_b_params_.rope_radius * 2.0,
                                           0.2f, 0.6f, 1.0f, 0.9f));
      // Hook: 刹停时变红，否则保持亮橙红
      if (show_red_b) {
        arr.markers.push_back(makeSphereMarker(15, "crane_b/hook", hb, crane_b_params_.hook_radius, 1.0f, 0.0f, 0.0f, 1.0f));
      } else {
        arr.markers.push_back(makeSphereMarker(15, "crane_b/hook", hb, crane_b_params_.hook_radius, 1.0f, 0.25f, 0.05f, 1.0f));
      }

      // 运动方向箭头：大臂/小车/吊钩 各自运动才显示；各自静止则删除箭头
      if (motion_arrows_enabled_) {
        const float ar = 0.95f, ag = 0.95f, ab = 0.95f, aa = 0.95f;
        // 大臂回转：切向箭头（由 theta 速度符号决定方向）
        if (std::abs(b_.theta_vel_deg_per_sec) > arrow_theta_vel_thresh_deg_per_sec_) {
          const double sign = (b_.theta_vel_deg_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d p = cb + 0.8 * crane_b_params_.arm_length * dir_b;
          const Eigen::Vector3d tang = sign * Eigen::Vector3d(-dir_b.y(), dir_b.x(), 0.0);
          arr.markers.push_back(makeArrowMarker(17, "crane_b/arrow_arm", p, p + arm_arrow_length_m_ * tang, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(17, "crane_b/arrow_arm", visualization_msgs::Marker::ARROW));
        }
        // 小车变幅：沿大臂方向箭头（由 r 速度符号决定方向）
        if (std::abs(b_.r_vel_m_per_sec) > arrow_r_vel_thresh_m_per_sec_) {
          const double sign = (b_.r_vel_m_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d d = sign * dir_b;
          arr.markers.push_back(makeArrowMarker(18, "crane_b/arrow_trolley", trolley_b, trolley_b + trolley_arrow_length_m_ * d, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(18, "crane_b/arrow_trolley", visualization_msgs::Marker::ARROW));
        }
        // 吊钩起升：竖直箭头（由 z 速度符号决定方向）
        if (std::abs(b_.z_vel_m_per_sec) > arrow_z_vel_thresh_m_per_sec_) {
          const double sign = (b_.z_vel_m_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d d(0.0, 0.0, sign);
          arr.markers.push_back(makeArrowMarker(19, "crane_b/arrow_hook", hb, hb + hook_arrow_length_m_ * d, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(19, "crane_b/arrow_hook", visualization_msgs::Marker::ARROW));
        }
      }
    }

    // Crane C: mast + arm + trolley + rope + hook
    {
      const Eigen::Vector3d mast_center = base_c + Eigen::Vector3d(0.0, 0.0, 0.5 * crane_c_params_.mast_height);
      arr.markers.push_back(makeCylinderMarker(21, "crane_c/mast", mast_center,
                                               crane_c_params_.mast_radius, crane_c_params_.mast_height,
                                               0.75f, 0.75f, 0.75f, 0.9f));

      const Eigen::Vector3d arm_center = cc + 0.5 * crane_c_params_.arm_length * dir_c;
      arr.markers.push_back(makeCubeMarker(22, "crane_c/arm", arm_center, th_c,
                                           crane_c_params_.arm_length, crane_c_params_.arm_width, crane_c_params_.arm_thickness,
                                           cr_c, cg_c, cbg_c, 0.9f));

      // Counter-jib (back arm): extend to opposite direction
      const Eigen::Vector3d counter_center = cc - 0.5 * crane_c_params_.counter_arm_length * dir_c;
      arr.markers.push_back(makeCubeMarker(26, "crane_c/counter_arm", counter_center, th_c,
                                           crane_c_params_.counter_arm_length, crane_c_params_.counter_arm_width, crane_c_params_.counter_arm_thickness,
                                           cr_c, cg_c, cbg_c, 0.9f));

      arr.markers.push_back(makeCubeMarker(23, "crane_c/trolley", trolley_c, th_c,
                                           crane_c_params_.trolley_length, crane_c_params_.trolley_width, crane_c_params_.trolley_height,
                                           0.9f, 0.2f, 0.9f, 0.95f));

      arr.markers.push_back(makeLineMarker(24, "crane_c/rope", trolley_c, hc, crane_c_params_.rope_radius * 2.0,
                                           0.2f, 0.6f, 1.0f, 0.9f));
      // Hook: 刹停时变红，否则保持亮橙红
      if (show_red_c) {
        arr.markers.push_back(makeSphereMarker(25, "crane_c/hook", hc, crane_c_params_.hook_radius, 1.0f, 0.0f, 0.0f, 1.0f));
      } else {
        arr.markers.push_back(makeSphereMarker(25, "crane_c/hook", hc, crane_c_params_.hook_radius, 1.0f, 0.25f, 0.05f, 1.0f));
      }

      // 运动方向箭头：大臂/小车/吊钩 各自运动才显示；各自静止则删除箭头
      if (motion_arrows_enabled_) {
        const float ar = 0.95f, ag = 0.95f, ab = 0.95f, aa = 0.95f;
        // 大臂回转：切向箭头
        if (std::abs(c_.theta_vel_deg_per_sec) > arrow_theta_vel_thresh_deg_per_sec_) {
          const double sign = (c_.theta_vel_deg_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d p = cc + 0.8 * crane_c_params_.arm_length * dir_c;
          const Eigen::Vector3d tang = sign * Eigen::Vector3d(-dir_c.y(), dir_c.x(), 0.0);
          arr.markers.push_back(makeArrowMarker(27, "crane_c/arrow_arm", p, p + arm_arrow_length_m_ * tang, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(27, "crane_c/arrow_arm", visualization_msgs::Marker::ARROW));
        }
        // 小车变幅：沿大臂方向箭头
        if (std::abs(c_.r_vel_m_per_sec) > arrow_r_vel_thresh_m_per_sec_) {
          const double sign = (c_.r_vel_m_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d d = sign * dir_c;
          arr.markers.push_back(makeArrowMarker(28, "crane_c/arrow_trolley", trolley_c, trolley_c + trolley_arrow_length_m_ * d, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(28, "crane_c/arrow_trolley", visualization_msgs::Marker::ARROW));
        }
        // 吊钩起升：竖直箭头
        if (std::abs(c_.z_vel_m_per_sec) > arrow_z_vel_thresh_m_per_sec_) {
          const double sign = (c_.z_vel_m_per_sec >= 0.0) ? 1.0 : -1.0;
          const Eigen::Vector3d d(0.0, 0.0, sign);
          arr.markers.push_back(makeArrowMarker(29, "crane_c/arrow_hook", hc, hc + hook_arrow_length_m_ * d, ar, ag, ab, aa));
        } else {
          arr.markers.push_back(makeDeleteMarker(29, "crane_c/arrow_hook", visualization_msgs::Marker::ARROW));
        }
      }
    }

    // Distance summary (computed here for visualization)
    const DistResult d_ab = computeDistance(ca, ta, cb, tb, trolley_a, ha, trolley_b, hb);
    const DistResult d_ac = computeDistance(ca, ta, cc, tc, trolley_a, ha, trolley_c, hc);
    const DistResult d_bc = computeDistance(cb, tb, cc, tc, trolley_b, hb, trolley_c, hc);
    DistResult d_min = d_ab;
    std::string pair_label = "A-B:" + d_ab.pair;
    if (d_ac.d_min < d_min.d_min) {
      d_min = d_ac;
      pair_label = "A-C:" + d_ac.pair;
    }
    if (d_bc.d_min < d_min.d_min) {
      d_min = d_bc;
      pair_label = "B-C:" + d_bc.pair;
    }
    const Eigen::Vector3d text_pos = (ca + cb + cc) / 3.0 + Eigen::Vector3d(0.0, 0.0, 3.0);
    const std::string text = "level=" + level + "  d_min=" + std::to_string(d_min.d_min) + "  pair=" + pair_label;
    arr.markers.push_back(makeTextMarker(100, "summary", text_pos, text, 0.8, 1.0f, 1.0f, 1.0f, 0.95f));

    pub_markers_.publish(arr);
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle pnh_;

  ros::Subscriber sub_a_;
  ros::Subscriber sub_b_;
  ros::Subscriber sub_c_;
  ros::Subscriber sub_limit_;
  ros::Subscriber sub_limit_a_;
  ros::Subscriber sub_limit_b_;
  ros::Subscriber sub_limit_c_;
  ros::Publisher pub_markers_;
  ros::Timer timer_;
  tf2_ros::TransformBroadcaster tf_br_;

  std::string crane_a_state_topic_;
  std::string crane_b_state_topic_;
  std::string crane_c_state_topic_;
  std::string gear_limit_topic_;
  std::string marker_topic_;

  std::string world_frame_;
  std::string crane_a_prefix_;
  std::string crane_b_prefix_;
  std::string crane_c_prefix_;

  double viz_hz_{20.0};
  double stop_red_duration_{1.0};  // 刹停红色闪烁持续时间（秒）

  CraneParams crane_a_params_;
  CraneParams crane_b_params_;
  CraneParams crane_c_params_;

  CraneState a_;
  CraneState b_;
  CraneState c_;

  bool having_risk_{false};
  bool need_stop_{false};
  bool stop_a_{false};  // A塔刹停状态
  bool stop_b_{false};  // B塔刹停状态
  bool stop_c_{false};  // C塔刹停状态
  ros::Time stop_time_a_;  // A塔刹停时间戳（初始化为无效）
  ros::Time stop_time_b_;  // B塔刹停时间戳（初始化为无效）
  ros::Time stop_time_c_;  // C塔刹停时间戳（初始化为无效）
  
  // 风险等级：0=无风险, 1=低风险, 2=高风险
  double risk_level_a_{0.0};  // A塔风险等级
  double risk_level_b_{0.0};  // B塔风险等级
  double risk_level_c_{0.0};  // C塔风险等级
  
  // 闪烁频率参数
  double low_risk_flash_freq_{1.0};   // 低风险闪烁频率（Hz）
  double high_risk_flash_freq_{5.0};  // 高风险闪烁频率（Hz）

  // 运动方向箭头（按组件各自是否运动来显示/隐藏）
  bool motion_arrows_enabled_{true};
  double arrow_theta_vel_thresh_deg_per_sec_{0.05};
  double arrow_r_vel_thresh_m_per_sec_{0.01};
  double arrow_z_vel_thresh_m_per_sec_{0.01};
  double arm_arrow_length_m_{4.0};
  double trolley_arrow_length_m_{3.0};
  double hook_arrow_length_m_{3.0};
  double arrow_shaft_diameter_m_{0.25};
  double arrow_head_diameter_m_{0.45};
  double arrow_head_length_m_{0.7};

  bool use_state_topics_{true};
  bool use_state_topic_a_{true};
  bool use_state_topic_b_{true};
  bool use_state_topic_c_{true};
  bool use_gear_limit_topic_{true};
  double static_a_theta_deg_{0.0};
  double static_a_r_{10.0};
  double static_a_z_{10.0};
  double static_b_theta_deg_{180.0};
  double static_b_r_{10.0};
  double static_b_z_{10.0};
  double static_c_theta_deg_{90.0};
  double static_c_r_{12.0};
  double static_c_z_{10.0};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "multi_crane_anticollision_viz_node");
  MultiCraneAntiCollisionVizNode node;
  ros::spin();
  return 0;
}

