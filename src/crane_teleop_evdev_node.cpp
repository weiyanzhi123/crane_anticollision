#include <ros/ros.h>
#include <crane_msg/JointPoseMsg.h>
#include <crane_msg/LoadCollisionAvoidCtrlLimit.h>
#include "crane_anticollision/CraneGearLimit.h"

#include <linux/input.h>
#include <fcntl.h>
#include <unistd.h>

#include <cerrno>
#include <cmath>
#include <cstring>
#include <string>
#include <mutex>
#include <unordered_set>
#include <vector>

#include <XmlRpcValue.h>

// Read /dev/input/event* to get real KEY_DOWN/KEY_UP, enabling multi-key combos + immediate stop on release.

static bool setNonBlocking(int fd) {
  int flags = fcntl(fd, F_GETFL, 0);
  if (flags < 0) return false;
  return (fcntl(fd, F_SETFL, flags | O_NONBLOCK) == 0);
}

static inline void normalizeAngleDeg(double& deg) {
  while (deg < 0.0) deg += 360.0;
  while (deg >= 360.0) deg -= 360.0;
}

struct CraneState {
  double theta_deg{0.0};
  double r{0.0};
  double z{0.0};
  double cmd_theta{0.0};  // 目标命令：-1, 0, or +1
  double cmd_r{0.0};
  double cmd_z{0.0};
  
  // 实际速度（平滑后的，用于加速/减速）
  double vel_theta{0.0};  // 当前实际速度（度/秒）
  double vel_r{0.0};      // 当前实际速度（米/秒）
  double vel_z{0.0};      // 当前实际速度（米/秒）

  // Gear index (1-based)
  int gear_theta{1};
  int gear_r{1};
  int gear_z{1};
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "crane_teleop_evdev");
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  std::string device;
  std::string output_topic_a, output_topic_b;
  std::string gear_limit_topic_a, gear_limit_topic_b;
  double hz = 50.0;
  double vel_theta_deg_per_sec = 5.0;  // 最大速度
  double vel_r_m_per_sec = 0.5;
  double vel_z_m_per_sec = 0.3;
  
  // 加速度参数（更接近真实场景）
  double accel_theta_deg_per_sec2 = 10.0;  // 回转加速度（度/秒²）
  double accel_r_m_per_sec2 = 1.0;         // 小车加速度（米/秒²）
  double accel_z_m_per_sec2 = 0.6;         // 起升加速度（米/秒²）
  
  // 减速度参数（通常比加速度大，刹停更快）
  double decel_theta_deg_per_sec2 = 15.0;  // 回转减速度（度/秒²）
  double decel_r_m_per_sec2 = 1.5;         // 小车减速度（米/秒²）
  double decel_z_m_per_sec2 = 0.9;         // 起升减速度（米/秒²）

  // Defaults: try common udev symlink (user should override on their machine)
  pnh.param<std::string>("device", device, "/dev/input/by-id/usb-keyboard-event-kbd");
  pnh.param<std::string>("output_topic_a", output_topic_a, "/crane_teleop/crane_a_states");
  pnh.param<std::string>("output_topic_b", output_topic_b, "/crane_teleop/crane_b_states");
  pnh.param<std::string>("gear_limit_topic_a", gear_limit_topic_a, "/planner/gear_limit_a");
  pnh.param<std::string>("gear_limit_topic_b", gear_limit_topic_b, "/planner/gear_limit_b");
  pnh.param("hz", hz, 50.0);
  pnh.param("vel_theta_deg_per_sec", vel_theta_deg_per_sec, 5.0);
  pnh.param("vel_r_m_per_sec", vel_r_m_per_sec, 0.5);
  pnh.param("vel_z_m_per_sec", vel_z_m_per_sec, 0.3);
  
  pnh.param("accel_theta_deg_per_sec2", accel_theta_deg_per_sec2, 10.0);
  pnh.param("accel_r_m_per_sec2", accel_r_m_per_sec2, 1.0);
  pnh.param("accel_z_m_per_sec2", accel_z_m_per_sec2, 0.6);
  
  pnh.param("decel_theta_deg_per_sec2", decel_theta_deg_per_sec2, 15.0);
  pnh.param("decel_r_m_per_sec2", decel_r_m_per_sec2, 1.5);
  pnh.param("decel_z_m_per_sec2", decel_z_m_per_sec2, 0.9);

  std::vector<double> slew_gears_deg_per_sec_default{0.7, 1.77, 2.66, 3.5};
  std::vector<double> trolley_gears_m_per_sec_default{0.263, 0.616, 1.036};
  std::vector<double> hoist_gears_m_per_sec_default{0.0735, 0.182, 0.362, 0.460, 0.604};
  std::vector<double> slew_gears_deg_per_sec;
  std::vector<double> trolley_gears_m_per_sec;
  std::vector<double> hoist_gears_m_per_sec;

  auto loadGearList = [&](const std::string& name, std::vector<double>& out,
                          const std::vector<double>& defaults) {
    XmlRpc::XmlRpcValue value;
    if (pnh.getParam(name, value) && value.getType() == XmlRpc::XmlRpcValue::TypeArray) {
      out.clear();
      for (int i = 0; i < value.size(); ++i) {
        out.push_back(static_cast<double>(value[i]));
      }
    } else {
      out = defaults;
    }
  };

  loadGearList("slew_gears_deg_per_sec", slew_gears_deg_per_sec, slew_gears_deg_per_sec_default);
  loadGearList("trolley_gears_m_per_sec", trolley_gears_m_per_sec, trolley_gears_m_per_sec_default);
  loadGearList("hoist_gears_m_per_sec", hoist_gears_m_per_sec, hoist_gears_m_per_sec_default);

  CraneState A, B;
  pnh.param("init_a_theta_deg", A.theta_deg, 45.0);
  pnh.param("init_a_r", A.r, 15.0);
  pnh.param("init_a_z", A.z, 25.0);
  pnh.param("init_b_theta_deg", B.theta_deg, 235.0);
  pnh.param("init_b_r", B.r, 22.0);
  pnh.param("init_b_z", B.z, 6.0);

  const int fd = open(device.c_str(), O_RDONLY);
  if (fd < 0) {
    ROS_ERROR("Failed to open evdev device '%s': %s", device.c_str(), std::strerror(errno));
    ROS_ERROR("Tips:");
    ROS_ERROR("  - Find a keyboard device: ls -l /dev/input/by-id/*kbd OR ls -l /dev/input/event*");
    ROS_ERROR("  - Set param ~device to the correct event node.");
    ROS_ERROR("  - Ensure permissions (may require sudo or udev rule).");
    return 1;
  }
  if (!setNonBlocking(fd)) {
    ROS_WARN("Failed to set non-blocking mode on '%s': %s", device.c_str(), std::strerror(errno));
  }

  ros::Publisher pub_a = nh.advertise<crane_msg::JointPoseMsg>(output_topic_a, 1);
  ros::Publisher pub_b = nh.advertise<crane_msg::JointPoseMsg>(output_topic_b, 1);

  // 订阅防碰撞限制消息
  std::mutex limit_mutex;
  bool force_stop_a = false, force_stop_b = false;
  int gear_limit_theta_a = 4, gear_limit_r_a = 3, gear_limit_z_a = 5;
  int gear_limit_theta_b = 4, gear_limit_r_b = 3, gear_limit_z_b = 5;
  
  auto limit_cb_a = [&](const crane_msg::LoadCollisionAvoidCtrlLimit::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(limit_mutex);
    force_stop_a = msg->need_stop_now;  // 强制刹停标志
  };
  auto limit_cb_b = [&](const crane_msg::LoadCollisionAvoidCtrlLimit::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(limit_mutex);
    force_stop_b = msg->need_stop_now;  // 强制刹停标志
  };

  auto gear_limit_cb_a = [&](const crane_anticollision::CraneGearLimit::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(limit_mutex);
    gear_limit_theta_a = msg->gear_theta_limit;
    gear_limit_r_a = msg->gear_r_limit;
    gear_limit_z_a = msg->gear_z_limit;
  };
  auto gear_limit_cb_b = [&](const crane_anticollision::CraneGearLimit::ConstPtr& msg) {
    std::lock_guard<std::mutex> lock(limit_mutex);
    gear_limit_theta_b = msg->gear_theta_limit;
    gear_limit_r_b = msg->gear_r_limit;
    gear_limit_z_b = msg->gear_z_limit;
  };
  ros::Subscriber sub_limit_a = nh.subscribe<crane_msg::LoadCollisionAvoidCtrlLimit>(
      "/planner/collision_avoid_info_a", 1, limit_cb_a);
  ros::Subscriber sub_limit_b = nh.subscribe<crane_msg::LoadCollisionAvoidCtrlLimit>(
      "/planner/collision_avoid_info_b", 1, limit_cb_b);
  ros::Subscriber sub_gear_limit_a = nh.subscribe<crane_anticollision::CraneGearLimit>(
      gear_limit_topic_a, 1, gear_limit_cb_a);
  ros::Subscriber sub_gear_limit_b = nh.subscribe<crane_anticollision::CraneGearLimit>(
      gear_limit_topic_b, 1, gear_limit_cb_b);

  std::unordered_set<int> pressed;  // key codes currently pressed

  ROS_INFO("========================================");
  ROS_INFO("Crane Teleop (evdev, true multi-key)");
  ROS_INFO("device: %s", device.c_str());
  ROS_INFO("A keys: W/S (r+/r-)  A/D (theta-/theta+)  Q/E (z+/z-)");
  ROS_INFO("B keys: I/K (r+/r-)  J/L (theta-/theta+) U/O (z+/z-)");
  ROS_INFO("SPACE: stop all  X: exit");
  ROS_INFO("Gear: hold direction key + number key (1..5) to set gear for that axis");
  ROS_INFO("Topics: A=%s , B=%s", output_topic_a.c_str(), output_topic_b.c_str());
  ROS_INFO("========================================");

  ros::Rate rate(hz > 0.0 ? hz : 50.0);

  while (ros::ok()) {
    // Drain all pending input events
    for (;;) {
      input_event ev;
      const ssize_t n = read(fd, &ev, sizeof(ev));
      if (n < 0) {
        if (errno == EAGAIN || errno == EWOULDBLOCK) break;
        ROS_WARN_THROTTLE(1.0, "evdev read error: %s", std::strerror(errno));
        break;
      }
      if (n != sizeof(ev)) break;

      if (ev.type == EV_KEY) {
        // value: 0 release, 1 press, 2 autorepeat (treat as pressed)
        if (ev.value == 0) pressed.erase(ev.code);
        else pressed.insert(ev.code);
      }
    }

    auto down = [&](int key_code) -> bool { return pressed.find(key_code) != pressed.end(); };

    const int gear_keys[] = {KEY_1, KEY_2, KEY_3, KEY_4, KEY_5, KEY_6, KEY_7, KEY_8, KEY_9};
    bool any_gear_pressed = false;
    for (int key : gear_keys) {
      if (down(key)) {
        any_gear_pressed = true;
        break;
      }
    }

    auto getPressedGear = [&](int max_gear) -> int {
      if (max_gear <= 0) return 0;
      const int max_check = std::min(max_gear, static_cast<int>(sizeof(gear_keys) / sizeof(gear_keys[0])));
      for (int i = max_check; i >= 1; --i) {
        if (down(gear_keys[i - 1])) return i;
      }
      return 0;
    };

    // Compute commands from current pressed set (this gives immediate stop on release)
    auto computeCmd = [&](CraneState& C, int which) {
      // reset
      C.cmd_theta = C.cmd_r = C.cmd_z = 0.0;

      if (down(KEY_SPACE)) {
        C.cmd_theta = C.cmd_r = C.cmd_z = 0.0;
        return;
      }
      if (down(KEY_X)) {
        ros::shutdown();
        return;
      }

      if (which == 0) {
        if (down(KEY_W)) C.cmd_r += 1.0;
        if (down(KEY_S)) C.cmd_r -= 1.0;
        if (down(KEY_A)) C.cmd_theta -= 1.0;
        if (down(KEY_D)) C.cmd_theta += 1.0;
        if (down(KEY_Q)) C.cmd_z += 1.0;
        if (down(KEY_E)) C.cmd_z -= 1.0;
      } else if (which == 1) {
        if (down(KEY_I)) C.cmd_r += 1.0;
        if (down(KEY_K)) C.cmd_r -= 1.0;
        if (down(KEY_J)) C.cmd_theta -= 1.0;
        if (down(KEY_L)) C.cmd_theta += 1.0;
        if (down(KEY_U)) C.cmd_z += 1.0;
        if (down(KEY_O)) C.cmd_z -= 1.0;
      }

      // Normalize to [-1, 0, 1] even if both pressed
      C.cmd_theta = (C.cmd_theta > 0.0) ? 1.0 : (C.cmd_theta < 0.0 ? -1.0 : 0.0);
      C.cmd_r = (C.cmd_r > 0.0) ? 1.0 : (C.cmd_r < 0.0 ? -1.0 : 0.0);
      C.cmd_z = (C.cmd_z > 0.0) ? 1.0 : (C.cmd_z < 0.0 ? -1.0 : 0.0);

      const bool theta_active = (C.cmd_theta != 0.0);
      const bool r_active = (C.cmd_r != 0.0);
      const bool z_active = (C.cmd_z != 0.0);

      const int theta_gear = getPressedGear(static_cast<int>(slew_gears_deg_per_sec.size()));
      const int r_gear = getPressedGear(static_cast<int>(trolley_gears_m_per_sec.size()));
      const int z_gear = getPressedGear(static_cast<int>(hoist_gears_m_per_sec.size()));

      if (!any_gear_pressed) {
        if (theta_active) C.gear_theta = 1;
        if (r_active) C.gear_r = 1;
        if (z_active) C.gear_z = 1;
      }

      if (theta_active && theta_gear > 0) C.gear_theta = theta_gear;
      if (r_active && r_gear > 0) C.gear_r = r_gear;
      if (z_active && z_gear > 0) C.gear_z = z_gear;

      C.gear_theta = std::max(1, std::min(C.gear_theta, std::max(1, static_cast<int>(slew_gears_deg_per_sec.size()))));
      C.gear_r = std::max(1, std::min(C.gear_r, std::max(1, static_cast<int>(trolley_gears_m_per_sec.size()))));
      C.gear_z = std::max(1, std::min(C.gear_z, std::max(1, static_cast<int>(hoist_gears_m_per_sec.size()))));
    };

    computeCmd(A, 0);
    computeCmd(B, 1);

    {
      std::lock_guard<std::mutex> lock(limit_mutex);
      if (gear_limit_theta_a <= 0) { A.cmd_theta = 0.0; A.gear_theta = 0; }
      else A.gear_theta = std::min(A.gear_theta, gear_limit_theta_a);
      if (gear_limit_r_a <= 0) { A.cmd_r = 0.0; A.gear_r = 0; }
      else A.gear_r = std::min(A.gear_r, gear_limit_r_a);
      if (gear_limit_z_a <= 0) { A.cmd_z = 0.0; A.gear_z = 0; }
      else A.gear_z = std::min(A.gear_z, gear_limit_z_a);

      if (gear_limit_theta_b <= 0) { B.cmd_theta = 0.0; B.gear_theta = 0; }
      else B.gear_theta = std::min(B.gear_theta, gear_limit_theta_b);
      if (gear_limit_r_b <= 0) { B.cmd_r = 0.0; B.gear_r = 0; }
      else B.gear_r = std::min(B.gear_r, gear_limit_r_b);
      if (gear_limit_z_b <= 0) { B.cmd_z = 0.0; B.gear_z = 0; }
      else B.gear_z = std::min(B.gear_z, gear_limit_z_b);
    }

    // 强制刹停覆盖手动控制（优先级最高）
    {
      std::lock_guard<std::mutex> lock(limit_mutex);
      if (force_stop_a) {
        A.cmd_theta = A.cmd_r = A.cmd_z = 0.0;  // 强制刹停，覆盖手动控制
      }
      if (force_stop_b) {
        B.cmd_theta = B.cmd_r = B.cmd_z = 0.0;  // 强制刹停，覆盖手动控制
      }
    }

    const double dt = rate.expectedCycleTime().toSec();

    // 平滑速度更新（加速/减速过程）
    auto updateVelocities = [&](CraneState& C, double max_v_theta, double max_v_r, double max_v_z,
                                double accel_theta, double accel_r, double accel_z,
                                double decel_theta, double decel_r, double decel_z) {
      // 目标速度
      double target_v_theta = C.cmd_theta * max_v_theta;
      double target_v_r = C.cmd_r * max_v_r;
      double target_v_z = C.cmd_z * max_v_z;
      
      // 平滑过渡到目标速度（加速/减速）
      auto smoothVelocity = [dt](double& current, double target, double accel, double decel) {
        double diff = target - current;
        if (std::abs(diff) < 1e-6) {
          current = target;
          return;
        }
        
        // 判断是加速还是减速
        // 如果目标速度和当前速度方向相反，先减速到0
        bool opposite_sign = (target > 0 && current < 0) || (target < 0 && current > 0);
        double max_change;
        
        if (opposite_sign) {
          // 方向相反：先减速到0
          max_change = decel * dt;
          if (std::abs(current) <= max_change) {
            current = 0.0;  // 已减速到0
          } else {
            current += (current > 0 ? -max_change : max_change);  // 向0减速
          }
        } else {
          // 方向相同：判断是加速还是减速
          bool is_accel = std::abs(target) > std::abs(current);
          max_change = (is_accel ? accel : decel) * dt;
          
          if (std::abs(diff) <= max_change) {
            current = target;  // 已经达到目标
          } else {
            current += (diff > 0 ? max_change : -max_change);  // 平滑过渡
          }
        }
      };
      
      smoothVelocity(C.vel_theta, target_v_theta, accel_theta, decel_theta);
      smoothVelocity(C.vel_r, target_v_r, accel_r, decel_r);
      smoothVelocity(C.vel_z, target_v_z, accel_z, decel_z);
    };
    
    auto gearSpeed = [&](const std::vector<double>& gears, int gear_idx, double fallback) {
      if (gears.empty()) return fallback;
      const int idx = std::max(1, std::min(gear_idx, static_cast<int>(gears.size())));
      return gears[idx - 1];
    };

    const double max_v_theta_a = gearSpeed(slew_gears_deg_per_sec, A.gear_theta, vel_theta_deg_per_sec);
    const double max_v_r_a = gearSpeed(trolley_gears_m_per_sec, A.gear_r, vel_r_m_per_sec);
    const double max_v_z_a = gearSpeed(hoist_gears_m_per_sec, A.gear_z, vel_z_m_per_sec);
    const double max_v_theta_b = gearSpeed(slew_gears_deg_per_sec, B.gear_theta, vel_theta_deg_per_sec);
    const double max_v_r_b = gearSpeed(trolley_gears_m_per_sec, B.gear_r, vel_r_m_per_sec);
    const double max_v_z_b = gearSpeed(hoist_gears_m_per_sec, B.gear_z, vel_z_m_per_sec);

    updateVelocities(A, max_v_theta_a, max_v_r_a, max_v_z_a,
                     accel_theta_deg_per_sec2, accel_r_m_per_sec2, accel_z_m_per_sec2,
                     decel_theta_deg_per_sec2, decel_r_m_per_sec2, decel_z_m_per_sec2);
    updateVelocities(B, max_v_theta_b, max_v_r_b, max_v_z_b,
                     accel_theta_deg_per_sec2, accel_r_m_per_sec2, accel_z_m_per_sec2,
                     decel_theta_deg_per_sec2, decel_r_m_per_sec2, decel_z_m_per_sec2);

    // 用实际速度更新位置
    A.theta_deg += A.vel_theta * dt;
    A.r += A.vel_r * dt;
    A.z += A.vel_z * dt;
    normalizeAngleDeg(A.theta_deg);
    A.r = std::max(0.0, A.r);
    A.z = std::max(0.0, A.z);

    B.theta_deg += B.vel_theta * dt;
    B.r += B.vel_r * dt;
    B.z += B.vel_z * dt;
    normalizeAngleDeg(B.theta_deg);
    B.r = std::max(0.0, B.r);
    B.z = std::max(0.0, B.z);

    const ros::Time stamp = ros::Time::now();

    crane_msg::JointPoseMsg msg_a;
    msg_a.header.stamp = stamp;
    msg_a.header.frame_id = "world";
    msg_a.PolarAngle = A.theta_deg;
    msg_a.PolarRadius = A.r;
    msg_a.PolarHeight = A.z;
    msg_a.PolarAngleVel = A.vel_theta;  // 使用实际速度
    msg_a.PolarRadiusVel = A.vel_r;
    msg_a.PolarHeightVel = A.vel_z;
    msg_a.SensorMsgError = false;
    msg_a.PLCMode = A.gear_theta + (A.gear_r * 10) + (A.gear_z * 100);

    crane_msg::JointPoseMsg msg_b;
    msg_b.header.stamp = stamp;
    msg_b.header.frame_id = "world";
    msg_b.PolarAngle = B.theta_deg;
    msg_b.PolarRadius = B.r;
    msg_b.PolarHeight = B.z;
    msg_b.PolarAngleVel = B.vel_theta;  // 使用实际速度
    msg_b.PolarRadiusVel = B.vel_r;
    msg_b.PolarHeightVel = B.vel_z;
    msg_b.SensorMsgError = false;
    msg_b.PLCMode = B.gear_theta + (B.gear_r * 10) + (B.gear_z * 100);
    pub_a.publish(msg_a);
    pub_b.publish(msg_b);

    ros::spinOnce();
    rate.sleep();
  }

  close(fd);
  return 0;
}

