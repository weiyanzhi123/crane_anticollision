#include "crane_anticollision/anti_collision_core.hpp"

#include <algorithm>
#include <cmath>

namespace crane_anticollision {

namespace {
constexpr double kDeg2Rad = M_PI / 180.0;

double clampDouble(double v, double lo, double hi) {
  return std::max(lo, std::min(hi, v));
}

double segmentSegmentDistance(const Eigen::Vector3d& p1, const Eigen::Vector3d& q1,
                              const Eigen::Vector3d& p2, const Eigen::Vector3d& q2) {
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

double pointSegmentDistance(const Eigen::Vector3d& p, const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
  const Eigen::Vector3d ab = b - a;
  const double ab2 = ab.squaredNorm();
  if (ab2 <= 1e-12) return (p - a).norm();
  const double t = clampDouble((p - a).dot(ab) / ab2, 0.0, 1.0);
  const Eigen::Vector3d q = a + t * ab;
  return (p - q).norm();
}

double unwrapAngle(double prev, double curr) {
  double diff = curr - prev;
  if (diff > 180.0) curr -= 360.0;
  else if (diff < -180.0) curr += 360.0;
  return curr;
}

double estimateVelocity(const std::deque<double>& history, double dt) {
  if (history.size() < 2) return 0.0;
  if (history.size() == 2) {
    return (history.back() - history.front()) / (dt * (history.size() - 1));
  }
  double sum_t = 0.0, sum_x = 0.0, sum_tx = 0.0, sum_t2 = 0.0;
  for (std::size_t i = 0; i < history.size(); ++i) {
    const double t = static_cast<double>(i) * dt;
    const double x = history[i];
    sum_t += t;
    sum_x += x;
    sum_tx += t * x;
    sum_t2 += t * t;
  }
  const std::size_t n = history.size();
  const double denom = static_cast<double>(n) * sum_t2 - sum_t * sum_t;
  if (std::abs(denom) < 1e-9) return 0.0;
  return (static_cast<double>(n) * sum_tx - sum_t * sum_x) / denom;
}
}  // namespace

bool AntiCollisionCore::init(const Config& cfg) {
  cfg_ = cfg;
  computeRMax(cfg_.crane_a);
  computeRMax(cfg_.crane_b);

  crane_a_state_ = CraneState{};
  crane_b_state_ = CraneState{};
  return true;
}

void AntiCollisionCore::computeRMax(CraneParams& p) {
  // 与旧实现保持一致
  p.r_max = p.arm_length + p.arm_radius + p.hook_radius + 2.0;
}

void AntiCollisionCore::updateCraneA(const Observation& obs) {
  updateState(crane_a_state_, obs, cfg_.crane_a);
}

void AntiCollisionCore::updateCraneB(const Observation& obs) {
  updateState(crane_b_state_, obs, cfg_.crane_b);
}

void AntiCollisionCore::updateState(CraneState& state, const Observation& obs, const CraneParams& params) {
  if (!obs.valid) {
    state.valid = false;
    return;
  }

  const double now = obs.t_sec;
  double dt = 0.0;
  if (state.has_last_update) {
    dt = now - state.last_update_sec;
  }
  state.has_last_update = true;
  state.last_update_sec = now;

  double theta_raw = obs.theta_deg + params.theta_offset_deg;
  if (state.valid && dt > 0.0 && dt < 1.0) {
    theta_raw = unwrapAngle(state.theta_deg, theta_raw);
  }

  state.theta_deg = theta_raw;
  state.r = obs.r;
  state.z = obs.z;
  state.valid = true;

  if (dt > 0.0 && dt < 1.0) {
    state.theta_history.push_back(state.theta_deg);
    state.r_history.push_back(state.r);
    state.z_history.push_back(state.z);

    while (state.theta_history.size() > cfg_.velocity_window_size) state.theta_history.pop_front();
    while (state.r_history.size() > cfg_.velocity_window_size) state.r_history.pop_front();
    while (state.z_history.size() > cfg_.velocity_window_size) state.z_history.pop_front();

    if (state.theta_history.size() >= 2) {
      state.theta_dot_deg_per_sec = estimateVelocity(state.theta_history, dt);
      state.theta_dot_deg_per_sec = clampDouble(state.theta_dot_deg_per_sec,
                                                -cfg_.max_theta_dot_deg_per_sec,
                                                cfg_.max_theta_dot_deg_per_sec);
    }
    if (state.r_history.size() >= 2) {
      state.r_dot_m_per_sec = estimateVelocity(state.r_history, dt);
      state.r_dot_m_per_sec = clampDouble(state.r_dot_m_per_sec,
                                          -cfg_.max_r_dot_m_per_sec,
                                          cfg_.max_r_dot_m_per_sec);
    }
    if (state.z_history.size() >= 2) {
      state.z_dot_m_per_sec = estimateVelocity(state.z_history, dt);
      state.z_dot_m_per_sec = clampDouble(state.z_dot_m_per_sec,
                                          -cfg_.max_z_dot_m_per_sec,
                                          cfg_.max_z_dot_m_per_sec);
    }
  }
}

AntiCollisionCore::Result AntiCollisionCore::step(double /*now_sec*/) {
  Result out;

  if (!crane_a_state_.valid || !crane_b_state_.valid) {
    // 默认安全
    out.a = PerCraneDecision{};
    out.b = PerCraneDecision{};
    if (cb_) cb_(out);
    return out;
  }

  // 快速粗筛
  const Eigen::Vector3d base_a = cfg_.crane_a.base_xyz;
  const Eigen::Vector3d base_b = cfg_.crane_b.base_xyz;
  const double d_CC = (base_b - base_a).head<2>().norm();
  if (d_CC > cfg_.crane_a.r_max + cfg_.crane_b.r_max) {
    if (cb_) cb_(out);
    return out;
  }

  // 精确计算当前距离
  const double d_min = computeMinDistance();

  // 短时预测（考虑减速过程）
  double d_min_pred = d_min;
  double ttc = std::numeric_limits<double>::infinity();

  // 与原实现保持一致：theta 的减速度会乘 kDeg2Rad
  const double decel_theta = cfg_.prediction_decel_theta_deg_per_sec2 * kDeg2Rad;
  const double decel_r = cfg_.prediction_decel_r_m_per_sec2;
  const double decel_z = cfg_.prediction_decel_z_m_per_sec2;

  for (double t = 0.0; t <= cfg_.prediction_time; t += cfg_.prediction_step) {
    const double d_t = computeMinDistanceAtTimeWithDecel(t, decel_theta, decel_r, decel_z);
    if (d_t < d_min_pred) d_min_pred = d_t;
    if (d_t < cfg_.D_stop && std::isinf(ttc)) {
      ttc = t;
    }
  }

  const double d_decision = std::min(d_min, d_min_pred);

  // 检查是否有运动趋势（速度不为0）
  const double speed_a = std::sqrt(
      std::pow(crane_a_state_.theta_dot_deg_per_sec, 2) +
      std::pow(crane_a_state_.r_dot_m_per_sec, 2) +
      std::pow(crane_a_state_.z_dot_m_per_sec, 2));
  const double speed_b = std::sqrt(
      std::pow(crane_b_state_.theta_dot_deg_per_sec, 2) +
      std::pow(crane_b_state_.r_dot_m_per_sec, 2) +
      std::pow(crane_b_state_.z_dot_m_per_sec, 2));
  constexpr double kSpeedThreshold = 0.01;
  const bool is_moving_a = (speed_a > kSpeedThreshold);
  const bool is_moving_b = (speed_b > kSpeedThreshold);
  const bool is_moving = is_moving_a || is_moving_b;

  bool having_risk = false;
  bool low_risk = false;
  bool high_risk = false;

  if (is_moving) {
    if (d_decision < cfg_.D_high_risk || (ttc <= cfg_.t_latency + cfg_.t_brake + cfg_.t_margin)) {
      high_risk = true;
      having_risk = true;
    } else if (d_decision < cfg_.D_low_risk) {
      low_risk = true;
      having_risk = true;
    } else if (d_decision < cfg_.D_safe) {
      having_risk = true;
    }
  }

  // 策略3：智能协商避让
  double risk_contribution_a = computeRiskContribution(crane_a_state_, cfg_.crane_a, true);
  double risk_contribution_b = computeRiskContribution(crane_b_state_, cfg_.crane_b, false);
  risk_contribution_a *= (1.0 / cfg_.crane_a_priority);
  risk_contribution_b *= (1.0 / cfg_.crane_b_priority);

  bool stop_a = false, stop_b = false;
  double limit_a = 1.0, limit_b = 1.0;
  bool force_stop_a = false, force_stop_b = false;

  bool moving_toward_risk_a = false, moving_toward_risk_b = false;
  if (is_moving_a && having_risk) moving_toward_risk_a = isMovingTowardRisk(true);
  if (is_moving_b && having_risk) moving_toward_risk_b = isMovingTowardRisk(false);

  if (high_risk && is_moving) {
    if (is_moving_a && moving_toward_risk_a) {
      stop_a = true;
      force_stop_a = true;
      limit_a = 0.0;
    }
    if (is_moving_b && moving_toward_risk_b) {
      stop_b = true;
      force_stop_b = true;
      limit_b = 0.0;
    }
  } else if (low_risk && is_moving) {
    if (is_moving_a && moving_toward_risk_a) limit_a = 0.7;
    if (is_moving_b && moving_toward_risk_b) limit_b = 0.7;
  } else if (having_risk && is_moving) {
    if (risk_contribution_a > risk_contribution_b) {
      if (is_moving_a && moving_toward_risk_a) limit_a = 0.5;
      if (is_moving_b && moving_toward_risk_b) limit_b = 0.7;
    } else {
      if (is_moving_a && moving_toward_risk_a) limit_a = 0.7;
      if (is_moving_b && moving_toward_risk_b) limit_b = 0.5;
    }
  }

  out.d_min = d_min;
  out.d_min_pred = d_min_pred;
  out.ttc = ttc;

  out.a.having_risk = having_risk;
  out.a.need_stop_now = stop_a || force_stop_a;
  out.a.limit = force_stop_a ? 0.0 : limit_a;
  out.a.high_risk = high_risk;
  out.a.low_risk = low_risk;
  out.a.force_stop = force_stop_a;

  out.b.having_risk = having_risk;
  out.b.need_stop_now = stop_b || force_stop_b;
  out.b.limit = force_stop_b ? 0.0 : limit_b;
  out.b.high_risk = high_risk;
  out.b.low_risk = low_risk;
  out.b.force_stop = force_stop_b;

  if (cb_) cb_(out);
  return out;
}

double AntiCollisionCore::computeRiskContribution(const CraneState& state, const CraneParams& params, bool /*is_a*/) const {
  const double speed_magnitude = std::sqrt(
      std::pow(state.theta_dot_deg_per_sec * params.arm_length * kDeg2Rad, 2) +
      std::pow(state.r_dot_m_per_sec, 2) +
      std::pow(state.z_dot_m_per_sec, 2));
  return speed_magnitude;
}

bool AntiCollisionCore::isMovingTowardRisk(bool is_a) {
  constexpr double eps = 1e-3;

  const CraneState state_a_backup = crane_a_state_;
  const CraneState state_b_backup = crane_b_state_;

  const double d_current = computeMinDistance();

  double d_dtheta_a = 0.0, d_dr_a = 0.0, d_dz_a = 0.0;
  double d_dtheta_b = 0.0, d_dr_b = 0.0, d_dz_b = 0.0;

  {
    crane_a_state_.theta_deg += eps;
    const double d_pert = computeMinDistance();
    d_dtheta_a = (d_pert - d_current) / eps;
    crane_a_state_ = state_a_backup;
  }
  {
    crane_a_state_.r += eps;
    const double d_pert = computeMinDistance();
    d_dr_a = (d_pert - d_current) / eps;
    crane_a_state_ = state_a_backup;
  }
  {
    crane_a_state_.z += eps;
    const double d_pert = computeMinDistance();
    d_dz_a = (d_pert - d_current) / eps;
    crane_a_state_ = state_a_backup;
  }

  {
    crane_b_state_.theta_deg += eps;
    const double d_pert = computeMinDistance();
    d_dtheta_b = (d_pert - d_current) / eps;
    crane_b_state_ = state_b_backup;
  }
  {
    crane_b_state_.r += eps;
    const double d_pert = computeMinDistance();
    d_dr_b = (d_pert - d_current) / eps;
    crane_b_state_ = state_b_backup;
  }
  {
    crane_b_state_.z += eps;
    const double d_pert = computeMinDistance();
    d_dz_b = (d_pert - d_current) / eps;
    crane_b_state_ = state_b_backup;
  }

  crane_a_state_ = state_a_backup;
  crane_b_state_ = state_b_backup;

  double d_dt = 0.0;
  if (is_a) {
    d_dt = d_dtheta_a * (crane_a_state_.theta_dot_deg_per_sec * kDeg2Rad) +
           d_dr_a * crane_a_state_.r_dot_m_per_sec +
           d_dz_a * crane_a_state_.z_dot_m_per_sec;
  } else {
    d_dt = d_dtheta_b * (crane_b_state_.theta_dot_deg_per_sec * kDeg2Rad) +
           d_dr_b * crane_b_state_.r_dot_m_per_sec +
           d_dz_b * crane_b_state_.z_dot_m_per_sec;
  }
  return d_dt < -1e-6;
}

double AntiCollisionCore::computeMinDistance() const {
  return computeMinDistanceAtTime(0.0);
}

double AntiCollisionCore::computeMinDistanceAtTime(double t) const {
  return computeMinDistanceAtTimeWithDecel(t, 0.0, 0.0, 0.0);
}

double AntiCollisionCore::computeMinDistanceAtTimeWithDecel(double t, double decel_theta, double decel_r, double decel_z) const {
  const CraneState a_pred = predictStateWithDecel(crane_a_state_, t, cfg_.crane_a, decel_theta, decel_r, decel_z);
  const CraneState b_pred = predictStateWithDecel(crane_b_state_, t, cfg_.crane_b, decel_theta, decel_r, decel_z);

  const bool is_prediction = (t > 0.0);

  const Eigen::Vector3d C_a = cfg_.crane_a.base_xyz + Eigen::Vector3d(0.0, 0.0, cfg_.crane_a.mast_height);
  const Eigen::Vector3d C_b = cfg_.crane_b.base_xyz + Eigen::Vector3d(0.0, 0.0, cfg_.crane_b.mast_height);

  const double theta_a_rad = a_pred.theta_deg * kDeg2Rad;
  const double theta_b_rad = b_pred.theta_deg * kDeg2Rad;

  const Eigen::Vector3d T_a = C_a + cfg_.crane_a.arm_length * Eigen::Vector3d(std::cos(theta_a_rad), std::sin(theta_a_rad), 0.0);
  const Eigen::Vector3d T_b = C_b + cfg_.crane_b.arm_length * Eigen::Vector3d(std::cos(theta_b_rad), std::sin(theta_b_rad), 0.0);

  const Eigen::Vector3d P_a = C_a + a_pred.r * Eigen::Vector3d(std::cos(theta_a_rad), std::sin(theta_a_rad), 0.0);
  const Eigen::Vector3d P_b = C_b + b_pred.r * Eigen::Vector3d(std::cos(theta_b_rad), std::sin(theta_b_rad), 0.0);

  const Eigen::Vector3d H_a(P_a.x(), P_a.y(), a_pred.z);
  const Eigen::Vector3d H_b(P_b.x(), P_b.y(), b_pred.z);

  const double arm_height_a = C_a.z();
  const double arm_height_b = C_b.z();

  double d_min = std::numeric_limits<double>::max();

  if (std::abs(arm_height_a - arm_height_b) < 5.0) {
    const double d_arm_arm = segmentSegmentDistance(C_a, T_a, C_b, T_b) -
                             (cfg_.crane_a.arm_radius + cfg_.crane_b.arm_radius);
    d_min = std::min(d_min, d_arm_arm);
  }

  if (arm_height_a < arm_height_b) {
    const double arm_top_a = arm_height_a + cfg_.crane_a.arm_radius;
    const double rope_bottom_b = std::min(H_b.z(), P_b.z()) - cfg_.crane_b.rope_radius;
    const double hook_bottom_b = H_b.z() - cfg_.crane_b.hook_radius;

    const bool height_check_rope = (rope_bottom_b <= arm_top_a + 2.0);
    const bool height_check_hook = (hook_bottom_b <= arm_top_a + 2.0);

    if (height_check_rope || height_check_hook) {
      const Eigen::Vector2d C_a_2d = C_a.head<2>();
      const Eigen::Vector2d P_b_2d = P_b.head<2>();
      const Eigen::Vector2d H_b_2d = H_b.head<2>();

      const double max_horizontal_reach_a =
          cfg_.crane_a.arm_length + cfg_.crane_a.arm_radius +
          std::max(cfg_.crane_b.rope_radius, cfg_.crane_b.hook_radius);

      bool should_check = false;
      if (is_prediction) {
        const Eigen::Vector2d T_a_2d = T_a.head<2>();
        const double dist_Ta_to_Pb = (T_a_2d - P_b_2d).norm();
        const double dist_Ta_to_Hb = (T_a_2d - H_b_2d).norm();
        const double extended_reach = max_horizontal_reach_a * 2.0;
        should_check = (dist_Ta_to_Pb <= extended_reach || dist_Ta_to_Hb <= extended_reach);
      } else {
        const double dist_Pb_to_Ca = (P_b_2d - C_a_2d).norm();
        const double dist_Hb_to_Ca = (H_b_2d - C_a_2d).norm();
        const double min_horizontal_dist = std::min(dist_Pb_to_Ca, dist_Hb_to_Ca);
        should_check = (min_horizontal_dist <= max_horizontal_reach_a + 5.0);
      }

      if (should_check) {
        if (height_check_rope) {
          const double d_armA_ropeB = segmentSegmentDistance(C_a, T_a, P_b, H_b) -
                                      (cfg_.crane_a.arm_radius + cfg_.crane_b.rope_radius);
          d_min = std::min(d_min, d_armA_ropeB);
        }
        if (height_check_hook) {
          const double d_armA_hookB = pointSegmentDistance(H_b, C_a, T_a) -
                                      (cfg_.crane_a.arm_radius + cfg_.crane_b.hook_radius);
          d_min = std::min(d_min, d_armA_hookB);
        }
      }
    }
  }

  if (arm_height_b < arm_height_a) {
    const double arm_top_b = arm_height_b + cfg_.crane_b.arm_radius;
    const double rope_bottom_a = std::min(H_a.z(), P_a.z()) - cfg_.crane_a.rope_radius;
    const double hook_bottom_a = H_a.z() - cfg_.crane_a.hook_radius;

    const bool height_check_rope = (rope_bottom_a <= arm_top_b + 2.0);
    const bool height_check_hook = (hook_bottom_a <= arm_top_b + 2.0);

    if (height_check_rope || height_check_hook) {
      const Eigen::Vector2d C_b_2d = C_b.head<2>();
      const Eigen::Vector2d P_a_2d = P_a.head<2>();
      const Eigen::Vector2d H_a_2d = H_a.head<2>();

      const double max_horizontal_reach_b =
          cfg_.crane_b.arm_length + cfg_.crane_b.arm_radius +
          std::max(cfg_.crane_a.rope_radius, cfg_.crane_a.hook_radius);

      bool should_check = false;
      if (is_prediction) {
        const Eigen::Vector2d T_b_2d = T_b.head<2>();
        const double dist_Tb_to_Pa = (T_b_2d - P_a_2d).norm();
        const double dist_Tb_to_Ha = (T_b_2d - H_a_2d).norm();
        const double extended_reach = max_horizontal_reach_b * 2.0;
        should_check = (dist_Tb_to_Pa <= extended_reach || dist_Tb_to_Ha <= extended_reach);
      } else {
        const double dist_Pa_to_Cb = (P_a_2d - C_b_2d).norm();
        const double dist_Ha_to_Cb = (H_a_2d - C_b_2d).norm();
        const double min_horizontal_dist = std::min(dist_Pa_to_Cb, dist_Ha_to_Cb);
        should_check = (min_horizontal_dist <= max_horizontal_reach_b + 5.0);
      }

      if (should_check) {
        if (height_check_rope) {
          const double d_armB_ropeA = segmentSegmentDistance(C_b, T_b, P_a, H_a) -
                                      (cfg_.crane_b.arm_radius + cfg_.crane_a.rope_radius);
          d_min = std::min(d_min, d_armB_ropeA);
        }
        if (height_check_hook) {
          const double d_armB_hookA = pointSegmentDistance(H_a, C_b, T_b) -
                                      (cfg_.crane_b.arm_radius + cfg_.crane_a.hook_radius);
          d_min = std::min(d_min, d_armB_hookA);
        }
      }
    }
  }

  return d_min;
}

AntiCollisionCore::CraneState AntiCollisionCore::predictStateWithDecel(const CraneState& state, double t, const CraneParams& params,
                                                                      double decel_theta, double decel_r, double decel_z) const {
  CraneState pred = state;
  if (t <= 0.0) return pred;

  double v_theta_final = state.theta_dot_deg_per_sec;
  double v_r_final = state.r_dot_m_per_sec;
  double v_z_final = state.z_dot_m_per_sec;

  if (decel_theta > 0.0) {
    const double t_stop_theta = std::abs(v_theta_final) / decel_theta;
    if (t <= t_stop_theta) {
      v_theta_final = v_theta_final - (v_theta_final > 0 ? decel_theta : -decel_theta) * t;
    } else {
      v_theta_final = 0.0;
    }
  }

  if (decel_r > 0.0) {
    const double t_stop_r = std::abs(v_r_final) / decel_r;
    if (t <= t_stop_r) {
      v_r_final = v_r_final - (v_r_final > 0 ? decel_r : -decel_r) * t;
    } else {
      v_r_final = 0.0;
    }
  }

  if (decel_z > 0.0) {
    const double t_stop_z = std::abs(v_z_final) / decel_z;
    if (t <= t_stop_z) {
      v_z_final = v_z_final - (v_z_final > 0 ? decel_z : -decel_z) * t;
    } else {
      v_z_final = 0.0;
    }
  }

  (void)v_theta_final;
  (void)v_r_final;
  (void)v_z_final;

  auto computeDistanceWithDecel = [](double v0, double t_local, double decel) -> double {
    if (decel <= 0.0) return v0 * t_local;
    const double t_stop = std::abs(v0) / decel;
    if (t_local <= t_stop) {
      return v0 * t_local - 0.5 * (v0 > 0 ? decel : -decel) * t_local * t_local;
    }
    return v0 * t_stop - 0.5 * (v0 > 0 ? decel : -decel) * t_stop * t_stop;
  };

  pred.theta_deg += computeDistanceWithDecel(state.theta_dot_deg_per_sec, t, decel_theta);
  pred.r = clampDouble(state.r + computeDistanceWithDecel(state.r_dot_m_per_sec, t, decel_r), 0.0, params.arm_length);
  pred.z = std::max(0.0, state.z + computeDistanceWithDecel(state.z_dot_m_per_sec, t, decel_z));
  return pred;
}

}  // namespace crane_anticollision

