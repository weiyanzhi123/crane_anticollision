#pragma once

#include <Eigen/Dense>

#include <cstddef>
#include <cstdint>
#include <deque>
#include <functional>
#include <limits>

namespace crane_anticollision {

class AntiCollisionCore {
public:
  struct CraneParams {
    Eigen::Vector3d base_xyz{0.0, 0.0, 0.0};
    // 塔身高度（用于把回转中心/大臂平面抬到塔顶，需与RViz可视化保持一致）
    double mast_height{45.0};
    // Angle offsets are in radians.
    double theta_offset_rad{0.0};
    double arm_length{50.0};
    double arm_radius{0.5};
    double rope_radius{0.05};
    double hook_radius{0.35};
    // 粗筛用的最大水平可达半径
    double r_max{0.0};
  };

  struct Config {
    // 输入：两台塔吊的结构参数
    CraneParams crane_a;
    CraneParams crane_b;

    // 阈值参数
    double D_safe{10.0};
    double D_warn{5.0};
    double D_stop{2.0};

    // 分级预警阈值
    double D_low_risk{8.0};
    double D_high_risk{3.0};

    // 预测参数
    double prediction_time{2.0};
    double prediction_step{0.05};
    std::size_t velocity_window_size{10};

    // 系统延迟与制动
    double t_latency{0.1};
    double t_brake{0.5};
    double t_margin{0.2};

    // 速度限幅（用于速度估计限幅）
    // Angular velocity limit (rad/s) for estimation clamping.
    double max_theta_dot_rad_per_sec{0.17453292519943295};  // 10 deg/s
    double max_r_dot_m_per_sec{1.0};
    double max_z_dot_m_per_sec{0.5};

    // Movement speed weights (slew/trolley/hoist).
    double speed_weight_slew{1.0};
    double speed_weight_trolley{0.6};
    double speed_weight_hoist{0.3};

    // 优先级（策略3：智能协商避让）
    double crane_a_priority{1.0};
    double crane_b_priority{0.5};

    // Prediction deceleration (rad/s^2) for angular motion.
    double prediction_decel_theta_rad_per_sec2{0.2617993877991494};  // 15 deg/s^2
    double prediction_decel_r_m_per_sec2{1.5};
    double prediction_decel_z_m_per_sec2{0.9};

    // Command-fusion (encoder trend) for prediction.
    bool command_fusion_enabled{true};
    double command_fusion_alpha{0.6};
    double command_fusion_lead_time{0.2};

    // Latency compensation & braking-distance guard
    bool latency_compensation_enabled{true};
    double latency_compensation_time{0.15};
    bool braking_distance_guard_enabled{true};
    double braking_distance_margin{0.5};
  };

  struct Observation {
    // 是否有效：若为 false，core 会认为该塔吊状态无效
    bool valid{false};
    // 时间戳（秒，单调/系统时间均可，但两次 update 需同一时间基准）
    double t_sec{0.0};
    // 极坐标物理量（与现有 JointPoseMsg 对齐）
    // Angles are in radians.
    double theta_rad{0.0};
    double r{0.0};
    double z{0.0};
  };

  struct PerCraneDecision {
    bool having_risk{false};
    bool need_stop_now{false};
    double limit{1.0};
    bool high_risk{false};
    bool low_risk{false};
    bool force_stop{false};
  };

  struct Result {
    PerCraneDecision a;
    PerCraneDecision b;

    // Debug/观测用（可选）
    double d_min{std::numeric_limits<double>::infinity()};
    double d_min_pred{std::numeric_limits<double>::infinity()};
    double ttc{std::numeric_limits<double>::infinity()};
  };

  struct DistanceResult {
    double min{std::numeric_limits<double>::infinity()};
    double arm_arm{std::numeric_limits<double>::infinity()};
    double arm_rope{std::numeric_limits<double>::infinity()};
    double arm_hook{std::numeric_limits<double>::infinity()};
  };

  using ResultCallback = std::function<void(const Result&)>;

  AntiCollisionCore() = default;

  // 初始化/重新初始化（会清空内部状态）
  bool init(const Config& cfg);

  void setResultCallback(ResultCallback cb) { cb_ = std::move(cb); }

  void updateCraneA(const Observation& obs);
  void updateCraneB(const Observation& obs);

  // 运行一次逻辑，返回结果
  Result step(double now_sec);

private:
  struct CraneState {
    bool valid{false};
    double theta_rad{0.0};
    double r{0.0};
    double z{0.0};

    std::deque<double> theta_history;
    std::deque<double> r_history;
    std::deque<double> z_history;

    double theta_dot_rad_per_sec{0.0};
    double r_dot_m_per_sec{0.0};
    double z_dot_m_per_sec{0.0};

    double theta_dot_pred_rad_per_sec{0.0};
    double r_dot_pred_m_per_sec{0.0};
    double z_dot_pred_m_per_sec{0.0};

    double theta_ddot_rad_per_sec2{0.0};
    double r_ddot_m_per_sec2{0.0};
    double z_ddot_m_per_sec2{0.0};

    bool has_last_velocity{false};
    double last_theta_dot_rad_per_sec{0.0};
    double last_r_dot_m_per_sec{0.0};
    double last_z_dot_m_per_sec{0.0};

    bool has_last_update{false};
    double last_update_sec{0.0};
  };

  void updateState(CraneState& state, const Observation& obs, const CraneParams& params);

  double computeRiskContribution(const CraneState& state, const CraneParams& params, bool is_a) const;
  bool isMovingTowardRisk(bool is_a);

  double computeMinDistance() const;
  double computeMinDistanceAtTime(double t) const;
  double computeMinDistanceAtTimeWithDecel(double t, double decel_theta, double decel_r, double decel_z) const;
  DistanceResult computeMinDistanceDetailsAtTimeWithDecel(double t, double decel_theta, double decel_r, double decel_z) const;

  CraneState predictStateWithDecel(const CraneState& state, double t, const CraneParams& params,
                                  double decel_theta, double decel_r, double decel_z) const;

  static void computeRMax(CraneParams& p);

private:
  Config cfg_{};
  CraneState crane_a_state_{};
  CraneState crane_b_state_{};
  ResultCallback cb_{};
};

}  // namespace crane_anticollision

