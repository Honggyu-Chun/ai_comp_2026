#include <ros/ros.h>

#include <algorithm>
#include <cmath>
#include <iomanip>
#include <limits>
#include <string>
#include <vector>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <morai_msgs/CtrlCmd.h>
#include <nav_msgs/Odometry.h>
#include <osqp/osqp.h>
#include <nav_msgs/Path.h>
#include <std_msgs/Float64.h>
#include <std_msgs/String.h>

namespace {

const double kPi = 3.14159265358979323846;

double clampValue(const double value, const double min_value, const double max_value)
{
    return std::max(min_value, std::min(value, max_value));
}

double normalizeAngle(double angle)
{
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }
    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }
    return angle;
}

double displayValue(const double value, const double deadband = 0.005)
{
    return (std::abs(value) < deadband) ? 0.0 : value;
}

struct SparseCscData
{
    std::vector<c_float> values;
    std::vector<c_int> row_indices;
    std::vector<c_int> col_pointers;
};

void addQuadraticTerm(std::vector<std::vector<double> >& hessian,
                      std::vector<double>& gradient,
                      const std::vector<double>& coeff,
                      const double offset,
                      const double weight)
{
    if (weight <= 0.0) {
        return;
    }

    const std::size_t n = coeff.size();
    for (std::size_t i = 0; i < n; ++i) {
        if (std::abs(coeff[i]) < 1e-12) {
            continue;
        }
        gradient[i] += 2.0 * weight * offset * coeff[i];
        for (std::size_t j = 0; j < n; ++j) {
            if (std::abs(coeff[j]) < 1e-12) {
                continue;
            }
            hessian[i][j] += 2.0 * weight * coeff[i] * coeff[j];
        }
    }
}

SparseCscData denseUpperToCsc(const std::vector<std::vector<double> >& matrix)
{
    SparseCscData csc;
    const int n = static_cast<int>(matrix.size());
    csc.col_pointers.reserve(n + 1);
    csc.col_pointers.push_back(0);
    for (int col = 0; col < n; ++col) {
        for (int row = 0; row <= col; ++row) {
            const double value = matrix[row][col];
            if (std::abs(value) < 1e-10) {
                continue;
            }
            csc.values.push_back(static_cast<c_float>(value));
            csc.row_indices.push_back(static_cast<c_int>(row));
        }
        csc.col_pointers.push_back(static_cast<c_int>(csc.values.size()));
    }
    return csc;
}

SparseCscData denseToCsc(const std::vector<std::vector<double> >& matrix)
{
    SparseCscData csc;
    const int rows = static_cast<int>(matrix.size());
    const int cols = rows > 0 ? static_cast<int>(matrix.front().size()) : 0;
    csc.col_pointers.reserve(cols + 1);
    csc.col_pointers.push_back(0);
    for (int col = 0; col < cols; ++col) {
        for (int row = 0; row < rows; ++row) {
            const double value = matrix[row][col];
            if (std::abs(value) < 1e-10) {
                continue;
            }
            csc.values.push_back(static_cast<c_float>(value));
            csc.row_indices.push_back(static_cast<c_int>(row));
        }
        csc.col_pointers.push_back(static_cast<c_int>(csc.values.size()));
    }
    return csc;
}

struct PathPoint
{
    double x;
    double y;
    double yaw;
    double s;
    double curvature;
    double speed_limit_mps;
};

struct PathData
{
    std::vector<PathPoint> points;
    double total_length;
    bool external_profile;   // true = 판단(z)이 준 목표속도 프로파일 → 순수 추종

    PathData() : total_length(0.0), external_profile(false) {}
};

struct ReferencePoint
{
    double x;
    double y;
    double yaw;
    double s;
    double curvature;
    double speed_limit_mps;
};

struct MpcResult
{
    bool valid;
    double steer_cmd;
    double accel_cmd_mps2;
    double initial_cte;
    double initial_heading_error;
    double preview_curvature;
    double published_steer_cmd;
    double published_accel_cmd;
    double published_brake_cmd;
    double published_accel_mps2;
    double profile_speed_kph;
    double full_accel_limit_mps2;
    double speed_profile_slack_kph;
    bool steer_saturated;
    bool accel_limited;
    std::vector<double> steer_sequence;
    std::vector<double> accel_sequence;

    MpcResult()
        : valid(false),
          steer_cmd(0.0),
          accel_cmd_mps2(0.0),
          initial_cte(0.0),
          initial_heading_error(0.0),
          preview_curvature(0.0),
          published_steer_cmd(0.0),
          published_accel_cmd(0.0),
          published_brake_cmd(0.0),
          published_accel_mps2(0.0),
          profile_speed_kph(0.0),
          full_accel_limit_mps2(0.0),
          speed_profile_slack_kph(0.0),
          steer_saturated(false),
          accel_limited(false)
    {
    }
};

}  // namespace

class FullMpcControl {
public:
    FullMpcControl();

    void run();

private:
    ros::NodeHandle nh_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher ctrl_cmd_pub_;
    ros::Publisher rmse_pub_;
    ros::Publisher curvature_pub_;
    ros::Publisher hde_pub_;
    ros::Publisher cte_pub_;
    ros::Publisher target_accel_pub_;
    ros::Publisher penalty_pub_;

    ros::Subscriber gps_sub_;
    ros::Subscriber ego_vehicle_sub_;
    ros::Subscriber steer_feedback_sub_;
    ros::Subscriber local_path_sub_;
    ros::Subscriber map_state_sub_;
    ros::Subscriber car_state_sub_;
    ros::Subscriber lap_tuner_debug_sub_;

    nav_msgs::Path local_path_;
    geometry_msgs::Twist cmd_vel_msg_;
    morai_msgs::CtrlCmd ctrl_cmd_msg_;

    bool gps_state_;
    bool path_state_;
    bool vehicle_info_;
    bool use_morai_sim_;
    bool use_morai_steer_feedback_;
    bool use_measured_longitudinal_model_;
    bool enforce_speed_profile_;
    bool show_lap_tuner_debug_;

    std::string mode_;
    std::string morai_steer_feedback_unit_;
    std::string gps_topic_;
    std::string local_path_topic_;
    std::string map_state_topic_;
    std::string car_state_topic_;
    std::string real_velocity_topic_;
    std::string morai_speed_topic_;
    std::string morai_steer_feedback_topic_;
    std::string cmd_vel_topic_;
    std::string ctrl_cmd_topic_;
    std::string lap_tuner_debug_topic_;
    std::string map_state_;
    std::string car_state_;
    std::string lap_tuner_debug_text_;

    int control_frequency_;
    int morai_longitudinal_cmd_type_;
    double dt_control_;
    double current_speed_kph_;
    double current_speed_mps_;
    double morai_velocity_to_kph_scale_;
    double morai_steer_feedback_sign_;
    double last_steer_cmd_;
    double last_accel_cmd_mps2_;
    double last_accel_pedal_cmd_;
    double last_brake_pedal_cmd_;
    double estimated_steer_;
    double cte_square_sum_;
    int cte_count_;
    double status_print_hz_;
    double tunable_param_reload_hz_;
    ros::Time last_status_print_time_;
    ros::Time last_tunable_reload_time_;

    double wheel_base_;
    bool use_steer_model_gain_;
    double steer_model_gain_straight_;
    double steer_model_gain_per_curvature_;
    double tracking_point_offset_;
    double max_steer_rad_;
    double max_steer_rate_rad_;
    bool use_speed_adaptive_steering_;
    double steering_schedule_min_speed_kph_;
    double steering_schedule_max_speed_kph_;
    double low_speed_max_steer_rate_rad_;
    double high_speed_max_steer_rate_rad_;
    double low_speed_steering_time_constant_;
    double high_speed_steering_time_constant_;
    double max_speed_kph_;
    double max_accel_mps2_;
    double max_decel_mps2_;
    double max_jerk_mps3_;
    double accel_pedal_full_scale_mps2_;
    double brake_pedal_full_scale_mps2_;
    double max_accel_pedal_cmd_;
    double max_brake_pedal_cmd_;
    double steering_time_constant_;
    double lateral_accel_limit_mps2_;
    double curve_speed_safety_factor_;
    double path_resolution_min_;
    double curvature_smoothing_window_m_;
    double preview_time_s_;
    double preview_min_distance_m_;
    double preview_max_distance_m_;
    double current_preview_distance_m_;
    double measured_brake_decel_mps2_;
    double longitudinal_latency_s_;
    double minimum_curve_speed_kph_;
    double decision_slow_down_speed_kph_;
    double speed_profile_tolerance_kph_;
    std::vector<double> full_accel_by_speed_mps2_;

    int horizon_steps_;
    double mpc_dt_;
    double weight_cte_;
    double weight_heading_;
    double weight_speed_;
    double weight_steer_;
    double weight_steer_rate_;
    double weight_steer_accel_;
    double weight_accel_;
    double weight_jerk_;
    double weight_cte_speed_;
    double terminal_weight_;
    double weight_speed_profile_slack_;

    double curvature_estimate_;
    double estimate_uncertainty_;
    double measurement_noise_;
    double process_noise_;

    std::vector<double> warm_steer_;
    std::vector<double> warm_accel_;
    void gpsCallback(const nav_msgs::Odometry::ConstPtr& msg);
    void egoVehicleRealCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg);
    void speedMoraiCallback(const std_msgs::Float64::ConstPtr& msg);
    void steerFeedbackMoraiCallback(const std_msgs::Float64::ConstPtr& msg);
    void localPathCallback(const nav_msgs::Path::ConstPtr& msg);
    void mapStateCallback(const std_msgs::String::ConstPtr& msg);
    void carStateCallback(const std_msgs::String::ConstPtr& msg);
    void lapTunerDebugCallback(const std_msgs::String::ConstPtr& msg);

    PathData buildPathData(const nav_msgs::Path& path) const;
    std::size_t findClosestPathIndex(const PathData& path, double x, double y) const;
    ReferencePoint samplePathAtS(const PathData& path, double s) const;
    double computePreviewCurvature(const PathData& path, double start_s, double preview_distance) const;
    double applyCurvatureFilter(double measured_curvature);
    void applyBrakingSpeedProfile(PathData* path) const;
    double fullAccelLimitMps2(double speed_kph) const;
    bool isStopDecision() const;
    bool isSlowDownDecision() const;
    double decisionMaxSpeedMps() const;
    double steeringScheduleRatio() const;
    double scheduledMaxSteerRateRad() const;
    double scheduledSteeringTimeConstant() const;
    double steerModelGain(double curvature) const;
    void reloadTunableParameters(bool force = false);

    MpcResult solveMpc(const PathData& path, double preview_curvature);
    void shiftWarmStart(const MpcResult& result);
    void resetWarmStart();

    void publishControlCommand(double accel_mps2, double steer_cmd, MpcResult* result);
    void publishStopCommand();
    void publishDebugSignals(double curvature, double heading_error, double cross_track_error);
    void printStatus(const MpcResult& result) const;
};

FullMpcControl::FullMpcControl()
    : nh_("~"),
      gps_state_(false),
      path_state_(false),
      vehicle_info_(false),
      use_morai_sim_(false),
      use_morai_steer_feedback_(true),
      use_measured_longitudinal_model_(true),
      enforce_speed_profile_(true),
      show_lap_tuner_debug_(true),
      mode_("morai"),
      morai_steer_feedback_unit_("rad"),
      lap_tuner_debug_topic_("/mpc_lap_tuner/debug_text"),
      map_state_("go"),
      car_state_("fast"),
      lap_tuner_debug_text_(""),
      control_frequency_(50),
      morai_longitudinal_cmd_type_(1),
      dt_control_(1.0 / 50.0),
      current_speed_kph_(0.0),
      current_speed_mps_(0.0),
      morai_velocity_to_kph_scale_(1.0),
      morai_steer_feedback_sign_(1.0),
      last_steer_cmd_(0.0),
      last_accel_cmd_mps2_(0.0),
      last_accel_pedal_cmd_(0.0),
      last_brake_pedal_cmd_(0.0),
      estimated_steer_(0.0),
      cte_square_sum_(0.0),
      cte_count_(0),
      status_print_hz_(5.0),
      tunable_param_reload_hz_(2.0),
      wheel_base_(3.0),
      use_steer_model_gain_(false),
      steer_model_gain_straight_(1.0),
      steer_model_gain_per_curvature_(0.0),
      tracking_point_offset_(0.0),
      max_steer_rad_(45.0 * kPi / 180.0),
      max_steer_rate_rad_(75.0 * kPi / 180.0),
      use_speed_adaptive_steering_(false),
      steering_schedule_min_speed_kph_(15.0),
      steering_schedule_max_speed_kph_(60.0),
      low_speed_max_steer_rate_rad_(75.0 * kPi / 180.0),
      high_speed_max_steer_rate_rad_(40.0 * kPi / 180.0),
      low_speed_steering_time_constant_(0.28),
      high_speed_steering_time_constant_(0.50),
      max_speed_kph_(60.0),
      max_accel_mps2_(5.3),
      max_decel_mps2_(9.0),
      max_jerk_mps3_(1000.0),
      accel_pedal_full_scale_mps2_(5.3),
      brake_pedal_full_scale_mps2_(8.99),
      max_accel_pedal_cmd_(1.0),
      max_brake_pedal_cmd_(1.0),
      steering_time_constant_(0.28),
      lateral_accel_limit_mps2_(2.2),
      curve_speed_safety_factor_(1.0),
      path_resolution_min_(0.20),
      curvature_smoothing_window_m_(1.00),
      preview_time_s_(1.2),
      preview_min_distance_m_(5.0),
      preview_max_distance_m_(20.0),
      current_preview_distance_m_(5.0),
      measured_brake_decel_mps2_(8.99),
      longitudinal_latency_s_(0.03),
      minimum_curve_speed_kph_(3.0),
      decision_slow_down_speed_kph_(20.0),
      speed_profile_tolerance_kph_(0.0),
      full_accel_by_speed_mps2_({5.27, 4.84, 3.64, 3.56, 3.55, 3.50}),
      horizon_steps_(25),
      mpc_dt_(0.10),
      weight_cte_(120.0),
      weight_heading_(28.0),
      weight_speed_(25.0),
      weight_steer_(0.14),
      weight_steer_rate_(16.0),
      weight_steer_accel_(55.0),
      weight_accel_(0.0),
      weight_jerk_(0.0),
      weight_cte_speed_(5.0),
      terminal_weight_(24.0),
      weight_speed_profile_slack_(10000.0),
      curvature_estimate_(0.0),
      estimate_uncertainty_(1.0),
      measurement_noise_(0.02),
      process_noise_(0.006)
{
    nh_.param<std::string>("mode", mode_, std::string("morai"));
    use_morai_sim_ = (mode_ == "morai" || mode_ == "morai_sim");

    nh_.param<std::string>("gps_topic", gps_topic_, std::string("/gps_utm_odom"));
    nh_.param<std::string>("local_path_topic", local_path_topic_, std::string("/selected_path"));
    nh_.param<std::string>("map_state_topic", map_state_topic_, std::string("/state"));
    nh_.param<std::string>("car_state_topic", car_state_topic_, std::string("/moving_obs"));
    nh_.param<std::string>("real_velocity_topic", real_velocity_topic_, std::string("/gps_data/fix_velocity"));
    // competition UDP bridge 토픽. /morai/ego_topic(EgoVehicleStatus) 은 더 이상 발행되지 않는다.
    nh_.param<std::string>("morai_speed_topic", morai_speed_topic_, std::string("/current_speed"));
    nh_.param<std::string>("morai_steer_feedback_topic", morai_steer_feedback_topic_, std::string("/vehicle/front_steer_angle"));
    nh_.param<std::string>("cmd_vel_topic", cmd_vel_topic_, std::string("/cmd_vel"));
    nh_.param<std::string>("ctrl_cmd_topic", ctrl_cmd_topic_, std::string("/ctrl_cmd"));
    nh_.param("show_lap_tuner_debug", show_lap_tuner_debug_, true);
    nh_.param<std::string>("lap_tuner_debug_topic", lap_tuner_debug_topic_, std::string("/mpc_lap_tuner/debug_text"));
    // /current_speed 는 이미 m/s. 이 값은 속도 단위 보정용 배율(기본 1.0 = 그대로).
    nh_.param("morai_velocity_to_kph_scale", morai_velocity_to_kph_scale_, 1.0);
    nh_.param("morai_longitudinal_cmd_type", morai_longitudinal_cmd_type_, 1);
    morai_longitudinal_cmd_type_ = static_cast<int>(clampValue(morai_longitudinal_cmd_type_, 1, 3));
    nh_.param("use_morai_steer_feedback", use_morai_steer_feedback_, true);
    nh_.param<std::string>("morai_steer_feedback_unit", morai_steer_feedback_unit_, std::string("rad"));
    nh_.param("morai_steer_feedback_sign", morai_steer_feedback_sign_, 1.0);
    nh_.param("control_frequency", control_frequency_, 50);
    control_frequency_ = std::max(5, control_frequency_);
    dt_control_ = 1.0 / static_cast<double>(control_frequency_);
    nh_.param("status_print_hz", status_print_hz_, 5.0);
    status_print_hz_ = std::max(0.0, status_print_hz_);
    nh_.param("tunable_param_reload_hz", tunable_param_reload_hz_, 2.0);
    tunable_param_reload_hz_ = std::max(0.0, tunable_param_reload_hz_);

    nh_.param("wheel_base", wheel_base_, 3.0);
    nh_.param("use_steer_model_gain", use_steer_model_gain_, false);
    nh_.param("steer_model_gain_straight", steer_model_gain_straight_, 1.0);
    nh_.param("steer_model_gain_per_curvature", steer_model_gain_per_curvature_, 0.0);
    steer_model_gain_straight_ = std::max(0.1, steer_model_gain_straight_);
    steer_model_gain_per_curvature_ = std::max(0.0, steer_model_gain_per_curvature_);
    nh_.param("tracking_point_offset", tracking_point_offset_, 0.0);
    nh_.param("max_steer_deg", max_steer_rad_, 45.0);
    max_steer_rad_ *= kPi / 180.0;
    nh_.param("max_steer_rate_degps", max_steer_rate_rad_, 75.0);
    max_steer_rate_rad_ *= kPi / 180.0;
    nh_.param("use_speed_adaptive_steering", use_speed_adaptive_steering_, false);
    nh_.param("steering_schedule_min_speed_kph", steering_schedule_min_speed_kph_, 15.0);
    nh_.param("steering_schedule_max_speed_kph", steering_schedule_max_speed_kph_, 60.0);
    double low_speed_max_steer_rate_degps = max_steer_rate_rad_ * 180.0 / kPi;
    double high_speed_max_steer_rate_degps = max_steer_rate_rad_ * 180.0 / kPi;
    nh_.param("low_speed_max_steer_rate_degps", low_speed_max_steer_rate_degps, low_speed_max_steer_rate_degps);
    nh_.param("high_speed_max_steer_rate_degps", high_speed_max_steer_rate_degps, high_speed_max_steer_rate_degps);
    low_speed_max_steer_rate_rad_ = std::max(1.0, low_speed_max_steer_rate_degps) * kPi / 180.0;
    high_speed_max_steer_rate_rad_ = std::max(1.0, high_speed_max_steer_rate_degps) * kPi / 180.0;
    steering_schedule_max_speed_kph_ = std::max(steering_schedule_min_speed_kph_ + 1e-3,
                                                steering_schedule_max_speed_kph_);
    nh_.param("max_speed_kph", max_speed_kph_, 60.0);
    nh_.param("max_accel_mps2", max_accel_mps2_, 5.3);
    nh_.param("max_decel_mps2", max_decel_mps2_, 9.0);
    nh_.param("max_jerk_mps3", max_jerk_mps3_, 1000.0);
    nh_.param("accel_pedal_full_scale_mps2", accel_pedal_full_scale_mps2_, 5.3);
    nh_.param("brake_pedal_full_scale_mps2", brake_pedal_full_scale_mps2_, 8.99);
    nh_.param("max_accel_pedal_cmd", max_accel_pedal_cmd_, 1.0);
    nh_.param("max_brake_pedal_cmd", max_brake_pedal_cmd_, 1.0);
    max_accel_pedal_cmd_ = clampValue(max_accel_pedal_cmd_, 0.0, 1.0);
    max_brake_pedal_cmd_ = clampValue(max_brake_pedal_cmd_, 0.0, 1.0);
    nh_.param("steering_time_constant", steering_time_constant_, 0.28);
    nh_.param("low_speed_steering_time_constant", low_speed_steering_time_constant_, steering_time_constant_);
    nh_.param("high_speed_steering_time_constant", high_speed_steering_time_constant_, steering_time_constant_);
    low_speed_steering_time_constant_ = std::max(0.03, low_speed_steering_time_constant_);
    high_speed_steering_time_constant_ = std::max(0.03, high_speed_steering_time_constant_);
    nh_.param("lateral_accel_limit_mps2", lateral_accel_limit_mps2_, 2.2);
    nh_.param("curve_speed_safety_factor", curve_speed_safety_factor_, 1.0);
    curve_speed_safety_factor_ = clampValue(curve_speed_safety_factor_, 0.30, 1.00);
    nh_.param("path_resolution_min", path_resolution_min_, 0.20);
    nh_.param("curvature_smoothing_window_m", curvature_smoothing_window_m_, 1.00);
    nh_.param("preview_time_s", preview_time_s_, 1.2);
    nh_.param("preview_min_distance_m", preview_min_distance_m_, 5.0);
    nh_.param("preview_max_distance_m", preview_max_distance_m_, 20.0);
    nh_.param("use_measured_longitudinal_model", use_measured_longitudinal_model_, true);
    nh_.param("measured_brake_decel_mps2", measured_brake_decel_mps2_, 8.99);
    nh_.param("longitudinal_latency_s", longitudinal_latency_s_, 0.03);
    nh_.param("minimum_curve_speed_kph", minimum_curve_speed_kph_, 3.0);
    nh_.param("decision_slow_down_speed_kph", decision_slow_down_speed_kph_, 20.0);
    nh_.param("enforce_speed_profile", enforce_speed_profile_, true);
    nh_.param("speed_profile_tolerance_kph", speed_profile_tolerance_kph_, 0.0);
    std::vector<double> configured_full_accel;
    if (nh_.getParam("full_accel_by_speed_mps2", configured_full_accel) &&
        configured_full_accel.size() == 6) {
        full_accel_by_speed_mps2_ = configured_full_accel;
    } else if (!configured_full_accel.empty()) {
        ROS_WARN("full_accel_by_speed_mps2 must contain 6 values for 0-10 ... 50-60 kph; using measured defaults");
    }
    preview_time_s_ = std::max(0.1, preview_time_s_);
    preview_min_distance_m_ = std::max(0.0, preview_min_distance_m_);
    preview_max_distance_m_ = std::max(preview_min_distance_m_, preview_max_distance_m_);
    measured_brake_decel_mps2_ = std::max(0.1, measured_brake_decel_mps2_);
    longitudinal_latency_s_ = std::max(0.0, longitudinal_latency_s_);
    minimum_curve_speed_kph_ = clampValue(minimum_curve_speed_kph_, 0.0, max_speed_kph_);
    decision_slow_down_speed_kph_ =
        clampValue(decision_slow_down_speed_kph_, minimum_curve_speed_kph_, max_speed_kph_);
    speed_profile_tolerance_kph_ = std::max(0.0, speed_profile_tolerance_kph_);

    nh_.param("horizon_steps", horizon_steps_, 25);
    nh_.param("mpc_dt", mpc_dt_, 0.10);
    nh_.param("weight_cte", weight_cte_, 120.0);
    nh_.param("weight_heading", weight_heading_, 28.0);
    nh_.param("weight_speed", weight_speed_, 25.0);
    nh_.param("weight_steer", weight_steer_, 0.14);
    nh_.param("weight_steer_rate", weight_steer_rate_, 16.0);
    nh_.param("weight_steer_accel", weight_steer_accel_, 55.0);
    nh_.param("weight_accel", weight_accel_, 0.0);
    nh_.param("weight_jerk", weight_jerk_, 0.0);
    nh_.param("weight_cte_speed", weight_cte_speed_, 5.0);
    nh_.param("terminal_weight", terminal_weight_, 24.0);
    nh_.param("weight_speed_profile_slack", weight_speed_profile_slack_, 10000.0);
    weight_speed_profile_slack_ = std::max(1.0, weight_speed_profile_slack_);

    horizon_steps_ = clampValue(horizon_steps_, 8, 40);
    gps_sub_ = nh_.subscribe(gps_topic_, 1, &FullMpcControl::gpsCallback, this);
    local_path_sub_ = nh_.subscribe(local_path_topic_, 1, &FullMpcControl::localPathCallback, this);
    map_state_sub_ = nh_.subscribe(map_state_topic_, 1, &FullMpcControl::mapStateCallback, this);
    car_state_sub_ = nh_.subscribe(car_state_topic_, 1, &FullMpcControl::carStateCallback, this);
    if (show_lap_tuner_debug_) {
        lap_tuner_debug_sub_ =
            nh_.subscribe(lap_tuner_debug_topic_, 1, &FullMpcControl::lapTunerDebugCallback, this);
    }

    if (use_morai_sim_) {
        ego_vehicle_sub_ = nh_.subscribe(morai_speed_topic_, 1, &FullMpcControl::speedMoraiCallback, this);
        if (use_morai_steer_feedback_) {
            steer_feedback_sub_ = nh_.subscribe(morai_steer_feedback_topic_, 1,
                                                &FullMpcControl::steerFeedbackMoraiCallback, this);
        }
        ctrl_cmd_pub_ = nh_.advertise<morai_msgs::CtrlCmd>(ctrl_cmd_topic_, 1);
        ctrl_cmd_msg_ = morai_msgs::CtrlCmd();
        ctrl_cmd_msg_.longlCmdType = morai_longitudinal_cmd_type_;
    } else {
        ego_vehicle_sub_ = nh_.subscribe(real_velocity_topic_, 1, &FullMpcControl::egoVehicleRealCallback, this);
        cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>(cmd_vel_topic_, 1);
        cmd_vel_msg_ = geometry_msgs::Twist();
    }

    rmse_pub_ = nh_.advertise<std_msgs::Float64>("/rmse", 10);
    curvature_pub_ = nh_.advertise<std_msgs::Float64>("/curvature", 10);
    hde_pub_ = nh_.advertise<std_msgs::Float64>("/heading_error", 10);
    cte_pub_ = nh_.advertise<std_msgs::Float64>("/cross_track_error", 10);
    target_accel_pub_ = nh_.advertise<std_msgs::Float64>("/target_accel", 10);
    penalty_pub_ = nh_.advertise<std_msgs::Float64>("/penalty", 10);

    resetWarmStart();

    ROS_INFO("FullMpcControl mode: %s", mode_.c_str());
    ROS_INFO("FullMpcControl morai_velocity_to_kph_scale: %.3f", morai_velocity_to_kph_scale_);
    ROS_INFO("FullMpcControl MORAI longitudinal command type: %d", morai_longitudinal_cmd_type_);
    ROS_INFO("FullMpcControl wheel_base: %.3f m, max_speed: %.2f kph", wheel_base_, max_speed_kph_);
    if (use_steer_model_gain_) {
        ROS_INFO("Steer model gain: gain = %.3f + %.3f * abs(curvature)",
                 steer_model_gain_straight_,
                 steer_model_gain_per_curvature_);
    }
    ROS_INFO("FullMpcControl horizon: %d x %.3f s, frequency: %d Hz", horizon_steps_, mpc_dt_, control_frequency_);
    if (use_speed_adaptive_steering_) {
        ROS_INFO("Speed-adaptive steering: %.1f-%.1f kph | rate %.1f->%.1f deg/s | tau %.3f->%.3f s",
                 steering_schedule_min_speed_kph_,
                 steering_schedule_max_speed_kph_,
                 low_speed_max_steer_rate_rad_ * 180.0 / kPi,
                 high_speed_max_steer_rate_rad_ * 180.0 / kPi,
                 low_speed_steering_time_constant_,
                 high_speed_steering_time_constant_);
    }
    ROS_INFO("Measured longitudinal model: %s, full brake %.3f m/s^2, latency %.3f s",
             use_measured_longitudinal_model_ ? "on" : "off",
             measured_brake_decel_mps2_,
             longitudinal_latency_s_);
    ROS_INFO("FullMpcControl status print rate: %.1f Hz", status_print_hz_);
}

void FullMpcControl::gpsCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    (void)msg;
    gps_state_ = true;
}

void FullMpcControl::egoVehicleRealCallback(const geometry_msgs::TwistWithCovarianceStamped::ConstPtr& msg)
{
    vehicle_info_ = true;
    current_speed_mps_ = std::hypot(msg->twist.twist.linear.x, msg->twist.twist.linear.y);
    current_speed_kph_ = current_speed_mps_ * 3.6;
}

void FullMpcControl::speedMoraiCallback(const std_msgs::Float64::ConstPtr& msg)
{
    vehicle_info_ = true;
    // /current_speed = |ego 속도| [m/s] (competition UDP bridge). scale 은 단위 보정용(기본 1.0).
    current_speed_mps_ = std::max(0.0, msg->data * morai_velocity_to_kph_scale_);
    current_speed_kph_ = current_speed_mps_ * 3.6;
}

void FullMpcControl::steerFeedbackMoraiCallback(const std_msgs::Float64::ConstPtr& msg)
{
    // /vehicle/front_steer_angle = 앞바퀴 조향각 [rad] (bridge). unit=="deg" 면 도→라디안 변환.
    double steer_feedback = morai_steer_feedback_sign_ * msg->data;
    if (morai_steer_feedback_unit_ == "deg" ||
        morai_steer_feedback_unit_ == "degree" ||
        morai_steer_feedback_unit_ == "degrees") {
        steer_feedback *= kPi / 180.0;
    }
    estimated_steer_ = clampValue(steer_feedback, -max_steer_rad_, max_steer_rad_);
}

void FullMpcControl::localPathCallback(const nav_msgs::Path::ConstPtr& msg)
{
    path_state_ = true;
    local_path_ = *msg;
}

void FullMpcControl::mapStateCallback(const std_msgs::String::ConstPtr& msg)
{
    map_state_ = msg->data;
}

void FullMpcControl::carStateCallback(const std_msgs::String::ConstPtr& msg)
{
    car_state_ = msg->data;
}

void FullMpcControl::lapTunerDebugCallback(const std_msgs::String::ConstPtr& msg)
{
    lap_tuner_debug_text_ = msg->data;
}

PathData FullMpcControl::buildPathData(const nav_msgs::Path& path) const
{
    PathData path_data;
    path_data.points.reserve(path.poses.size());

    // 판단이 pose.position.z 에 실어준 목표속도[kph]의 최댓값. >0 이면 외부 프로파일로 간주.
    double profile_max_z = 0.0;

    for (std::size_t i = 0; i < path.poses.size(); ++i) {
        const geometry_msgs::Point& pos = path.poses[i].pose.position;
        if (!path_data.points.empty()) {
            const PathPoint& last = path_data.points.back();
            const double ds = std::hypot(pos.x - last.x, pos.y - last.y);
            if (ds < path_resolution_min_) {
                continue;
            }
        }

        PathPoint point;
        point.x = pos.x;
        point.y = pos.y;
        point.yaw = 0.0;
        point.s = path_data.points.empty()
                      ? 0.0
                      : path_data.points.back().s + std::hypot(pos.x - path_data.points.back().x,
                                                               pos.y - path_data.points.back().y);
        point.curvature = 0.0;
        // 판단 프로파일: z=목표속도[kph]. z<=0(raw path)면 아래 fallback 이 곡률·제동으로 채움.
        point.speed_limit_mps = pos.z / 3.6;
        profile_max_z = std::max(profile_max_z, pos.z);
        path_data.points.push_back(point);
    }

    if (path_data.points.size() < 2) {
        path_data.total_length = 0.0;
        return path_data;
    }

    for (std::size_t i = 0; i + 1 < path_data.points.size(); ++i) {
        const double dx = path_data.points[i + 1].x - path_data.points[i].x;
        const double dy = path_data.points[i + 1].y - path_data.points[i].y;
        path_data.points[i].yaw = std::atan2(dy, dx);
    }
    path_data.points.back().yaw = path_data.points[path_data.points.size() - 2].yaw;

    std::vector<double> raw_curvature(path_data.points.size(), 0.0);
    for (std::size_t i = 1; i + 1 < path_data.points.size(); ++i) {
        const double dyaw = normalizeAngle(path_data.points[i + 1].yaw - path_data.points[i - 1].yaw);
        const double ds = path_data.points[i + 1].s - path_data.points[i - 1].s;
        if (ds > 1e-6) {
            raw_curvature[i] = dyaw / ds;
        }
    }
    if (raw_curvature.size() > 2) {
        raw_curvature.front() = raw_curvature[1];
        raw_curvature.back() = raw_curvature[raw_curvature.size() - 2];
    }

    std::size_t window_begin = 0;
    std::size_t window_end = 0;
    for (std::size_t i = 0; i < path_data.points.size(); ++i) {
        double weighted_sum = 0.0;
        double weight_sum = 0.0;

        const double s_i = path_data.points[i].s;
        while (window_begin < path_data.points.size() &&
               s_i - path_data.points[window_begin].s > curvature_smoothing_window_m_) {
            ++window_begin;
        }
        window_end = std::max(window_end, window_begin);
        while (window_end < path_data.points.size() &&
               path_data.points[window_end].s - s_i <= curvature_smoothing_window_m_) {
            ++window_end;
        }

        for (std::size_t j = window_begin; j < window_end; ++j) {
            const double ds_abs = std::abs(path_data.points[j].s - s_i);
            // Triangular arc-length window. This is much less noisy than a 3-point
            // derivative when the selected_path resolution is very fine.
            const double weight = 1.0 - ds_abs / std::max(1e-3, curvature_smoothing_window_m_);
            weighted_sum += weight * raw_curvature[j];
            weight_sum += weight;
        }

        path_data.points[i].curvature =
            (weight_sum > 1e-6) ? weighted_sum / weight_sum : raw_curvature[i];
    }

    path_data.total_length = path_data.points.back().s;
    // 외부 프로파일(z>0)이 있으면 그대로 추종. 없으면(raw path) 기존 곡률·제동 프로파일 자체 계산.
    path_data.external_profile = (profile_max_z > 1e-3);
    if (!path_data.external_profile) {
        applyBrakingSpeedProfile(&path_data);
    }
    return path_data;
}

std::size_t FullMpcControl::findClosestPathIndex(const PathData& path, const double x, const double y) const
{
    std::size_t best_index = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < path.points.size(); ++i) {
        const double dx = path.points[i].x - x;
        const double dy = path.points[i].y - y;
        const double distance = dx * dx + dy * dy;
        if (distance < best_distance) {
            best_distance = distance;
            best_index = i;
        }
    }
    return best_index;
}

ReferencePoint FullMpcControl::samplePathAtS(const PathData& path, const double s) const
{
    ReferencePoint ref;
    ref.x = 0.0;
    ref.y = 0.0;
    ref.yaw = 0.0;
    ref.s = 0.0;
    ref.curvature = 0.0;
    ref.speed_limit_mps = max_speed_kph_ / 3.6;

    if (path.points.empty()) {
        return ref;
    }
    if (s <= 0.0) {
        ref.x = path.points.front().x;
        ref.y = path.points.front().y;
        ref.yaw = path.points.front().yaw;
        ref.s = path.points.front().s;
        ref.curvature = path.points.front().curvature;
        ref.speed_limit_mps = path.points.front().speed_limit_mps;
        return ref;
    }
    if (s >= path.total_length) {
        ref.x = path.points.back().x;
        ref.y = path.points.back().y;
        ref.yaw = path.points.back().yaw;
        ref.s = path.points.back().s;
        ref.curvature = path.points.back().curvature;
        ref.speed_limit_mps = path.points.back().speed_limit_mps;
        return ref;
    }

    std::size_t upper_index = 1;
    while (upper_index < path.points.size() && path.points[upper_index].s < s) {
        ++upper_index;
    }

    const PathPoint& lower = path.points[upper_index - 1];
    const PathPoint& upper = path.points[upper_index];
    const double ds = upper.s - lower.s;
    const double ratio = (ds > 1e-6) ? (s - lower.s) / ds : 0.0;

    ref.x = lower.x + (upper.x - lower.x) * ratio;
    ref.y = lower.y + (upper.y - lower.y) * ratio;
    ref.yaw = normalizeAngle(lower.yaw + normalizeAngle(upper.yaw - lower.yaw) * ratio);
    ref.s = s;
    ref.curvature = lower.curvature + (upper.curvature - lower.curvature) * ratio;
    ref.speed_limit_mps =
        lower.speed_limit_mps + (upper.speed_limit_mps - lower.speed_limit_mps) * ratio;
    return ref;
}

double FullMpcControl::fullAccelLimitMps2(const double speed_kph) const
{
    if (!use_measured_longitudinal_model_ || full_accel_by_speed_mps2_.size() != 6) {
        return max_accel_mps2_;
    }

    const int bin = static_cast<int>(clampValue(std::floor(std::max(0.0, speed_kph) / 10.0), 0.0, 5.0));
    return std::min(max_accel_mps2_, std::max(0.1, full_accel_by_speed_mps2_[bin]));
}

bool FullMpcControl::isStopDecision() const
{
    return car_state_ == "stop" || car_state_ == "em";
}

bool FullMpcControl::isSlowDownDecision() const
{
    return car_state_ == "slow_down" ||
           car_state_ == "slowdown" ||
           car_state_ == "change";
}

double FullMpcControl::decisionMaxSpeedMps() const
{
    if (isSlowDownDecision()) {
        return decision_slow_down_speed_kph_ / 3.6;
    }
    return max_speed_kph_ / 3.6;
}

double FullMpcControl::steeringScheduleRatio() const
{
    if (!use_speed_adaptive_steering_) {
        return 0.0;
    }

    const double range = std::max(1e-3, steering_schedule_max_speed_kph_ - steering_schedule_min_speed_kph_);
    return clampValue((current_speed_kph_ - steering_schedule_min_speed_kph_) / range, 0.0, 1.0);
}

double FullMpcControl::scheduledMaxSteerRateRad() const
{
    if (!use_speed_adaptive_steering_) {
        return max_steer_rate_rad_;
    }

    const double ratio = steeringScheduleRatio();
    return low_speed_max_steer_rate_rad_ +
           (high_speed_max_steer_rate_rad_ - low_speed_max_steer_rate_rad_) * ratio;
}

double FullMpcControl::scheduledSteeringTimeConstant() const
{
    if (!use_speed_adaptive_steering_) {
        return steering_time_constant_;
    }

    const double ratio = steeringScheduleRatio();
    return low_speed_steering_time_constant_ +
           (high_speed_steering_time_constant_ - low_speed_steering_time_constant_) * ratio;
}

double FullMpcControl::steerModelGain(const double curvature) const
{
    if (!use_steer_model_gain_) {
        return 1.0;
    }

    return steer_model_gain_straight_ + steer_model_gain_per_curvature_ * std::abs(curvature);
}

void FullMpcControl::reloadTunableParameters(const bool force)
{
    if (!force && tunable_param_reload_hz_ <= 0.0) {
        return;
    }

    const ros::Time now = ros::Time::now();
    if (!force &&
        !last_tunable_reload_time_.isZero() &&
        (now - last_tunable_reload_time_).toSec() < 1.0 / tunable_param_reload_hz_) {
        return;
    }
    last_tunable_reload_time_ = now;

    bool changed = false;
    const double eps = 1e-9;

    const auto updateDouble = [&](const std::string& name,
                                  double& target,
                                  const double min_value,
                                  const double max_value) {
        double loaded = target;
        if (!nh_.getParam(name, loaded)) {
            return;
        }
        loaded = clampValue(loaded, min_value, max_value);
        if (std::abs(loaded - target) > eps) {
            target = loaded;
            changed = true;
        }
    };

    const auto updateDegrees = [&](const std::string& name,
                                   double& target_rad,
                                   const double min_deg,
                                   const double max_deg) {
        double loaded_deg = target_rad * 180.0 / kPi;
        if (!nh_.getParam(name, loaded_deg)) {
            return;
        }
        loaded_deg = clampValue(loaded_deg, min_deg, max_deg);
        const double loaded_rad = loaded_deg * kPi / 180.0;
        if (std::abs(loaded_rad - target_rad) > eps) {
            target_rad = loaded_rad;
            changed = true;
        }
    };

    bool use_steer_model_gain = use_steer_model_gain_;
    if (nh_.getParam("use_steer_model_gain", use_steer_model_gain) &&
        use_steer_model_gain != use_steer_model_gain_) {
        use_steer_model_gain_ = use_steer_model_gain;
        changed = true;
    }

    updateDouble("steer_model_gain_straight", steer_model_gain_straight_, 0.1, 1000.0);
    updateDouble("steer_model_gain_per_curvature", steer_model_gain_per_curvature_, 0.0, 1000.0);

    updateDouble("path_resolution_min", path_resolution_min_, 0.05, 3.0);
    updateDouble("curvature_smoothing_window_m", curvature_smoothing_window_m_, 0.05, 10.0);

    updateDouble("preview_time_s", preview_time_s_, 0.1, 10.0);
    updateDouble("preview_min_distance_m", preview_min_distance_m_, 0.0, 200.0);
    updateDouble("preview_max_distance_m", preview_max_distance_m_, preview_min_distance_m_, 300.0);

    updateDegrees("low_speed_max_steer_rate_degps", low_speed_max_steer_rate_rad_, 1.0, 360.0);
    updateDegrees("high_speed_max_steer_rate_degps", high_speed_max_steer_rate_rad_, 1.0, 360.0);
    updateDouble("low_speed_steering_time_constant", low_speed_steering_time_constant_, 0.03, 5.0);
    updateDouble("high_speed_steering_time_constant", high_speed_steering_time_constant_, 0.03, 5.0);

    updateDouble("lateral_accel_limit_mps2", lateral_accel_limit_mps2_, 0.1, 30.0);
    updateDouble("curve_speed_safety_factor", curve_speed_safety_factor_, 0.30, 1.00);
    updateDouble("minimum_curve_speed_kph", minimum_curve_speed_kph_, 0.0, max_speed_kph_);
    updateDouble("decision_slow_down_speed_kph", decision_slow_down_speed_kph_, minimum_curve_speed_kph_, max_speed_kph_);
    updateDouble("speed_profile_tolerance_kph", speed_profile_tolerance_kph_, 0.0, 100.0);

    updateDouble("weight_cte", weight_cte_, 0.0, 1.0e7);
    updateDouble("weight_heading", weight_heading_, 0.0, 1.0e7);
    updateDouble("weight_speed", weight_speed_, 0.0, 1.0e7);
    updateDouble("weight_steer", weight_steer_, 0.0, 1.0e7);
    updateDouble("weight_steer_rate", weight_steer_rate_, 0.0, 1.0e7);
    updateDouble("weight_steer_accel", weight_steer_accel_, 0.0, 1.0e7);
    updateDouble("weight_accel", weight_accel_, 0.0, 1.0e7);
    updateDouble("weight_jerk", weight_jerk_, 0.0, 1.0e7);
    updateDouble("weight_cte_speed", weight_cte_speed_, 0.0, 1.0e7);
    updateDouble("terminal_weight", terminal_weight_, 0.0, 1.0e4);
    updateDouble("weight_speed_profile_slack", weight_speed_profile_slack_, 1.0, 1.0e9);

    updateDouble("measurement_noise", measurement_noise_, 1.0e-6, 100.0);
    updateDouble("process_noise", process_noise_, 1.0e-9, 100.0);

    preview_max_distance_m_ = std::max(preview_min_distance_m_, preview_max_distance_m_);
    decision_slow_down_speed_kph_ =
        clampValue(decision_slow_down_speed_kph_, minimum_curve_speed_kph_, max_speed_kph_);

    if (changed) {
        ROS_INFO("Reloaded MPC tunables: gain_curv=%.4f, w_cte=%.3f, w_heading=%.3f, "
                 "w_steer_rate=%.3f, w_steer_accel=%.3f, preview=[%.2f, %.2f]m, "
                 "smooth=%.2fm, steer_rate=%.1f->%.1fdeg/s",
                 steer_model_gain_per_curvature_,
                 weight_cte_,
                 weight_heading_,
                 weight_steer_rate_,
                 weight_steer_accel_,
                 preview_min_distance_m_,
                 preview_max_distance_m_,
                 curvature_smoothing_window_m_,
                 low_speed_max_steer_rate_rad_ * 180.0 / kPi,
                 high_speed_max_steer_rate_rad_ * 180.0 / kPi);
    }
}

void FullMpcControl::applyBrakingSpeedProfile(PathData* path) const
{
    if (path == NULL || path->points.empty()) {
        return;
    }

    const double max_speed_mps = decisionMaxSpeedMps();
    const double effective_lateral_accel =
        std::max(0.1, curve_speed_safety_factor_ * lateral_accel_limit_mps2_);

    // First make the local speed limit imposed by curvature at every path point.
    for (std::size_t i = 0; i < path->points.size(); ++i) {
        const double curvature = std::abs(path->points[i].curvature);
        double curve_speed_mps = max_speed_mps;
        if (curvature > 1e-5) {
            curve_speed_mps = std::sqrt(effective_lateral_accel / curvature);
        }
        path->points[i].speed_limit_mps =
            clampValue(curve_speed_mps, minimum_curve_speed_kph_ / 3.6, max_speed_mps);
    }

    // Backward pass: the limit at each earlier point is the fastest speed from which
    // the measured full-brake deceleration can still reach the next point's limit.
    if (path->points.size() < 2) {
        return;
    }
    const double brake_decel = use_measured_longitudinal_model_
                                   ? std::min(max_decel_mps2_, measured_brake_decel_mps2_)
                                   : max_decel_mps2_;
    for (std::size_t i = path->points.size() - 1; i-- > 0;) {
        const double ds = std::max(0.0, path->points[i + 1].s - path->points[i].s);
        const double next_limit = path->points[i + 1].speed_limit_mps;
        const double reachable_speed =
            std::sqrt(std::max(0.0, next_limit * next_limit + 2.0 * brake_decel * ds));
        path->points[i].speed_limit_mps =
            std::min(path->points[i].speed_limit_mps, reachable_speed);
    }
}

double FullMpcControl::computePreviewCurvature(const PathData& path,
                                               const double start_s,
                                               const double preview_distance) const
{
    double signed_peak = 0.0;
    const double end_s = std::min(path.total_length, start_s + preview_distance);
    for (double sample_s = start_s; sample_s <= end_s; sample_s += 0.5) {
        const ReferencePoint ref = samplePathAtS(path, sample_s);
        if (std::abs(ref.curvature) > std::abs(signed_peak)) {
            signed_peak = ref.curvature;
        }
    }
    return signed_peak;
}

double FullMpcControl::applyCurvatureFilter(const double measured_curvature)
{
    estimate_uncertainty_ += process_noise_;
    const double gain = estimate_uncertainty_ / (estimate_uncertainty_ + measurement_noise_);
    curvature_estimate_ += gain * (measured_curvature - curvature_estimate_);
    estimate_uncertainty_ *= (1.0 - gain);
    return curvature_estimate_;
}

MpcResult FullMpcControl::solveMpc(const PathData& path, const double preview_curvature)
{
    MpcResult result;
    if (path.points.size() < 3 || path.total_length < 1.0) {
        return result;
    }

    const std::size_t start_index = findClosestPathIndex(path, tracking_point_offset_, 0.0);
    const double start_s = path.points[start_index].s;

    // 속도 목표 clamp 범위: 판단 프로파일이 있으면 순수 추종(하한 0=정지 반영, slow_down 캡 미적용,
    // 상한은 안전상 60만). 없으면(fallback) 기존 곡률/decision 로직.
    const double v_lower = path.external_profile ? 0.0 : (minimum_curve_speed_kph_ / 3.6);
    const double v_upper = path.external_profile ? (max_speed_kph_ / 3.6) : decisionMaxSpeedMps();

    if (static_cast<int>(warm_steer_.size()) != horizon_steps_ ||
        static_cast<int>(warm_accel_.size()) != horizon_steps_) {
        resetWarmStart();
    }

    const ReferencePoint initial_ref = samplePathAtS(path, start_s);
    double path_age_s = 0.0;
    if (local_path_.header.stamp != ros::Time(0)) {
        path_age_s = std::max(0.0, (ros::Time::now() - local_path_.header.stamp).toSec());
    }
    // Do not let a stale timestamp create an unbounded spatial jump. A truly stale
    // path is handled by the upstream local path watchdog.
    path_age_s = std::min(path_age_s, 0.20);
    const double longitudinal_distance_compensation =
        current_speed_mps_ * (path_age_s + longitudinal_latency_s_);
    const ReferencePoint initial_speed_ref = samplePathAtS(
        path,
        std::min(path.total_length, start_s + longitudinal_distance_compensation));
    const double dx0 = tracking_point_offset_ - initial_ref.x;
    const double dy0 = 0.0 - initial_ref.y;
    const double initial_path_frame_cte =
        (-std::sin(initial_ref.yaw) * dx0) + (std::cos(initial_ref.yaw) * dy0);
    const double initial_cte = -initial_path_frame_cte;
    const double initial_heading_error = normalizeAngle(initial_ref.yaw);

    const int steps = horizon_steps_;
    const int steer_offset = 0;
    const int accel_offset = steps;
    const int speed_slack_offset = 2 * steps;
    const int vars = 2 * steps + (enforce_speed_profile_ ? 1 : 0);

    std::vector<std::vector<double> > hessian(vars, std::vector<double>(vars, 0.0));
    std::vector<double> gradient(vars, 0.0);

    for (int i = 0; i < vars; ++i) {
        hessian[i][i] += 1e-6;
    }
    if (enforce_speed_profile_) {
        hessian[speed_slack_offset][speed_slack_offset] +=
            2.0 * weight_speed_profile_slack_;
    }

    std::vector<double> cte_coeff(vars, 0.0);
    std::vector<double> heading_coeff(vars, 0.0);
    std::vector<double> speed_coeff(vars, 0.0);
    std::vector<double> steer_state_coeff(vars, 0.0);
    std::vector<std::vector<double> > predicted_speed_coeffs;
    std::vector<double> predicted_speed_offsets;
    std::vector<double> profile_speed_limits;
    predicted_speed_coeffs.reserve(steps);
    predicted_speed_offsets.reserve(steps);
    profile_speed_limits.reserve(steps);

    double cte_offset = initial_cte;
    double heading_offset = initial_heading_error;
    double speed_offset = current_speed_mps_;
    double steer_state_offset = estimated_steer_;

    const double steer_alpha =
        std::exp(-mpc_dt_ / std::max(0.03, scheduledSteeringTimeConstant()));
    const double nominal_speed =
        std::max(0.5, current_speed_mps_);

    for (int step = 0; step < steps; ++step) {
        const double temporal_preview_distance = (step + 1) * nominal_speed * mpc_dt_;
        // Lateral MPC deliberately uses a shorter preview so it does not cut a
        // corner. Longitudinal MPC keeps the full temporal horizon below.
        const double lateral_preview_distance =
            std::min(temporal_preview_distance, current_preview_distance_m_);
        const double ref_s = std::min(path.total_length,
                                      start_s + lateral_preview_distance);
        const ReferencePoint ref = samplePathAtS(path, ref_s);
        const ReferencePoint speed_ref = samplePathAtS(
            path,
            std::min(path.total_length,
                     start_s + temporal_preview_distance + longitudinal_distance_compensation));
        const double steer_model_gain = steerModelGain(ref.curvature);
        const double yaw_steer_gain = steer_model_gain / std::max(0.1, wheel_base_);
        const double ff = clampValue(
            std::atan(wheel_base_ * ref.curvature / std::max(0.1, steer_model_gain)),
            -max_steer_rad_,
            max_steer_rad_);
        const double desired_speed_mps = clampValue(
            speed_ref.speed_limit_mps, v_lower, v_upper);

        const std::vector<double> cte_prev_coeff = cte_coeff;
        const std::vector<double> heading_prev_coeff = heading_coeff;
        const std::vector<double> speed_prev_coeff = speed_coeff;

        const double cte_prev_offset = cte_offset;
        const double heading_prev_offset = heading_offset;
        const double speed_prev_offset = speed_offset;
        const double v_model_upper =
            std::max(max_speed_kph_ / 3.6, std::max(current_speed_mps_, speed_prev_offset));
        const double v_model = clampValue(speed_prev_offset, 0.5, v_model_upper);

        cte_offset = cte_prev_offset + v_model * heading_prev_offset * mpc_dt_;
        for (int j = 0; j < vars; ++j) {
            cte_coeff[j] = cte_prev_coeff[j] + v_model * heading_prev_coeff[j] * mpc_dt_;
        }

        steer_state_offset *= steer_alpha;
        for (int j = 0; j < vars; ++j) {
            steer_state_coeff[j] *= steer_alpha;
        }
        steer_state_coeff[steer_offset + step] += (1.0 - steer_alpha);

        heading_offset = heading_prev_offset + v_model * ref.curvature * mpc_dt_
                         - v_model * yaw_steer_gain * steer_state_offset * mpc_dt_;
        for (int j = 0; j < vars; ++j) {
            heading_coeff[j] = heading_prev_coeff[j]
                               - v_model * yaw_steer_gain * steer_state_coeff[j] * mpc_dt_;
        }

        speed_offset = speed_prev_offset;
        speed_coeff = speed_prev_coeff;
        speed_coeff[accel_offset + step] += mpc_dt_;
        predicted_speed_coeffs.push_back(speed_coeff);
        predicted_speed_offsets.push_back(speed_offset);
        profile_speed_limits.push_back(desired_speed_mps);

        const double cte_weight = weight_cte_ + weight_cte_speed_ * v_model * v_model;
        addQuadraticTerm(hessian, gradient, cte_coeff, cte_offset, cte_weight);
        addQuadraticTerm(hessian, gradient, heading_coeff, heading_offset, weight_heading_);
        addQuadraticTerm(hessian, gradient, speed_coeff, speed_offset - desired_speed_mps, weight_speed_);

        std::vector<double> steer_track_coeff(vars, 0.0);
        steer_track_coeff[steer_offset + step] = 1.0;
        addQuadraticTerm(hessian, gradient, steer_track_coeff, -ff, weight_steer_);

        std::vector<double> accel_cost_coeff(vars, 0.0);
        accel_cost_coeff[accel_offset + step] = 1.0;
        addQuadraticTerm(hessian, gradient, accel_cost_coeff, 0.0, weight_accel_);

        if (step == steps - 1) {
            addQuadraticTerm(hessian, gradient, cte_coeff, cte_offset, terminal_weight_ * weight_cte_);
            addQuadraticTerm(hessian, gradient, heading_coeff, heading_offset, terminal_weight_ * weight_heading_);
        }
    }

    for (int step = 0; step < steps; ++step) {
        std::vector<double> steer_rate_coeff(vars, 0.0);
        double steer_rate_offset = 0.0;
        steer_rate_coeff[steer_offset + step] = 1.0;
        if (step == 0) {
            steer_rate_offset = -last_steer_cmd_;
        } else {
            steer_rate_coeff[steer_offset + step - 1] = -1.0;
        }
        addQuadraticTerm(hessian, gradient, steer_rate_coeff, steer_rate_offset, weight_steer_rate_);

        std::vector<double> steer_accel_coeff(vars, 0.0);
        double steer_accel_offset = 0.0;
        steer_accel_coeff[steer_offset + step] = 1.0;
        if (step == 0) {
            steer_accel_offset = -last_steer_cmd_;
        } else if (step == 1) {
            steer_accel_coeff[steer_offset + step - 1] = -2.0;
            steer_accel_offset = last_steer_cmd_;
        } else {
            steer_accel_coeff[steer_offset + step - 1] = -2.0;
            steer_accel_coeff[steer_offset + step - 2] = 1.0;
        }
        addQuadraticTerm(hessian, gradient, steer_accel_coeff, steer_accel_offset, weight_steer_accel_);

        std::vector<double> jerk_coeff(vars, 0.0);
        double jerk_offset = 0.0;
        jerk_coeff[accel_offset + step] = 1.0 / std::max(1e-3, mpc_dt_);
        if (step == 0) {
            jerk_offset = -last_accel_cmd_mps2_ / std::max(1e-3, mpc_dt_);
        } else {
            jerk_coeff[accel_offset + step - 1] = -1.0 / std::max(1e-3, mpc_dt_);
        }
        addQuadraticTerm(hessian, gradient, jerk_coeff, jerk_offset, weight_jerk_);
    }

    std::vector<std::vector<double> > constraint_matrix;
    std::vector<double> lower_bounds;
    std::vector<double> upper_bounds;

    for (int step = 0; step < steps; ++step) {
        std::vector<double> row(vars, 0.0);
        row[steer_offset + step] = 1.0;
        constraint_matrix.push_back(row);
        lower_bounds.push_back(-max_steer_rad_);
        upper_bounds.push_back(max_steer_rad_);
    }
    double accel_constraint_speed_kph = current_speed_kph_;
    for (int step = 0; step < steps; ++step) {
        std::vector<double> row(vars, 0.0);
        row[accel_offset + step] = 1.0;
        constraint_matrix.push_back(row);
        const double step_decel_limit = use_measured_longitudinal_model_
                                            ? std::min(max_decel_mps2_, measured_brake_decel_mps2_)
                                            : max_decel_mps2_;
        lower_bounds.push_back(-step_decel_limit);
        const double step_accel_limit = fullAccelLimitMps2(accel_constraint_speed_kph);
        upper_bounds.push_back(step_accel_limit);
        accel_constraint_speed_kph = std::min(
            max_speed_kph_,
            accel_constraint_speed_kph + step_accel_limit * mpc_dt_ * 3.6);
    }

    const double steer_rate_limit = scheduledMaxSteerRateRad() * mpc_dt_;
    for (int step = 0; step < steps; ++step) {
        std::vector<double> row(vars, 0.0);
        row[steer_offset + step] = 1.0;
        if (step == 0) {
            lower_bounds.push_back(last_steer_cmd_ - steer_rate_limit);
            upper_bounds.push_back(last_steer_cmd_ + steer_rate_limit);
        } else {
            row[steer_offset + step - 1] = -1.0;
            lower_bounds.push_back(-steer_rate_limit);
            upper_bounds.push_back(steer_rate_limit);
        }
        constraint_matrix.push_back(row);
    }

    const double accel_step_limit = max_jerk_mps3_ * mpc_dt_;
    for (int step = 0; step < steps; ++step) {
        std::vector<double> row(vars, 0.0);
        row[accel_offset + step] = 1.0;
        if (step == 0) {
            lower_bounds.push_back(last_accel_cmd_mps2_ - accel_step_limit);
            upper_bounds.push_back(last_accel_cmd_mps2_ + accel_step_limit);
        } else {
            row[accel_offset + step - 1] = -1.0;
            lower_bounds.push_back(-accel_step_limit);
            upper_bounds.push_back(accel_step_limit);
        }
        constraint_matrix.push_back(row);
    }

    if (enforce_speed_profile_) {
        const double physical_decel = use_measured_longitudinal_model_
                                          ? std::min(max_decel_mps2_, measured_brake_decel_mps2_)
                                          : max_decel_mps2_;
        const double tolerance_mps = speed_profile_tolerance_kph_ / 3.6;
        for (int step = 0; step < steps; ++step) {
            // If the path arrives while the vehicle is already above its envelope,
            // relax only to the lowest speed physically reachable with full braking.
            // This keeps the QP feasible while forcing the deceleration bound.
            const double minimum_reachable_speed = std::max(
                0.0,
                current_speed_mps_ - physical_decel * (step + 1) * mpc_dt_);
            const double hard_speed_limit =
                std::max(profile_speed_limits[step], minimum_reachable_speed) + tolerance_mps;

            std::vector<double> speed_row = predicted_speed_coeffs[step];
            speed_row[speed_slack_offset] = -1.0;
            constraint_matrix.push_back(speed_row);
            lower_bounds.push_back(-OSQP_INFTY);
            upper_bounds.push_back(hard_speed_limit - predicted_speed_offsets[step]);
        }

        std::vector<double> slack_bound_row(vars, 0.0);
        slack_bound_row[speed_slack_offset] = 1.0;
        constraint_matrix.push_back(slack_bound_row);
        lower_bounds.push_back(0.0);
        upper_bounds.push_back(OSQP_INFTY);
    }

    SparseCscData p_csc = denseUpperToCsc(hessian);
    SparseCscData a_csc = denseToCsc(constraint_matrix);
    if (p_csc.values.empty() || a_csc.col_pointers.empty()) {
        return result;
    }

    std::vector<c_float> q(vars, 0.0);
    for (int i = 0; i < vars; ++i) {
        q[i] = static_cast<c_float>(gradient[i]);
    }

    std::vector<c_float> lower(lower_bounds.size(), 0.0);
    std::vector<c_float> upper(upper_bounds.size(), 0.0);
    for (std::size_t i = 0; i < lower_bounds.size(); ++i) {
        lower[i] = static_cast<c_float>(lower_bounds[i]);
        upper[i] = static_cast<c_float>(upper_bounds[i]);
    }

    csc* P = csc_matrix(static_cast<c_int>(vars),
                        static_cast<c_int>(vars),
                        static_cast<c_int>(p_csc.values.size()),
                        p_csc.values.data(),
                        p_csc.row_indices.data(),
                        p_csc.col_pointers.data());
    csc* A = csc_matrix(static_cast<c_int>(constraint_matrix.size()),
                        static_cast<c_int>(vars),
                        static_cast<c_int>(a_csc.values.size()),
                        a_csc.values.data(),
                        a_csc.row_indices.data(),
                        a_csc.col_pointers.data());

    OSQPData data;
    data.n = static_cast<c_int>(vars);
    data.m = static_cast<c_int>(constraint_matrix.size());
    data.P = P;
    data.A = A;
    data.q = q.data();
    data.l = lower.data();
    data.u = upper.data();

    OSQPSettings settings;
    osqp_set_default_settings(&settings);
    settings.verbose = false;
    settings.polish = true;
    settings.max_iter = 1000;
    settings.eps_abs = 1e-3;
    settings.eps_rel = 1e-3;
    settings.warm_start = true;

    OSQPWorkspace* work = NULL;
    const c_int setup_status = osqp_setup(&work, &data, &settings);
    if (setup_status != 0 || work == NULL) {
        if (P != NULL) {
            c_free(P);
        }
        if (A != NULL) {
            c_free(A);
        }
        return result;
    }

    std::vector<c_float> warm(vars, 0.0);
    for (int i = 0; i < steps; ++i) {
        warm[steer_offset + i] = static_cast<c_float>(warm_steer_[i]);
        warm[accel_offset + i] = static_cast<c_float>(warm_accel_[i]);
    }
    osqp_warm_start_x(work, warm.data());

    osqp_solve(work);
    const bool solved =
        work->info != NULL &&
        (work->info->status_val == OSQP_SOLVED ||
         work->info->status_val == OSQP_SOLVED_INACCURATE);
    if (!solved || work->solution == NULL || work->solution->x == NULL) {
        if (work->info != NULL) {
            ROS_WARN_THROTTLE(1.0, "OSQP MPC failed: status=%s (%d)",
                              work->info->status,
                              static_cast<int>(work->info->status_val));
        }
        osqp_cleanup(work);
        c_free(P);
        c_free(A);
        return result;
    }

    std::vector<double> best_steer(steps, 0.0);
    std::vector<double> best_accel(steps, 0.0);
    double solved_speed_slack_kph = 0.0;
    for (int i = 0; i < steps; ++i) {
        best_steer[i] = clampValue(static_cast<double>(work->solution->x[steer_offset + i]),
                                   -max_steer_rad_,
                                   max_steer_rad_);
        best_accel[i] = clampValue(static_cast<double>(work->solution->x[accel_offset + i]),
                                   -max_decel_mps2_,
                                   max_accel_mps2_);
    }
    if (enforce_speed_profile_) {
        solved_speed_slack_kph =
            std::max(0.0, static_cast<double>(work->solution->x[speed_slack_offset])) * 3.6;
    }

    osqp_cleanup(work);
    c_free(P);
    c_free(A);

    result.valid = true;
    result.steer_sequence = best_steer;
    result.accel_sequence = best_accel;
    result.steer_cmd = best_steer.front();
    result.accel_cmd_mps2 = best_accel.front();
    result.initial_cte = initial_cte;
    result.initial_heading_error = initial_heading_error;
    result.preview_curvature = preview_curvature;
    result.profile_speed_kph =
        clampValue(initial_speed_ref.speed_limit_mps, v_lower, v_upper) * 3.6;
    result.full_accel_limit_mps2 = fullAccelLimitMps2(current_speed_kph_);
    result.speed_profile_slack_kph = solved_speed_slack_kph;
    return result;
}

void FullMpcControl::shiftWarmStart(const MpcResult& result)
{
    if (!result.valid || result.steer_sequence.empty() || result.accel_sequence.empty()) {
        resetWarmStart();
        return;
    }

    warm_steer_.assign(horizon_steps_, result.steer_sequence.back());
    warm_accel_.assign(horizon_steps_, result.accel_sequence.back());
    for (int i = 0; i < horizon_steps_; ++i) {
        const int source = std::min(static_cast<int>(result.steer_sequence.size()) - 1, i + 1);
        warm_steer_[i] = result.steer_sequence[source];
        warm_accel_[i] = result.accel_sequence[source];
    }
}

void FullMpcControl::resetWarmStart()
{
    warm_steer_.assign(horizon_steps_, 0.0);
    warm_accel_.assign(horizon_steps_, 0.0);
}

void FullMpcControl::publishControlCommand(const double accel_mps2,
                                           const double steer_cmd,
                                           MpcResult* result)
{
    const double effective_steer_rate_rad = scheduledMaxSteerRateRad();
    const double limited_steer = clampValue(steer_cmd,
                                           last_steer_cmd_ - effective_steer_rate_rad * dt_control_,
                                           last_steer_cmd_ + effective_steer_rate_rad * dt_control_);
    const double limited_accel = clampValue(accel_mps2,
                                            last_accel_cmd_mps2_ - max_jerk_mps3_ * dt_control_,
                                            last_accel_cmd_mps2_ + max_jerk_mps3_ * dt_control_);

    last_steer_cmd_ = clampValue(limited_steer, -max_steer_rad_, max_steer_rad_);
    last_accel_cmd_mps2_ = clampValue(limited_accel, -max_decel_mps2_, max_accel_mps2_);

    if (last_accel_cmd_mps2_ > 0.03) {
        const double pedal_full_scale = use_measured_longitudinal_model_
                                            ? fullAccelLimitMps2(current_speed_kph_)
                                            : accel_pedal_full_scale_mps2_;
        last_accel_pedal_cmd_ =
            clampValue(last_accel_cmd_mps2_ / std::max(0.1, pedal_full_scale),
                       0.0,
                       max_accel_pedal_cmd_);
        last_brake_pedal_cmd_ = 0.0;
    } else if (last_accel_cmd_mps2_ < -0.03) {
        const double pedal_full_scale = use_measured_longitudinal_model_
                                            ? measured_brake_decel_mps2_
                                            : brake_pedal_full_scale_mps2_;
        last_accel_pedal_cmd_ = 0.0;
        last_brake_pedal_cmd_ =
            clampValue(-last_accel_cmd_mps2_ / std::max(0.1, pedal_full_scale),
                       0.0,
                       max_brake_pedal_cmd_);
    } else {
        last_accel_pedal_cmd_ = 0.0;
        last_brake_pedal_cmd_ = 0.0;
    }

    if (result != NULL) {
        const double steer_limit_display_threshold = 0.3 * kPi / 180.0;
        const double accel_limit_display_threshold = 0.15;
        result->published_steer_cmd = last_steer_cmd_;
        result->published_accel_cmd = last_accel_pedal_cmd_;
        result->published_brake_cmd = last_brake_pedal_cmd_;
        result->published_accel_mps2 = last_accel_cmd_mps2_;
        result->steer_saturated =
            std::abs(limited_steer - steer_cmd) > steer_limit_display_threshold ||
            std::abs(last_steer_cmd_) > max_steer_rad_ - steer_limit_display_threshold;
        result->accel_limited =
            std::abs(limited_accel - accel_mps2) > accel_limit_display_threshold;
    }

    const double steer_tau = std::max(0.03, scheduledSteeringTimeConstant());
    const double steer_rate = clampValue((last_steer_cmd_ - estimated_steer_) / steer_tau,
                                         -effective_steer_rate_rad,
                                         effective_steer_rate_rad);
    estimated_steer_ = clampValue(estimated_steer_ + steer_rate * dt_control_,
                                  -max_steer_rad_,
                                  max_steer_rad_);

    if (use_morai_sim_) {
        ctrl_cmd_msg_.longlCmdType = morai_longitudinal_cmd_type_;
        ctrl_cmd_msg_.velocity = 0.0;
        ctrl_cmd_msg_.steering = last_steer_cmd_;

        if (morai_longitudinal_cmd_type_ == 3) {
            // Acceleration control: this matches the MPC prediction model.
            ctrl_cmd_msg_.accel = 0.0;
            ctrl_cmd_msg_.brake = 0.0;
            ctrl_cmd_msg_.acceleration = last_accel_cmd_mps2_;
        } else if (morai_longitudinal_cmd_type_ == 2) {
            // Velocity control fallback. Keep a short one-step speed target.
            ctrl_cmd_msg_.accel = 0.0;
            ctrl_cmd_msg_.brake = 0.0;
            ctrl_cmd_msg_.velocity =
                std::max(0.0, current_speed_kph_ + last_accel_cmd_mps2_ * dt_control_ * 3.6);
            ctrl_cmd_msg_.acceleration = 0.0;
        } else {
            // Throttle/brake control fallback.
            ctrl_cmd_msg_.accel = last_accel_pedal_cmd_;
            ctrl_cmd_msg_.brake = last_brake_pedal_cmd_;
            ctrl_cmd_msg_.velocity = 0.0;
            ctrl_cmd_msg_.acceleration = 0.0;
        }

        ctrl_cmd_pub_.publish(ctrl_cmd_msg_);
    } else {
        cmd_vel_msg_.linear.x = std::max(0.0, current_speed_mps_ + last_accel_cmd_mps2_ * dt_control_);
        cmd_vel_msg_.angular.z = -last_steer_cmd_;
        cmd_vel_pub_.publish(cmd_vel_msg_);
    }
}

void FullMpcControl::publishStopCommand()
{
    last_accel_cmd_mps2_ = 0.0;
    last_accel_pedal_cmd_ = 0.0;
    last_brake_pedal_cmd_ = 0.0;
    last_steer_cmd_ = 0.0;
    estimated_steer_ = 0.0;
    resetWarmStart();
    if (use_morai_sim_) {
        // In acceleration mode MORAI may ignore brake, so force pedal mode for stops/failsafes.
        ctrl_cmd_msg_.longlCmdType = 1;
        ctrl_cmd_msg_.accel = 0.0;
        ctrl_cmd_msg_.brake = 1.0;
        ctrl_cmd_msg_.velocity = 0.0;
        ctrl_cmd_msg_.acceleration = 0.0;
        ctrl_cmd_msg_.steering = 0.0;
        ctrl_cmd_pub_.publish(ctrl_cmd_msg_);
    } else {
        cmd_vel_msg_ = geometry_msgs::Twist();
        cmd_vel_pub_.publish(cmd_vel_msg_);
    }
}

void FullMpcControl::publishDebugSignals(const double curvature,
                                         const double heading_error,
                                         const double cross_track_error)
{
    std_msgs::Float64 curvature_msg;
    curvature_msg.data = curvature;
    curvature_pub_.publish(curvature_msg);

    std_msgs::Float64 heading_msg;
    heading_msg.data = heading_error;
    hde_pub_.publish(heading_msg);

    std_msgs::Float64 cte_msg;
    cte_msg.data = cross_track_error;
    cte_pub_.publish(cte_msg);

    std_msgs::Float64 target_accel_msg;
    target_accel_msg.data = last_accel_cmd_mps2_;
    target_accel_pub_.publish(target_accel_msg);

    const double lateral_accel_est =
        current_speed_mps_ * current_speed_mps_ * std::tan(estimated_steer_) / std::max(0.1, wheel_base_);
    std_msgs::Float64 penalty_msg;
    penalty_msg.data = std::max(0.0, std::abs(lateral_accel_est) - lateral_accel_limit_mps2_);
    penalty_pub_.publish(penalty_msg);

    cte_square_sum_ += cross_track_error * cross_track_error;
    ++cte_count_;
    std_msgs::Float64 rmse_msg;
    rmse_msg.data = std::sqrt(cte_square_sum_ / static_cast<double>(std::max(1, cte_count_)));
    rmse_pub_.publish(rmse_msg);
}

void FullMpcControl::printStatus(const MpcResult& result) const
{
    const double steer_deg = displayValue(last_steer_cmd_ * 180.0 / kPi);
    const double raw_steer_deg = displayValue(result.steer_cmd * 180.0 / kPi);
    const double cte = displayValue(result.initial_cte);
    const double heading_deg = displayValue(result.initial_heading_error * 180.0 / kPi);
    const double preview_curvature = displayValue(result.preview_curvature, 0.0005);
    const double accel_cmd = displayValue(result.published_accel_mps2);

    std::cout << std::fixed << std::setprecision(2);
    std::cout << "Full MPC steer: " << std::showpos << std::setw(7) << steer_deg
              << std::noshowpos << " deg | raw: " << std::showpos << std::setw(7) << raw_steer_deg
              << std::noshowpos << " deg | accel: " << std::showpos << std::setw(6) << accel_cmd
              << std::noshowpos << " m/s^2 | current: " << std::setw(6) << current_speed_kph_
              << " kph" << std::endl;
    std::cout << "CTE: " << std::showpos << std::setw(7) << cte
              << std::noshowpos << " | heading err: " << std::showpos << std::setw(7) << heading_deg
              << std::noshowpos << " deg | preview curvature: " << std::showpos << std::setw(7)
              << preview_curvature << std::noshowpos
              << " | preview: " << std::setw(5) << current_preview_distance_m_ << " m" << std::endl;
    std::cout << "pedal accel: " << std::setw(5) << result.published_accel_cmd
              << " | brake: " << std::setw(5) << result.published_brake_cmd
              << " | profile speed: " << std::setw(6) << result.profile_speed_kph
              << " kph | full accel limit: " << std::setw(5) << result.full_accel_limit_mps2
              << " m/s^2 | speed slack: " << std::setw(5) << result.speed_profile_slack_kph
              << " kph" << std::endl;
    std::cout << "sat steer: " << (result.steer_saturated ? "yes" : "no")
              << " | accel limited: " << (result.accel_limited ? "yes" : "no")
              << " | map: " << map_state_
              << " | decision: " << car_state_ << std::endl;
    if (show_lap_tuner_debug_ && !lap_tuner_debug_text_.empty()) {
        std::cout << "Lap tuner: " << lap_tuner_debug_text_ << std::endl;
    }
    std::cout << "=================================================================" << std::endl;
}

void FullMpcControl::run()
{
    reloadTunableParameters();

    if (isStopDecision()) {
        publishStopCommand();
        std::cout << "decision stop" << std::endl;
        std::cout << "=================================================================" << std::endl;
        return;
    }

    if (!path_state_ || !vehicle_info_ || local_path_.poses.size() < 3) {
        publishStopCommand();
        if (!path_state_) {
            std::cout << "path_state check" << std::endl;
        }
        if (!vehicle_info_) {
            std::cout << "vehicle_info check" << std::endl;
        }
        std::cout << "=================================================================" << std::endl;
        return;
    }

    const PathData path_data = buildPathData(local_path_);
    if (path_data.points.size() < 3 || path_data.total_length < 1.0) {
        publishStopCommand();
        std::cout << "selected_path is too short for Full MPC" << std::endl;
        std::cout << "=================================================================" << std::endl;
        return;
    }

    const std::size_t closest_index = findClosestPathIndex(path_data, tracking_point_offset_, 0.0);
    const double path_start_s = path_data.points[closest_index].s;
    current_preview_distance_m_ =
        clampValue(current_speed_mps_ * preview_time_s_,
                   preview_min_distance_m_,
                   preview_max_distance_m_);
    const double raw_preview_curvature =
        computePreviewCurvature(path_data, path_start_s, current_preview_distance_m_);
    const double preview_curvature = applyCurvatureFilter(raw_preview_curvature);

    MpcResult result = solveMpc(path_data, preview_curvature);
    if (!result.valid) {
        publishStopCommand();
        std::cout << "Full MPC optimization failed" << std::endl;
        std::cout << "=================================================================" << std::endl;
        return;
    }

    shiftWarmStart(result);

    publishControlCommand(result.accel_cmd_mps2, result.steer_cmd, &result);
    publishDebugSignals(preview_curvature, result.initial_heading_error, result.initial_cte);
    const ros::Time now = ros::Time::now();
    if (status_print_hz_ > 0.0 &&
        (last_status_print_time_.isZero() ||
         (now - last_status_print_time_).toSec() >= 1.0 / status_print_hz_)) {
        printStatus(result);
        last_status_print_time_ = now;
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "FullMpcControl");

    FullMpcControl controller;
    ros::NodeHandle nh("~");
    int control_frequency = 50;
    nh.param("control_frequency", control_frequency, 50);
    ros::Rate rate(std::max(5, control_frequency));

    while (ros::ok()) {
        ros::spinOnce();
        controller.run();
        rate.sleep();
    }

    return 0;
}
