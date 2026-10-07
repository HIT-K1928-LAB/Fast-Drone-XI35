#ifndef YOPO_PLANNER_YOPO_PLANNER_H_
#define YOPO_PLANNER_YOPO_PLANNER_H_

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <array>
#include <mutex>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>
#include <ros/time.h>
#include <vector>

namespace yopo_planner {

struct YopoParams {
    int image_height           = 96;
    int image_width            = 160;
    int horizon_num            = 5;
    int vertical_num           = 3;
    int radio_num              = 1;
    int traj_num               = 15;
    double velocity            = 6.0;
    double vel_max_train       = 6.0;
    double acc_max_train       = 6.0;
    double horizon_camera_fov  = 90.0;
    double vertical_camera_fov = 60.0;
    double horizon_anchor_fov  = 30.0;
    double vertical_anchor_fov = 30.0;
    double radio_range         = 5.0;
    double goal_length         = 10.0;
    // Sensor-valid depth interval is independent from the depth scale used by
    // the pretrained network. Invalid sensor samples are filled before model
    // normalization; valid samples are always divided by model_depth_max.
    double sensor_min_depth          = 0.04;
    double sensor_reliable_max_depth = 20.0;
    double model_depth_max           = 20.0;
    double ctrl_dt             = 0.02;
    double arrive_distance     = 5.0;
    // Optional final straight-line braking segment. Keep disabled on the real
    // vehicle until the final approach has been checked for obstacles.
    bool terminal_approach_enabled = false;
    double terminal_brake_accel = 3.0;
    double terminal_brake_margin = 0.5;
    double terminal_position_tolerance = 0.25;
    double terminal_speed_tolerance = 0.2;
    double terminal_stable_time = 0.3;
    double arrival_yaw_rate    = 0.8;
    double arrival_yaw_min_distance = 0.3;
    // Dynamic-target tracking keeps the camera optical axis on the target
    // independently of the lateral direction selected by a YOPO primitive.
    double dynamic_target_yaw_rate = 1.2;
    double dynamic_target_yaw_accel = 2.5;
    double dynamic_target_yaw_deadband_deg = 1.0;
    // The network is pulled toward a point this far in front of the tracked
    // aircraft. Once inside the radius, publish a smooth stop/hold instead of
    // asking a long YOPO primitive to pass through the target.
    double dynamic_target_standoff_distance = 1.2;
    double dynamic_target_standoff_hysteresis = 0.25;
    // During tracker COASTING, shorten the selected primitive progressively
    // instead of continuing at full speed on an increasingly uncertain goal.
    double dynamic_target_min_speed_scale = 0.35;
    double dynamic_target_coast_slowdown_time = 1.5;
    // After a hard loss the vehicle brakes, then scans around the last target
    // bearing while the detector continues looking for reacquisition.
    bool dynamic_target_search_enabled = true;
    double dynamic_target_search_yaw_amplitude_deg = 25.0;
    double dynamic_target_search_yaw_rate = 0.5;
    // A front-facing depth camera cannot protect a translation toward a goal
    // outside its horizontal field of view. Brake first, then turn in place
    // until the new goal enters the configured forward sector.
    bool goal_yaw_alignment_enabled       = true;
    double goal_yaw_align_enter_deg       = 35.0;
    double goal_yaw_align_rate            = 0.8;
    double goal_yaw_brake_speed_tolerance = 0.15;
    double goal_yaw_brake_stable_time     = 0.3;
    double goal_yaw_align_min_distance    = 0.5;
    double pitch_angle_deg     = 0.0;
    bool plan_from_reference   = false;
    bool require_camera_extrinsic = false;
};

struct Poly5Solver {
    Eigen::Matrix<double, 6, 1> coeff = Eigen::Matrix<double, 6, 1>::Zero();

    void reset(
        double pos0, double vel0, double acc0, double pos1, double vel1, double acc1, double tf);
    double position(double t) const;
    double velocity(double t) const;
    double acceleration(double t) const;
    double jerk(double t) const;
};

class YopoPlanner {
  public:
    explicit YopoPlanner(YopoParams params = YopoParams());

    const YopoParams& params() const { return params_; }
    double trajTime() const { return segment_time_; }
    int trajNum() const { return params_.traj_num; }

    void setGoal(const Eigen::Vector3d& goal);
    bool updateDynamicTarget(
        const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
    void setDynamicTargetTrackingScale(double scale);
    bool handleDynamicTargetLost();
    Eigen::Vector3d goal() const { return goal_; }
    bool odomInitialized() const { return odom_init_; }
    bool arrived() const { return arrive_; }
    bool terminalApproachActive() const;
    bool hasTrajectory() const { return has_trajectory_; }
    double controlTime() const;
    bool dynamicTargetLost() const;
    bool dynamicLossHoldActive() const;
    bool dynamicStandoffHoldActive() const;
    bool goalAlignmentActive() const;
    bool cameraExtrinsicReady() const { return camera_extrinsic_ready_; }
    bool requiresCameraExtrinsic() const { return params_.require_camera_extrinsic; }

    void setCameraToBodyExtrinsic(const Eigen::Matrix3d& rotation_bc);
    void updateOdometry(const nav_msgs::Odometry& odom);
    bool syncReferenceFromCurrentOdom();
    std::array<float, 1 * 9 * 3 * 5> prepareObsInput();
    void updateTrajectory(
        const std::array<float, 1 * 9 * 3 * 5>& endstate_pred,
        const std::array<float, 1 * 3 * 5>& score_pred, bool return_all_preds);
    bool fillControlCommand(quadrotor_msgs::PositionCommand* cmd);
    quadrotor_msgs::PositionCommand emptyControlCommand() const;

    std::vector<Eigen::Vector3f> bestTrajectoryPoints(int samples) const;
    std::vector<Eigen::Vector3f> latticeTrajectoryPoints(int samples) const;
    std::vector<Eigen::Vector4f> allTrajectoryPoints(int samples) const;

  private:
    enum class GoalAlignmentState { INACTIVE, BRAKING, TURNING };

    struct EndState {
        Eigen::Vector3d pos = Eigen::Vector3d::Zero();
        Eigen::Vector3d vel = Eigen::Vector3d::Zero();
        Eigen::Vector3d acc = Eigen::Vector3d::Zero();
    };

    void buildLattice();
    void syncReferenceFromOdomLocked();
    void initializeArrivalHoldLocked(const Eigen::Vector3d& position, double yaw);
    void startTerminalApproachLocked(const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
    void startDynamicStandoffHoldLocked(
        const Eigen::Vector3d& position, const Eigen::Vector3d& velocity);
    void startGoalAlignmentIfNeededLocked();
    bool fillGoalAlignmentCommandLocked(quadrotor_msgs::PositionCommand* cmd);
    double currentYawLocked() const;
    EndState predToEndstate(const std::array<float, 1 * 9 * 3 * 5>& pred, int action_id) const;
    EndState predToEndstateByLattice(const Eigen::Matrix<double, 9, 1>& pred, int lattice_id) const;
    Eigen::Vector3d currentStartPos() const;
    Eigen::Vector3d currentStartVel() const;
    static double wrapToPi(double angle);
    static std::pair<double, double> calculateYaw(
        const Eigen::Vector3d& vel_dir, const Eigen::Vector3d& goal_dir, double last_yaw,
        double dt);
    static std::pair<double, double> calculateTargetYaw(
        double target_yaw, double last_yaw, double dt, double max_yaw_rate);
    static std::pair<double, double> calculateTargetYawLimited(
        double target_yaw, double last_yaw, double last_yaw_rate, double dt,
        double max_yaw_rate, double max_yaw_accel, double deadband);
    static Eigen::Matrix<double, 6, 1> polyCoefficients(
        double pos0, double vel0, double acc0, double pos1, double vel1, double acc1, double tf);

    YopoParams params_;
    double vel_max_              = 6.0;
    double acc_max_              = 6.0;
    double segment_time_         = 10.0 / 6.0;
    double yaw_diff_             = 0.0;
    double pitch_diff_           = 0.0;
    Eigen::Matrix3d rotation_bc_ = Eigen::Matrix3d::Identity();
    Eigen::Matrix3d rotation_wc_ = Eigen::Matrix3d::Identity();
    bool camera_extrinsic_ready_ = false;

    std::vector<Eigen::Vector3d> lattice_pos_;
    std::vector<Eigen::Vector2d> lattice_angle_;
    std::vector<Eigen::Matrix3d> lattice_rbp_;

    nav_msgs::Odometry odom_;
    bool odom_init_                 = false;
    bool desire_init_               = false;
    bool arrive_                    = false;
    bool terminal_approach_active_  = false;
    double terminal_time_           = 0.0;
    double terminal_stable_elapsed_ = 0.0;
    double terminal_yaw_            = 0.0;
    ros::Time terminal_last_odom_stamp_;
    bool arrival_hover_initialized_ = false;
    bool has_trajectory_            = false;
    bool dynamic_target_active_     = false;
    bool dynamic_target_lost_       = false;
    bool dynamic_loss_hold_active_  = false;
    bool dynamic_standoff_hold_active_ = false;
    double dynamic_standoff_release_distance_ = 0.0;
    double dynamic_target_tracking_scale_ = 1.0;
    double dynamic_target_yaw_rate_cmd_ = 0.0;
    double dynamic_loss_search_center_yaw_ = 0.0;
    int dynamic_loss_search_direction_ = 1;
    bool goal_alignment_pending_    = false;
    GoalAlignmentState goal_alignment_state_ = GoalAlignmentState::INACTIVE;
    Eigen::Vector3d goal_              = Eigen::Vector3d(50.0, 0.0, 2.0);
    Eigen::Vector3d goal_velocity_     = Eigen::Vector3d::Zero();
    Eigen::Vector3d dynamic_target_position_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d goal_alignment_hold_pos_ = Eigen::Vector3d::Zero();
    double goal_alignment_command_yaw_       = 0.0;
    double goal_alignment_target_yaw_        = 0.0;
    double goal_alignment_stable_elapsed_    = 0.0;
    Eigen::Vector3d arrival_hover_pos_ = Eigen::Vector3d::Zero();
    double arrival_hover_yaw_          = 0.0;
    double arrival_target_yaw_         = 0.0;
    Eigen::Vector3d desire_pos_        = Eigen::Vector3d::Zero();
    Eigen::Vector3d desire_vel_        = Eigen::Vector3d::Zero();
    Eigen::Vector3d desire_acc_        = Eigen::Vector3d::Zero();
    double last_yaw_                   = 0.0;
    double ctrl_time_                  = 0.0;

    Poly5Solver poly_x_;
    Poly5Solver poly_y_;
    Poly5Solver poly_z_;
    quadrotor_msgs::PositionCommand last_control_msg_;
    bool has_last_control_msg_ = false;

    std::vector<EndState> last_all_endstates_;
    std::vector<float> last_scores_;
    mutable std::mutex mutex_;
};

}  // namespace yopo_planner

#endif  // YOPO_PLANNER_YOPO_PLANNER_H_
