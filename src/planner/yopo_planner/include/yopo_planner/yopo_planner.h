#ifndef YOPO_PLANNER_YOPO_PLANNER_H_
#define YOPO_PLANNER_YOPO_PLANNER_H_

#include <array>
#include <mutex>
#include <vector>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <nav_msgs/Odometry.h>
#include <quadrotor_msgs/PositionCommand.h>

namespace yopo_planner {

struct YopoParams {
  int image_height = 96;
  int image_width = 160;
  int horizon_num = 5;
  int vertical_num = 3;
  int radio_num = 1;
  int traj_num = 15;
  double velocity = 6.0;
  double vel_max_train = 6.0;
  double acc_max_train = 6.0;
  double horizon_camera_fov = 90.0;
  double vertical_camera_fov = 60.0;
  double horizon_anchor_fov = 30.0;
  double vertical_anchor_fov = 30.0;
  double radio_range = 5.0;
  double goal_length = 10.0;
  double min_depth = 0.04;
  double max_depth = 20.0;
  double ctrl_dt = 0.02;
  double arrive_distance = 5.0;
  double pitch_angle_deg = 0.0;
  bool plan_from_reference = false;
};

struct Poly5Solver {
  Eigen::Matrix<double, 6, 1> coeff = Eigen::Matrix<double, 6, 1>::Zero();

  void reset(double pos0, double vel0, double acc0, double pos1, double vel1,
             double acc1, double tf);
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
  Eigen::Vector3d goal() const { return goal_; }
  bool odomInitialized() const { return odom_init_; }
  bool arrived() const { return arrive_; }
  bool hasTrajectory() const { return has_trajectory_; }

  void updateOdometry(const nav_msgs::Odometry& odom);
  std::array<float, 1 * 9 * 3 * 5> prepareObsInput();
  void updateTrajectory(const std::array<float, 1 * 9 * 3 * 5>& endstate_pred,
                        const std::array<float, 1 * 3 * 5>& score_pred,
                        bool return_all_preds);
  bool fillControlCommand(quadrotor_msgs::PositionCommand* cmd);
  quadrotor_msgs::PositionCommand emptyControlCommand() const;

  std::vector<Eigen::Vector3f> bestTrajectoryPoints(int samples) const;
  std::vector<Eigen::Vector3f> latticeTrajectoryPoints(int samples) const;
  std::vector<Eigen::Vector4f> allTrajectoryPoints(int samples) const;

 private:
  struct EndState {
    Eigen::Vector3d pos = Eigen::Vector3d::Zero();
    Eigen::Vector3d vel = Eigen::Vector3d::Zero();
    Eigen::Vector3d acc = Eigen::Vector3d::Zero();
  };

  void buildLattice();
  EndState predToEndstate(const std::array<float, 1 * 9 * 3 * 5>& pred,
                          int action_id) const;
  EndState predToEndstateByLattice(const Eigen::Matrix<double, 9, 1>& pred,
                                   int lattice_id) const;
  Eigen::Vector3d currentStartPos() const;
  Eigen::Vector3d currentStartVel() const;
  static double wrapToPi(double angle);
  static std::pair<double, double> calculateYaw(const Eigen::Vector3d& vel_dir,
                                                const Eigen::Vector3d& goal_dir,
                                                double last_yaw, double dt);
  static Eigen::Matrix<double, 6, 1> polyCoefficients(double pos0, double vel0,
                                                       double acc0, double pos1,
                                                       double vel1, double acc1,
                                                       double tf);

  YopoParams params_;
  double vel_max_ = 6.0;
  double acc_max_ = 6.0;
  double segment_time_ = 10.0 / 6.0;
  double yaw_diff_ = 0.0;
  double pitch_diff_ = 0.0;
  Eigen::Matrix3d rotation_bc_ = Eigen::Matrix3d::Identity();
  Eigen::Matrix3d rotation_wc_ = Eigen::Matrix3d::Identity();

  std::vector<Eigen::Vector3d> lattice_pos_;
  std::vector<Eigen::Vector2d> lattice_angle_;
  std::vector<Eigen::Matrix3d> lattice_rbp_;

  nav_msgs::Odometry odom_;
  bool odom_init_ = false;
  bool desire_init_ = false;
  bool arrive_ = false;
  bool has_trajectory_ = false;
  Eigen::Vector3d goal_ = Eigen::Vector3d(50.0, 0.0, 2.0);
  Eigen::Vector3d desire_pos_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d desire_vel_ = Eigen::Vector3d::Zero();
  Eigen::Vector3d desire_acc_ = Eigen::Vector3d::Zero();
  double last_yaw_ = 0.0;
  double ctrl_time_ = 0.0;

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
