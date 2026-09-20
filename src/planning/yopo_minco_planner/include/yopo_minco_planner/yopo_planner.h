#pragma once
#include "yopo_minco_planner/minco_solver.h"
#include "yopo_minco_planner/yopo_engine.h"
#include <Eigen/Geometry>
#include <string>
namespace yopo_minco_planner {
struct PlannerConfig {
  double velocity=0.5, training_velocity=6., training_acceleration=6.;
  double tail_range=10., horizontal_fov=90., vertical_fov=60.;
  double horizontal_anchor_fov=30., vertical_anchor_fov=30.;
  double duration_min=0.1, radius_lambda=2., safe_radius=0.35;
  double max_speed=0.8, max_acceleration=1.5, max_jerk=10.;
  double min_height=0.6, max_height=1.5, height_band=0.3, test_radius=3.;
  double max_duration=120., check_dt=0.02;
};
struct PlanningState {
  Eigen::Matrix3d head=Eigen::Matrix3d::Zero();
  Eigen::Matrix3d rotation_wc=Eigen::Matrix3d::Identity();
  Eigen::Vector3d goal=Eigen::Vector3d::Zero(), origin=Eigen::Vector3d::Zero();
};
struct Candidate {
  Eigen::Vector3d inner;
  Eigen::Matrix3d tail;
  Eigen::Vector2d durations;
};
struct Plan {
  MincoSolver trajectory;
  int action=-1;
  double score=0., corridor_mu=0.;
};
class YopoPlanner {
 public:
  explicit YopoPlanner(PlannerConfig config);
  Observation observation(const PlanningState& state) const;
  Candidate decode(const NetworkOutput& out,int image_index) const;
  bool plan(const PlanningState& state,const NetworkOutput& out,Plan* plan,std::string* reason) const;
  bool check(const Eigen::Vector3d& p,const Eigen::Vector3d& v,const Eigen::Vector3d& a,
             const Eigen::Vector3d& j,const Eigen::Vector3d& origin) const;
  static double wrap(double x);
  static std::pair<double,double> yaw(const Eigen::Vector3d& velocity,const Eigen::Vector3d& goal,double last,double dt);
 private:
  PlannerConfig c_;
  std::array<Eigen::Vector2d,15> angles_;
  std::array<Eigen::Matrix3d,15> rotations_;
  double acc_scale_,piece_time_;
};
}
