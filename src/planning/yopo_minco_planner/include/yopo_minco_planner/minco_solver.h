#pragma once
#include <Eigen/Core>
#include <array>

namespace yopo_minco_planner {
// PVA rows are position, velocity, acceleration; columns are world x/y/z.
class MincoSolver {
 public:
  bool solve(const Eigen::Matrix3d& head, const Eigen::Matrix3d& tail,
             const Eigen::Vector3d& inner, const Eigen::Vector2d& durations);
  Eigen::Vector3d evaluate(double t, int derivative = 0) const;
  double duration() const { return durations_.sum(); }
  bool valid() const { return valid_; }
  const Eigen::Matrix<double,12,3>& coefficients() const { return coefficients_; }
 private:
  Eigen::Matrix<double,12,3> coefficients_ = Eigen::Matrix<double,12,3>::Zero();
  Eigen::Vector2d durations_ = Eigen::Vector2d::Zero();
  bool valid_ = false;
};
}
