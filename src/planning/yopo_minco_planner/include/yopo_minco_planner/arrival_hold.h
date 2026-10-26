#pragma once
#include <Eigen/Core>
#include <algorithm>
#include <cmath>
#include <utility>

namespace yopo_minco_planner {
// Static-goal arrival semantics from YOPO-Simple: latch measured position,
// then hold translation and turn along the shortest yaw arc. No ROS or GPU.
struct ArrivalHold {
    bool active=false;
    Eigen::Vector3d position=Eigen::Vector3d::Zero();
    double yaw=0, target_yaw=0;
    static double wrap(double angle) { return std::atan2(std::sin(angle),std::cos(angle)); }
    void reset() { active=false; }
    bool enter(const Eigen::Vector3d& actual, const Eigen::Vector3d& goal,
               double measured_yaw, double radius, double min_horizontal_distance) {
        if (active || (actual-goal).norm()>=radius) return false;
        active=true; position=actual; yaw=wrap(measured_yaw);
        const Eigen::Vector3d delta=goal-position;
        target_yaw=delta.head<2>().norm()>=min_horizontal_distance
            ? std::atan2(delta.y(),delta.x()) : yaw;
        return true;
    }
    std::pair<double,double> advance(double dt, double rate_limit) {
        if (!(dt>0) || !std::isfinite(dt)) return {yaw,0};
        const double step=std::clamp(wrap(target_yaw-yaw),-rate_limit*dt,rate_limit*dt);
        yaw=wrap(yaw+step);
        return {yaw,step/dt};
    }
};
}  // namespace yopo_minco_planner
