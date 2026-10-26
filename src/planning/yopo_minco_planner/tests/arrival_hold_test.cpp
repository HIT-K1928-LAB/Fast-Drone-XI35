#include "yopo_minco_planner/arrival_hold.h"
#include <iostream>
#include <stdexcept>
using yopo_minco_planner::ArrivalHold;
void require(bool ok) { if (!ok) throw std::runtime_error("arrival hold regression"); }
int main() {
    ArrivalHold h;
    const Eigen::Vector3d origin(0,0,1), goal(0,0.5,1);
    require(!h.enter(origin,Eigen::Vector3d(0,0,1.6),0,0.59,0.3));
    require(h.enter(origin,goal,0,0.59,0.3));
    require(!h.enter(Eigen::Vector3d(2,2,2),goal,1,0.59,0.3));
    require((h.position-origin).norm()==0); // no chasing drift or leaving hold
    auto first=h.advance(0.02,0.8);
    require(std::abs(first.first-0.016)<1e-12 && std::abs(first.second-0.8)<1e-12);
    for (int i=0;i<500;++i) require(std::abs(h.advance(0.02,0.8).second)<=0.8+1e-12);
    require(std::abs(h.yaw-std::atan2(0.5,0.0))<1e-12);
    h.reset(); require(!h.active);
    require(h.enter(origin,Eigen::Vector3d(0.1,0.1,1),1.2,0.59,0.3));
    require(h.advance(0.02,0.8).second==0 && std::abs(h.yaw-1.2)<1e-12);
    h.reset();
    require(h.enter(origin,Eigen::Vector3d(-0.5,-0.001,1),3.13,0.59,0.3));
    require(h.advance(0.02,0.8).second>0); // shortest arc across +/-pi
    require(h.advance(0,0.8).second==0);
    std::cout << "arrival hold: radius, latch, reset, yaw rate, dead zone, wrap passed\n";
}
