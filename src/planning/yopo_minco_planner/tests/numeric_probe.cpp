#include "yopo_minco_planner/yopo_planner.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <string>
using namespace yopo_minco_planner;
int main(int argc,char** argv) {
  if(argc!=3) return 2;
  std::ifstream in(argv[2]);std::string mode=argv[1];std::cout<<std::setprecision(17);
  if(mode=="solve") {
    Eigen::Matrix3d h,t;Eigen::Vector3d inner;Eigen::Vector2d d;
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) in>>h(r,c);
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) in>>t(r,c);
    for(int i=0;i<3;++i) in>>inner[i];
    for(int i=0;i<2;++i) in>>d[i];
    if(!in) return 3;
    MincoSolver solver;if(!solver.solve(h,t,inner,d)) return 4;
    for(int r=0;r<12;++r) for(int c=0;c<3;++c) std::cout<<solver.coefficients()(r,c)<<' ';
    double time;
    while(in>>time) for(int derivative=0;derivative<=4;++derivative) {
      auto v=solver.evaluate(time,derivative);for(int j=0;j<3;++j) std::cout<<v[j]<<' ';
    }
  } else if(mode=="decode") {
    PlannerConfig config;in>>config.velocity;YopoPlanner planner(config);NetworkOutput out;
    for(float& v:out.endstate) in>>v;
    if(!in) return 3;
    for(int i=0;i<15;++i) {
      auto c=planner.decode(out,i);
      for(int j=0;j<3;++j) std::cout<<c.inner[j]<<' ';
      for(int r=0;r<3;++r) for(int j=0;j<3;++j) std::cout<<c.tail(r,j)<<' ';
      std::cout<<c.durations[0]<<' '<<c.durations[1]<<' ';
    }
  } else if(mode=="obs") {
    PlannerConfig config;in>>config.velocity;YopoPlanner planner(config);PlanningState s;
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) in>>s.head(r,c);
    for(int r=0;r<3;++r) for(int c=0;c<3;++c) in>>s.rotation_wc(r,c);
    for(int j=0;j<3;++j) in>>s.goal[j];
    if(!in) return 3;
    for(float v:planner.observation(s)) std::cout<<v<<' ';
  } else if(mode=="guards") {
    PlannerConfig config;config.max_speed=100;config.max_acceleration=100;config.max_jerk=1000;
    config.min_height=-100;config.max_height=100;config.height_band=100;config.test_radius=100;
    YopoPlanner planner(config);PlanningState s;s.head(0,2)=1;s.origin={0,0,1};s.goal={1,0,1};
    NetworkOutput out;out.radius.fill(0);Plan result;std::string why;
    if(planner.plan(s,out,&result,&why)) return 10; // no admitted corridor
    for(int k=0;k<10;++k) for(int i=0;i<15;++i) out.radius[k*15+i]=0.8;
    out.score[7]=9;
    if(!planner.plan(s,out,&result,&why)||result.action!=7) return 11;
    for(int k=0;k<10;++k) out.radius[k*15+7]=0;
    out.score[3]=8;
    if(!planner.plan(s,out,&result,&why)||result.action!=3) return 12;
    config.test_radius=0.1;YopoPlanner restrictive(config);
    if(restrictive.plan(s,out,&result,&why)) return 13;
    MincoSolver solver;
    if(solver.solve(Eigen::Matrix3d::Zero(),Eigen::Matrix3d::Zero(),Eigen::Vector3d::Zero(),{0.,1.})) return 14;
    std::cout<<"guard tests passed";
  } else return 2;
  std::cout<<'\n';return 0;
}
