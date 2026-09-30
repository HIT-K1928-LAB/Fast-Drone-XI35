#include "yopo_minco_planner/recovery_gate.h"
#include <iostream>
#include <stdexcept>
using G=yopo_minco_planner::RecoveryGate;
static int checks=0;
void check(bool ok,const char* why) { ++checks; if (!ok) throw std::runtime_error(why); }
void stable(G& g,double base) {
  g.update(base+1.01,{{0,0,1}},0);
  g.update(base+2.02,{{0,0,1}},0);
  check(g.phase==G::Phase::REPLAN,"stable hover enters replan");
}
int main() {
 try {
  G g; g.start(1); check(g.phase==G::Phase::TRACK,"start");
  check(g.begin(10,1),"corridor starts recovery");
  g.update(10.5,{{0,0,1}},0); check(g.phase==G::Phase::WAIT_HOVER,"release delay");
  stable(g,10);
  for(int i=0;i<4;++i) g.candidate(12.1+i*.11,20+i*.11,true);
  check(!g.freshReady(12.44),"four frames insufficient");
  g.candidate(12.54,20.44,true); check(g.freshReady(12.54),"five distinct frames");
  check(!g.freshReady(13),"ready expires");
  g.candidate(13,21,true); check(g.good_frames==1,"gap resets streak");
  g.candidate(13.11,21,true); check(g.good_frames==1,"duplicate frame ignored");
  g.candidate(13.2,20,true); check(g.good_frames==1,"old frame ignored");
  g.candidate(13.22,21.22,false); check(g.good_frames==0,"rejection resets streak");
  for(int i=0;i<5;++i) g.candidate(13.3+i*.11,22+i*.11,true);
  check(g.freshReady(13.74),"ready again");
  auto revision=g.revision;
  g.update(13.75,{{0,0,1}},.2);
  check(g.phase==G::Phase::WAIT_HOVER && g.revision!=revision,"drift invalidates in-flight inference");
  g.stop("operator"); g.candidate(14,23,true);
  check(g.phase==G::Phase::STOPPED,"stop never resumes");
  g.start(1);g.begin(20,1);g.update(50,{{0,0,1}},0);
  check(g.phase==G::Phase::STOPPED,"timeout");
  g.start(1);g.begin(100,1);g.resumed();g.begin(102,1);g.resumed();
  check(!g.begin(104,1),"no-progress terminates repeated starts");
  g.start(2);g.begin(200,2);g.resumed();g.begin(203,1.8);g.resumed();g.begin(206,1.6);g.resumed();
  check(!g.begin(209,1.4),"attempt limit");
  g.start(1);g.begin(300,1);stable(g,300);
  g.update(302.1,{{.11,0,1}},0);
  check(g.phase==G::Phase::WAIT_HOVER,"position drift resets hold");
  std::cout << "PASS " << checks << " recovery gate checks\n";
 } catch(const std::exception& e) { std::cerr<<e.what()<<'\n'; return 1; }
}
