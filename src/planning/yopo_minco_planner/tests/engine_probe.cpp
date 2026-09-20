#include "yopo_minco_planner/yopo_engine.h"
#include <fstream>
#include <iostream>
#include <chrono>
using namespace yopo_minco_planner;
int main(int argc,char** argv) {
  // Pure inference: no ROS node, subscriptions, publishers or flight commands.
  if(argc!=4) return 2;
  std::ifstream in(argv[2],std::ios::binary);Depth depth;Observation obs;NetworkOutput out;
  in.read(reinterpret_cast<char*>(depth.data()),sizeof(depth));in.read(reinterpret_cast<char*>(obs.data()),sizeof(obs));
  if(!in || in.peek()!=std::char_traits<char>::eof()) return 3;
  YopoEngine engine;if(!engine.load(argv[1])) return 4;
  for(int i=0;i<3;++i) if(!engine.infer(depth,obs,&out)) return 5;
  auto start=std::chrono::steady_clock::now();
  for(int i=0;i<20;++i) if(!engine.infer(depth,obs,&out)) return 6;
  double ms=std::chrono::duration<double,std::milli>(std::chrono::steady_clock::now()-start).count()/20;
  std::ofstream f(argv[3],std::ios::binary);
  f.write(reinterpret_cast<char*>(out.endstate.data()),sizeof(out.endstate));
  f.write(reinterpret_cast<char*>(out.score.data()),sizeof(out.score));
  f.write(reinterpret_cast<char*>(out.radius.data()),sizeof(out.radius));
  std::cout<<"mean_host_inference_ms="<<ms<<'\n';return f?0:7;
}
