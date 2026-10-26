#include "yopo_minco_planner/runtime_clock.h"
#include <iostream>
using namespace yopo_minco_planner;
int main() {
    int failures=0;
    auto check=[&](bool x){if(!x) ++failures;};
    RuntimeStamp a{10,1},b{12,1.2};
    check(seconds(b,a)>0.199 && seconds(b,a)<0.201);
    check(wallSeconds(b,a)==2);
    ClockMonitor c;
    check(!c.ready(0,10,.3));
    check(c.observe(1,10)); check(c.ready(1,10.1,.3));
    check(!c.ready(1,10.4,.3));
    check(c.observe(1.1,10.5)); check(c.ready(1.1,10.6,.3));
    check(!c.observe(.5,10.7)); check(!c.ready(2,10.8,.3));
    check(!c.observe(2,10.8));
    std::cout<<"clock policy failures="<<failures<<std::endl;
    return failures?1:0;
}
