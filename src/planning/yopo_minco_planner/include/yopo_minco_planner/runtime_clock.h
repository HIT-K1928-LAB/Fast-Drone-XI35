#pragma once
#include <chrono>
namespace yopo_minco_planner {
inline double wallNow() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct RuntimeStamp {
    double wall=0, motion=0;
    bool operator==(const RuntimeStamp& other) const { return wall==other.wall && motion==other.motion; }
};
inline RuntimeStamp runtimeStamp(bool simulation, double sim) {
    const double wall=wallNow(); return {wall, simulation ? sim : wall};
}
inline double seconds(RuntimeStamp a, RuntimeStamp b) { return a.motion-b.motion; }
inline double wallSeconds(RuntimeStamp a, RuntimeStamp b) { return a.wall-b.wall; }
// A clock reset latches until process restart; a pause merely invalidates readiness.
class ClockMonitor {
 public:
    bool observe(double sim, double wall) {
        if (sim<last_) fault_=true;
        if(sim>last_) { last_=sim; advanced_wall_=wall; }
        return !fault_;
    }
    bool ready(double sim, double wall, double timeout) const {
        return !fault_ && sim>0 && last_>0 && sim>=last_ && wall-advanced_wall_<timeout;
    }
 private:
    double last_=0, advanced_wall_=0;
    bool fault_=false;
};
}
