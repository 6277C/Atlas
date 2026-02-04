#include "atlas/motion/controllers/exitConditions.hpp"
#include "pros/rtos.hpp"
#include <cmath>
namespace atlas {
ExitCondition::ExitCondition(const float range, const int time)
    : range(range),
      time(time) {}

bool ExitCondition::getExit() { return done; }

bool ExitCondition::compute(const float input) {
    const int curTime = pros::millis();
    if (std::fabs(input) > range) startTime = -1;
    else if (startTime == -1) startTime = curTime;
    else if (curTime >= startTime + time) done = true;
    return done;
}

void ExitCondition::reset() {
    startTime = -1;
    done = false;
} 

ExitCondition relativeAngularExit(1, 100);
} // namespace atlas