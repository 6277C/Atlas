#include "atlas/motion/controllers/PID.hpp"
#include "atlas/util.hpp"
#include "pros/rtos.hpp"
#include <cmath>
#include <optional>
namespace atlas {
PID::PID(float kP, float kI, float kD, float antiWindup, bool signFlipReset)
    : kP(kP), kI(kI), kD(kD), antiWindup(antiWindup), signFlipReset(signFlipReset) {};

float PID::compute(float error) {
  // calculate time delta
  const double now = pros::millis();
  // if this is the first interation, don't set prevTime
  // if it is not set, then assume dt is 0
  const double dt = (prevTime == std::nullopt) ? 0 : now - *prevTime;
  prevTime = now;

  // calc derivative (change in error / change in time)
  double derivative = (dt != 0) ? (error - prevError) / dt : 0;
  prevError = error;

  // calc integral (error * change in time)
  integral += error * dt;

  // sign flip reset
  if (atlas::sgn(error) != atlas::sgn(prevError) && signFlipReset)
    integral = 0;
  // anti windup
  if (std::fabs(error) > antiWindup && antiWindup != 0)
    integral = 0;
  return error * kP + integral * kI + derivative * kD;
}

void PID::setSignFlipReset(bool signFlipReset) { signFlipReset = signFlipReset; }

bool PID::getSignFlipReset() { return signFlipReset; }

void PID::reset(){
  integral = 0;
  prevError = 0;
}
} // namespace atlas