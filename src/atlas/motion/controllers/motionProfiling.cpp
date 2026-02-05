#include "atlas/motion/controllers/motionProfiling.hpp"
#include <cmath>

namespace atlas {

MotionProfile1DResult MotionProfile1D::compute(float maxSpeed, float timeToMaxSpeed, float dt, float goal) {
  MotionProfile1DResult result;

  // initialize
  result.t = {0.0};
  result.x = {0.0};
  result.v = {0.0};
  result.a = {0.0};

  const double accel = maxSpeed / timeToMaxSpeed;

  double timeAtMaxSpeed = goal / maxSpeed - timeToMaxSpeed;
  double timeFromMaxSpeed;
  double totalTime;
  double profileMaxSpeed;

  // triangular profile
  if (maxSpeed * timeToMaxSpeed > goal) {
    timeToMaxSpeed = std::sqrt(goal / accel);
    timeFromMaxSpeed = timeToMaxSpeed;
    totalTime = 2.0 * timeToMaxSpeed;
    profileMaxSpeed = accel * timeToMaxSpeed;
  }
  // trapezoidal profile
  else {
    timeFromMaxSpeed = timeToMaxSpeed + timeAtMaxSpeed;
    totalTime = timeFromMaxSpeed + timeToMaxSpeed;
    profileMaxSpeed = maxSpeed;
  }

  while (result.t.back() < totalTime) {
    double t = result.t.back() + dt;
    result.t.push_back(t);

    if (t < timeToMaxSpeed) {
      result.a.push_back(accel);
      result.v.push_back(accel * t);
    } else if (t < timeFromMaxSpeed) {
      result.a.push_back(0.0);
      result.v.push_back(profileMaxSpeed);
    } else if (t < totalTime) {
      double decelTime = t - timeFromMaxSpeed;
      result.a.push_back(-accel);
      result.v.push_back(profileMaxSpeed - accel * decelTime);
    } else {
      result.a.push_back(0.0);
      result.v.push_back(0.0);
    }
    result.x.push_back(result.x.back() + result.v.back() * dt);
  }

  return result;
}

} // namespace atlas
