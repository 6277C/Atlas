#pragma once

#include <vector>

namespace atlas {

// Result of a 1D motion profile computation
struct MotionProfile1DResult {
  std::vector<double> t; // time
  std::vector<double> x; // position
  std::vector<double> v; // velocity
  std::vector<double> a; // acceleration
};

// Stateless 1D motion profile generator
class MotionProfile1D {
public:
  MotionProfile1DResult compute(float maxSpeed, float timeToMaxSpeed, float dt, float goal);
};

} // namespace atlas
