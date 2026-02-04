#pragma once
#include "atlas/config/constants.hpp"
#include <optional>
namespace atlas {
class PID {
public:
  PID(float kP, float kI, float kD, float antiWindup, bool signFlipReset = true);

  float compute(float error);

  void setSignFlipReset(bool signFlipReset);

  void setConstants(const PIDConstants& constants);

  bool getSignFlipReset();

  void reset();

private:
  float kP = 0;
  float kI = 0;
  float kD = 0;
  float antiWindup = 0;
  float error = 0;
  float integral = 0;
  std::optional<float> prevTime = std::nullopt;
  float prevError = 0;
  float output = 0;
  bool signFlipReset = false;
};

extern PID angularPID;
extern PID linearPID;
} // namespace atlas