#pragma once
#include <optional>
namespace atlas {
class PID {
public:
  float kP = 0;
  float kI = 0;
  float kD = 0;
  float antiWindup = 0;

  PID(float kP, float kI, float kD, float antiWindup, bool signFlipReset = true);

  float compute(float error);

  void setSignFlipReset(bool signFlipReset);

  bool getSignFlipReset();

  void reset();

private:
  float error = 0;
  float integral = 0;
  std::optional<float> prevTime = std::nullopt;
  float prevError = 0;
  float output = 0;
  bool signFlipReset = false;
};
} // namespace atlas