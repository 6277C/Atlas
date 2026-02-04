#pragma once

namespace atlas {
class ExitCondition {
public:
  ExitCondition(const float range, const int time);
  bool getExit();
  bool compute(const float input);
  void reset();
protected:
  const float range;
  const int time;
  int startTime = -1;
  bool done = false;

};

extern ExitCondition relativeAngularExit;
} // namespace atlas