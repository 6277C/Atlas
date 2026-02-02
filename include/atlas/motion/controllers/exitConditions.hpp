#pragma once

namespace atlas {
class ExitConditions {
public:
  float settleError = 0;
  float settleTime = 0;
  float timeout = 0;
  float time_spent_settled = 0;
  float time_spent_running = 0;

  bool isSettled();
};
} // namespace atlas