#include "atlas/motion/controllers/exitConditions.hpp"
namespace atlas {

bool ExitConditions::isSettled() {
  if (time_spent_running > timeout && timeout != 0) {
    return true;
  }
  if (time_spent_settled > settleTime) {
    return true;
  }
  return false;
}
} // namespace atlas