#include "constants.hpp"
PIDConstants globalAngularParams {1, 1, 1, 3};
PIDConstants globalLinearParams {1, 1, 1, 1};
PIDConstants relativeAngularParams {1, 1, 1, 3};
PIDConstants relativeLinearParams {1, 1, 1, 1};
AMCLConfig AMCLconfig {150, 0.4f, 0.02f, 1.5f, 8.0f, 0.5f};