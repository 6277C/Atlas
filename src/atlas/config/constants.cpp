#include "constants.hpp"
PIDConstants globalAngularParams {1, 1, 1, 3};
PIDConstants globalLinearParams {1, 1, 1, 1};
PIDConstants relativeAngularParams {1, 1, 1, 3};
PIDConstants relativeLinearParams {1, 1, 1, 1};

/**
 * @brief Config for the AMCL
 * 
 * @param numParticles Total number of particles for localization
 * @param odomSigmaXY Linear distribution for init
 * @param odomSigmaAngle Angular distribution for init
 * @param distSigma Noise for distance sensors
 * @param gpsSigm Noise for gps
 * @param resamleThreshold Minimum confidence
 * @param gpsEnableThreshold Confidence threshold to enable GPS
 * @param gpsDisableThreshold Confidence threshold to disable GPS
 */
AMCLConfig amclConfig {
  150,
  20,
  1000,
  0.4f,
  0.02f,
  6.0f,
  8.0f,
  0.4f,
  0.6f,
  0.75f
};