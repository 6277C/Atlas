#pragma once

struct PIDConstants {
    float kP;
    float kI;
    float kD;
    float antiWindup;
};

struct AMCLConfig {
  int numParticles; // number of particles
  int distRefresh; // ms between distance sensor updates
  int GPSRefresh; // ms between gps updates

  // Motion model noise
  float odomSigmaXY; // amount of noise in inches
  float odomSigmaAngle; // amount of noise in degrees

  // Sensor model noise
  float distSigma; // amount of noise in inches
  float gpsSigma;  // amount of noise in inches

  // Resampling
  float resampleThreshold;
  float gpsEnableThresh;
  float gpsDisableThresh;
};
extern PIDConstants relativeAngularParams;
extern PIDConstants relativeLinearParams;
extern PIDConstants globalAngularParams;
extern PIDConstants globalLinearParams;
extern AMCLConfig amclConfig;   // ⭐ THIS WAS MISSING
