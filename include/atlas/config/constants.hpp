#pragma once

struct PIDConstants {
    float kP;
    float kI;
    float kD;
    float antiWindup;
};

struct AMCLConfig {
  int numParticles; // number of particles

  // Motion model noise
  float odomSigmaXY; // amount of noise in inches
  float odomSigmaAngle; // amount of noise in degrees

  // Sensor model noise
  float distSigma; // amount of noise in inches
  float gpsSigma;  // amount of noise in inches

  // Resampling
  float resampleThreshold;
};
extern PIDConstants relativeAngularParams;
extern PIDConstants relativeLinearParams;
extern PIDConstants globalAngularParams;
extern PIDConstants globalLinearParams;