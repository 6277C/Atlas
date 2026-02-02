#pragma once

namespace atlas {
class DriveCurve {
public:
  // virtual process of a input and output
  virtual float curve(float input) = 0;
};

// see https://www.desmos.com/calculator/umicbymbnl for an interactive graph
class ExpoDriveCurve : public DriveCurve {
public:
  ExpoDriveCurve(float deadband, float minOutput, float curve);
  // curve an input
  float curve(float input);

private:
  const float deadband = 0;
  const float minOutput = 0;
  const float curveGain = 1;
};
} // namespace atlas