#pragma once

#include "fieldMap.hpp"
#include "atlas/config/constants.hpp"
#include "atlas/localization/pose.hpp"
#include <vector>
namespace atlas {
struct Particle {
  Pose pose;
  float weight;
};


class AMCL {
public:
  AMCL(const AMCLConfig &cfg, FieldMap* map);
  void initialize(const Pose &start, float spreadXY, float spreadAngle);

  void updateOdomRad(const Pose &odomDeltaRad);
  void updateDistance(float sensorDist, float sensorAngleDeg);
  void updateGPS(const Pose &gpsPose);

  Pose getEstimate() const;
  float getConfidence() const;

private:
  void predict(const Pose &odomDelta);
  void normalizeWeights();
  void resample();
  float effectiveParticleCount() const;

  float gaussian(float x, float sigma) const;
  float randNormal(float sigma) const;

  std::vector<Particle> particles;
  AMCLConfig cfg;
  FieldMap *map;
};
}; // namespace atlas