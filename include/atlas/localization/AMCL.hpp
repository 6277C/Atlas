#pragma once

#include "fieldMap.hpp"
#include "atlas/config/constants.hpp"
#include "atlas/localization/pose.hpp"
#include "pros/distance.hpp"
#include <vector>
namespace atlas {

struct Particle {
  Pose pose;
  float weight;
};

class FieldMap;

class AMCLDistanceSensor {
  public:
    /**
     * @brief Construct a new AMCLDistanceSensor object
     * 
     * @param sensor The distance sensor to use
     * @param offsetForward The vertical offset of the sensor (positive forward)
     * @param offsetSideways The horizontal offset of the sensor (positive right)
     * @param sensorAngleDeg The angle of the sensor, relative to the forward direction of the robot, in degrees (positive clockwise)
     */
    AMCLDistanceSensor(pros::Distance *sensor, float offsetForward, float offsetSideways, float sensorAngleDeg);
    float offsetForward;
    float offsetSideways;
    float sensorAngleDeg;
    pros::Distance *sensor = nullptr;
};

class AMCL {
public:
  explicit AMCL(const AMCLConfig &cfg, FieldMap* map = nullptr);
  void initialize(const Pose &start, float spreadXY, float spreadAngle);

  void updateOdomRad(const Pose &odomDeltaRad);
  void updateDistance(float sensorDist, float sensorAngleDeg, float offsetForward, float offsetSideways);
  void updateGPS(const Pose &gpsPose);

  Pose getEstimate() const;
  float getConfidence() const;

  void initAMCL();

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

extern AMCL amcl;              // global filter
extern Pose amclPose;          // latest estimate
extern float amclConfidence;   // Neff-based confidence
}; // namespace atlas