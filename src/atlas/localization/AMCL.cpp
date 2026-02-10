#include "atlas/localization/amcl.hpp"
#include "atlas/config/constants.hpp"
#include "atlas/config/devices.hpp"
#include "atlas/localization/fieldMap.hpp"
#include "atlas/localization/pose.hpp"
#include "atlas/util.hpp"
#include "pros/rtos.hpp"

#include <cmath>
#include <cstdint>
#include <random>

namespace atlas {
AMCLDistanceSensor::AMCLDistanceSensor(pros::Distance *sensor, float offsetForward, float offsetSideways, float sensorAngleDeg) {
  this->sensor = sensor;
  this->offsetForward = offsetForward;
  this->offsetSideways = offsetSideways;
  this->sensorAngleDeg = sensorAngleDeg;
}
// --------------------------------------------------
// Globals
// --------------------------------------------------
AMCL amcl(amclConfig, &testMap);
Pose amclPose;        // latest estimate
float amclConfidence; // Neff-based confidence
static pros::Task *amclTask = nullptr;

// --------------------------------------------------
// Random utilities
// --------------------------------------------------

static std::default_random_engine rng;
static std::normal_distribution<float> unitNormal(0.0f, 1.0f);

float AMCL::randNormal(float sigma) const {
  static std::random_device rd;
  static std::mt19937 rng(rd());
  static std::normal_distribution<float> unitNormal(0.0f, 1.0f);
  return unitNormal(rng) * sigma;
}
float AMCL::gaussian(float x, float sigma) const { return std::exp(-(x * x) / (2.0f * sigma * sigma)); }

// --------------------------------------------------
// Constructor
// --------------------------------------------------

AMCL::AMCL(const AMCLConfig &cfg, FieldMap *map) : cfg(cfg), map(map) { particles.resize(cfg.numParticles); }

// --------------------------------------------------
// Initialization
// --------------------------------------------------

void AMCL::initialize(const Pose &start, float spreadXY, float spreadAngleRad) {
  for (auto &p : particles) {
    p.pose =
        Pose(start.x + randNormal(spreadXY), start.y + randNormal(spreadXY), start.angle + randNormal(spreadAngleRad));

    p.weight = 1.0f / cfg.numParticles;
  }
}

// --------------------------------------------------
// Prediction (odometry)
// --------------------------------------------------

void AMCL::updateOdomRad(const Pose &odomDeltaRad) { predict(odomDeltaRad); }

void AMCL::predict(const Pose &odomDelta) {
  for (auto &p : particles) {
    p.pose = p.pose.addWithAngle(odomDelta);
    // Motion noise
    p.pose.x += randNormal(cfg.odomSigmaXY);
    p.pose.y += randNormal(cfg.odomSigmaXY);
    p.pose.angle += randNormal(degToRad(cfg.odomSigmaAngle));

    // wrap angle after noise
    p.pose.angle = wrapAngleRad(p.pose.angle);
  }
}

// --------------------------------------------------
// Distance sensor update
// --------------------------------------------------

void AMCL::updateDistance(float sensorDist,            // inches, user-facing
                          float sensorAngleDeg,        // degrees relative to robot front
                          float offsetForward = 0.0f,  // inches forward/back
                          float offsetSideways = 0.0f) // inches right/left
{
  // Convert sensor angle from degrees to radians
  float sensorAngleRad = degToRad(sensorAngleDeg);

  for (auto &p : particles) {
    // Compute actual sensor position for this particle
    // Forward/back offset along particle heading
    // Right/left offset perpendicular to heading
    float sensorX = p.pose.x + offsetSideways * std::cos(p.pose.angle) + offsetForward * std::sin(p.pose.angle);

    float sensorY = p.pose.y + offsetForward * std::cos(p.pose.angle) - offsetSideways * std::sin(p.pose.angle);

    // Raycast from sensor position along sensor angle
    float expected = map->raycast(sensorX, sensorY, p.pose.angle + sensorAngleRad);

    // Update particle weight using Gaussian
    float error = sensorDist - expected;
    p.weight *= gaussian(error, cfg.distSigma);
  }

  // Normalize weights and resample if needed
  normalizeWeights();
}

// --------------------------------------------------
// GPS update (weak global correction)
// --------------------------------------------------

void AMCL::updateGPS(const Pose &gpsPose) {
  for (auto &p : particles) {
    float distError = p.pose.distance(gpsPose);
    p.weight *= gaussian(distError, cfg.gpsSigma);
  }
  normalizeWeights();
}

// --------------------------------------------------
// Weight handling
// --------------------------------------------------

void AMCL::normalizeWeights() {
  float sum = 0.0f;
  for (const auto &p : particles)
    sum += p.weight;

  if (sum <= 0.0f)
    return;

  for (auto &p : particles)
    p.weight /= sum;
}

float AMCL::effectiveParticleCount() const {
  float sumSq = 0.0f;
  for (const auto &p : particles)
    sumSq += p.weight * p.weight;

  return 1.0f / sumSq;
}

// --------------------------------------------------
// Resampling (low-variance)
// --------------------------------------------------

void AMCL::resample() {
  std::vector<Particle> newParticles;
  newParticles.reserve(cfg.numParticles);

  float step = 1.0f / cfg.numParticles;
  float r = ((float)rand() / RAND_MAX) * step;

  float c = particles[0].weight;
  int i = 0;

  for (int m = 0; m < cfg.numParticles; m++) {
    float U = r + m * step;
    while (U > c && i < cfg.numParticles - 1) {
      i++;
      c += particles[i].weight;
    }

    newParticles.push_back(particles[i]);
    newParticles.back().weight = step;
  }

  particles = newParticles;
}

// --------------------------------------------------
// State estimate
// --------------------------------------------------

Pose AMCL::getEstimate() const {
  Pose est(0.0f, 0.0f, 0.0f);

  float sinSum = 0.0f;
  float cosSum = 0.0f;

  for (const auto &p : particles) {
    est.x += p.pose.x * p.weight;
    est.y += p.pose.y * p.weight;

    sinSum += std::sin(p.pose.angle) * p.weight;
    cosSum += std::cos(p.pose.angle) * p.weight;
  }

  est.angle = std::atan2(sinSum, cosSum);
  return est;
}

// --------------------------------------------------
// Confidence
// --------------------------------------------------

float AMCL::getConfidence() const { return effectiveParticleCount() / cfg.numParticles; }

// --------------------------------------------------
// Main Loop
// --------------------------------------------------
void AMCL::initAMCL() {
  if (amclTask == nullptr) {
    amclTask = new pros::Task{[] { // initialize AMCL tast
      // INITIALIZE AMCL WITH STARTING POSE AND SPREAD
      atlas::Pose prevOdom = chassis.getPose();
      atlas::Pose curOdom = prevOdom;
      atlas::Pose odomDelta;
      uint32_t lastDistanceUpdate = 0;
      uint32_t lastGPSUpdate = 0;
      bool gpsEnabled = false;
      bool sensorUpdate = false;
      while (true) {
        uint32_t now = pros::millis();

        // odomDelta
        curOdom = chassis.getPose();
        odomDelta = curOdom.minusWithAngle(prevOdom);
        prevOdom = curOdom;

        // particle update
        amcl.updateOdomRad(odomDelta);

        // distance sensor weighting
        if (now - lastDistanceUpdate >= amclConfig.distRefresh) {
          float rightMeasured = rightDistanceSensor.sensor->get_distance() * 0.03937f; // inches
          //Sensor angle may be counter clockwise
          amcl.updateDistance(rightMeasured, rightDistanceSensor.sensorAngleDeg, rightDistanceSensor.offsetForward, rightDistanceSensor.offsetSideways);
          lastDistanceUpdate = now;
          sensorUpdate = true;
        }
        // update AMCL confidence
        amclConfidence = amcl.getConfidence();
        // enable gps when lost
        gpsEnabled = amclConfidence < amclConfig.gpsEnableThresh ||
           (gpsEnabled && amclConfidence < amclConfig.gpsDisableThresh);
        // gps sensor weighting
        if (gpsEnabled && now - lastGPSUpdate >= amclConfig.GPSRefresh){
          // update with gps
          //sensorUpdate = true;
        }
        // resample if confidence is too low
        if (sensorUpdate && amclConfidence < amclConfig.resampleThreshold) {
          amcl.resample();
        }
        // update global pose
        amclPose = amcl.getEstimate();
        // run main loop at 100Hz
        pros::delay(10);
      }
    }};
  }
}
} // namespace atlas
