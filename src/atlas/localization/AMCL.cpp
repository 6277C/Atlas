#include "atlas/localization/amcl.hpp"
#include "atlas/localization/fieldMap.hpp"
#include "atlas/localization/pose.hpp"
#include "atlas/config/constants.hpp"
#include "atlas/util.hpp"

#include <cmath>
#include <random>

namespace atlas {

// --------------------------------------------------
// Random utilities
// --------------------------------------------------

static std::default_random_engine rng;
static std::normal_distribution<float> unitNormal(0.0f, 1.0f);

static float randNormal(float sigma) {
  return unitNormal(rng) * sigma;
}

static float gaussian(float x, float sigma) {
  return std::exp(-(x * x) / (2.0f * sigma * sigma));
}

// --------------------------------------------------
// Constructor
// --------------------------------------------------

AMCL::AMCL(const AMCLConfig& cfg, FieldMap* map)
  : cfg(cfg), map(map) {
  particles.resize(cfg.numParticles);
}

// --------------------------------------------------
// Initialization
// --------------------------------------------------

void AMCL::initialize(const Pose& start,
                      float spreadXY,
                      float spreadAngle) {
  for (auto& p : particles) {
    p.pose = Pose(
      start.x + randNormal(spreadXY),
      start.y + randNormal(spreadXY),
      start.angle + randNormal(spreadAngle)
    );

    p.weight = 1.0f / cfg.numParticles;
  }
}

// --------------------------------------------------
// Prediction (odometry)
// --------------------------------------------------

void AMCL::updateOdomRad(const Pose& odomDeltaRad) {
  predict(odomDeltaRad);
}

void AMCL::predict(const Pose& odomDelta) {
  for (auto& p : particles) {
    // Apply odometry delta
    p.pose = p.pose + odomDelta;

    // Motion noise (XY dominant, tiny angle noise)
    p.pose.x     += randNormal(cfg.odomSigmaXY);
    p.pose.y     += randNormal(cfg.odomSigmaXY);
    p.pose.angle += randNormal(degToRad(cfg.odomSigmaAngle));
  }
}

// --------------------------------------------------
// Distance sensor update
// --------------------------------------------------

void AMCL::updateDistance(float sensorDist,
                          float sensorAngleDeg) {
  for (auto& p : particles) {
    float expected = map->raycast(
      p.pose.x,
      p.pose.y,
      p.pose.angle + degToRad(sensorAngleDeg)
    );

    float error = sensorDist - expected;
    p.weight *= gaussian(error, cfg.distSigma);
  }

  normalizeWeights();

  if (effectiveParticleCount() <
      cfg.resampleThreshold * cfg.numParticles) {
    resample();
  }
}

// --------------------------------------------------
// GPS update (weak global correction)
// --------------------------------------------------

void AMCL::updateGPS(const Pose& gpsPose) {
  for (auto& p : particles) {
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
  for (const auto& p : particles)
    sum += p.weight;

  if (sum <= 0.0f) return;

  for (auto& p : particles)
    p.weight /= sum;
}

float AMCL::effectiveParticleCount() const {
  float sumSq = 0.0f;
  for (const auto& p : particles)
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

  for (const auto& p : particles) {
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

float AMCL::getConfidence() const {
  return effectiveParticleCount() / cfg.numParticles;
}

} // namespace atlas
