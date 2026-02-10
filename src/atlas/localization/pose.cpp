#include "atlas/localization/pose.hpp"
#include "atlas/util.hpp"
#include <cmath>
atlas::Pose::Pose(float x, float y, float angle) {
  this->x = x;
  this->y = y;
  this->angle = angle;
}
atlas::Pose atlas::Pose::operator+(const atlas::Pose &other) const {
  return atlas::Pose(this->x + other.x, this->y + other.y, this->angle);
}
atlas::Pose atlas::Pose::addWithAngle(const atlas::Pose &other) const {
  return atlas::Pose(this->x + other.x, this->y + other.y, wrapAngleRad(this->angle + other.angle));
}

atlas::Pose atlas::Pose::operator-(const atlas::Pose &other) const {
  return atlas::Pose(this->x - other.x, this->y - other.y, this->angle);
}

atlas::Pose atlas::Pose::minusWithAngle(const atlas::Pose &other) const {
  return atlas::Pose(this->x - other.x, this->y - other.y, wrapAngleRad(this->angle - other.angle));
}

float atlas::Pose::operator*(const atlas::Pose &other) const { return this->x * other.x + this->y * other.y; }

atlas::Pose atlas::Pose::operator*(const float &other) const {
  return atlas::Pose(this->x * other, this->y * other, this->angle);
}

atlas::Pose atlas::Pose::operator/(const float &other) const {
  return atlas::Pose(this->x / other, this->y / other, this->angle);
}
atlas::Pose atlas::Pose::lerp(atlas::Pose other, float t) const {
  return atlas::Pose(this->x + (other.x - this->x) * t, this->y + (other.y - this->y) * t, this->angle);
}
float atlas::Pose::distance(atlas::Pose other) const { return std::hypot(this->x - other.x, this->y - other.y); }