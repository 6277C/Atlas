#include "atlas/localization/odom/setup.hpp"
#include "atlas/util.hpp"

namespace atlas {


OdomSensors::OdomSensors(TrackingWheel *vertical1, TrackingWheel *vertical2, TrackingWheel *horizontal1,
                                TrackingWheel *horizontal2, pros::Imu *imu) {
  this->vertical1 = vertical1;
  this->vertical2 = vertical2;
  this->horizontal1 = horizontal1;
  this->horizontal2 = horizontal2;
  this->imu = imu;
}
atlas::TrackingWheel::TrackingWheel(pros::Rotation *encoder, float wheelDiameter, float distance, float gearRatio) {
  this->rotation = encoder;
  this->wheelDiameter = wheelDiameter;
  this->distance = distance;
  this->gearRatio = gearRatio;
}

atlas::TrackingWheel::TrackingWheel(pros::MotorGroup *motors, float wheelDiameter, float distance, float rpm) {
  this->motors = motors;
  this->motors->set_encoder_units_all(pros::E_MOTOR_ENCODER_ROTATIONS);
  this->wheelDiameter = wheelDiameter;
  this->distance = distance;
  this->rpm = rpm;
}

void atlas::TrackingWheel::reset() {
  if (this->rotation != nullptr)
    this->rotation->reset_position();
  if (this->motors != nullptr)
    this->motors->tare_position_all();
}
float atlas::TrackingWheel::getDist() {
  if (this->rotation != nullptr) {
    return (float(this->rotation->get_position()) * this->wheelDiameter * M_PI / 36000) / this->gearRatio;
  } else if (this->motors != nullptr) {
    // get distance traveled by each motor
    std::vector<pros::MotorGears> gearsets = this->motors->get_gearing_all();
    std::vector<double> positions = this->motors->get_position_all();
    std::vector<float> distances;
    for (int i = 0; i < this->motors->size(); i++) {
      float in;
      switch (gearsets[i]) {
      case pros::MotorGears::red:
        in = 100;
        break;
      case pros::MotorGears::green:
        in = 200;
        break;
      case pros::MotorGears::blue:
        in = 600;
        break;
      default:
        in = 200;
        break;
      }
      distances.push_back(positions[i] * (wheelDiameter * M_PI) * (rpm / in));
    }
    return atlas::avg(distances);
  } else {
    return 0;
  }
}

float atlas::TrackingWheel::getOffset() { return this->distance; }

bool atlas::TrackingWheel::getType() {
  if (this->motors != nullptr)
    return true;
  return false;
}
}