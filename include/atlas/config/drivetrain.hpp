#pragma once

#include "pros/motor_group.hpp"
namespace atlas {
class Drivetrain {
public:
  float wheelDiameter;
  float rpm;
  float trackWidth;
  float horizontalDrift;
  Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, float trackWidth, float wheelDiameter,
             float rpm, float horizontalDrift);
  pros::MotorGroup *leftMotors;
  pros::MotorGroup *rightMotors;
};
} // namespace atlas