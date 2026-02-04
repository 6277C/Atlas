#pragma once
#include "atlas/config/chassis.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"

namespace atlas {
 class Drivetrain;
 class OdomSensors;
 class Chassis;
 class ExpoOPCurve;
}
extern pros::Controller master;
extern pros::MotorGroup left_motors;
extern pros::MotorGroup right_motors;
extern pros::Rotation verticalEnc;
extern pros::Rotation horizontalEnc;
extern atlas::Drivetrain drivetrain;
extern atlas::TrackingWheel horizontal;
extern atlas::TrackingWheel vertical;
extern atlas::OdomSensors sensors;
extern atlas::ExpoOPCurve throttleCurve;
extern atlas::Chassis chassis;