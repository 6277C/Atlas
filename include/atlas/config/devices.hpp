#pragma once
#include "atlas/config/chassis.hpp"
#include "atlas/localization/amcl.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
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
extern pros::IMU imu;
extern pros::Distance leftDistance;
extern pros::Distance rightDistance;
extern pros::Gps gps;
extern atlas::Drivetrain drivetrain;
extern atlas::TrackingWheel horizontal;
extern atlas::TrackingWheel vertical;
extern atlas::OdomSensors sensors;
extern atlas::ExpoOPCurve throttleCurve;
extern atlas::Chassis chassis;
extern atlas::AMCL amcl;
extern atlas::AMCLDistanceSensor rightDistanceSensor;