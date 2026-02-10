#include "atlas/localization/amcl.hpp"
#include "atlas/localization/fieldMap.hpp"
#include "atlas/localization/odom.hpp"
#include "pros/abstract_motor.hpp"
#include "pros/distance.hpp"
#include "pros/gps.hpp"
#include "pros/imu.hpp"
#include "pros/misc.h"
#include "pros/misc.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
pros::Controller master(pros::E_CONTROLLER_MASTER);
pros::MotorGroup leftMotors({-11, -12, -13},
                            pros::MotorGearset::blue); // left motor group - ports 3 (reversed), 4, 5 (reversed)
pros::MotorGroup rightMotors({14, 15, 16}, pros::MotorGearset::blue); // right motor group - ports 6, 7, 9 (reversed)

pros::Motor intake(19, pros::MotorGearset::blue);
pros::Motor indexer(17, pros::MotorGearset::blue);

pros::Distance leftDistance(18);
pros::Distance rightDistance(1);
pros::Imu imu(20);
pros::Gps gps(2);
pros::Rotation horizontalEnc(10);
// vertical tracking wheel encoder. Rotation sensor, port 11, reversed
pros::Rotation verticalEnc(9);
atlas::AMCLDistanceSensor rightDistanceSensor(&rightDistance, 3.0, 6.0, 90); // distance sensor on the right side of the robot, facing outwards
// horizontal tracking wheel. 2.75" diameter, 5.75" offset, back of the robot (negative)
atlas::TrackingWheel horizontal(&horizontalEnc, atlas::Omniwheel::NEW_2, 0.5);
// vertical tracking wheel. 2.75" diameter, 2.5" offset, left of the robot (negative)
atlas::TrackingWheel vertical(&verticalEnc, atlas::Omniwheel::NEW_2, 0.0);

atlas::Drivetrain drivetrain(&leftMotors, // left motor group
                              &rightMotors, // right motor group
                              10, // 10 inch track width
                              atlas::Omniwheel::NEW_325, // using new 4" omnis
                              450, // drivetrain rpm is 360
                              8 // horizontal drift is 2. If we had traction wheels, it would have been 8
);

// sensors for odometry
atlas::OdomSensors sensors(&vertical, // vertical tracking wheel
                            nullptr, // vertical tracking wheel 2, set to nullptr as we don't have a second one
                            &horizontal, // horizontal tracking wheel
                            nullptr, // horizontal tracking wheel 2, set to nullptr as we don't have a second one
                            &imu // inertial sensor
);

atlas::ExpoOPCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

atlas::Chassis chassis(drivetrain, sensors, &throttleCurve, nullptr);

atlas::AMCL amcl{amclConfig, &atlas::testMap};
