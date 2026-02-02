#pragma once
#include "atlas/config/drivetrain.hpp"
#include "atlas/config/opcurve.hpp"
#include "atlas/localization/odom/setup.hpp"
#include "atlas/localization/pose.hpp"
#include "atlas/motion/controllers/PID.hpp"
#include "pros/motors.h"

namespace atlas {
enum class AngularDirection {
  CCW, // turn counter clockwise
  CW,  // turn clock wise
  AUTO // turn shortest direction
};
enum class DriveSide {
  LEFT, // lock left side
  RIGHT // lock right side
};
struct TurnToHeadingParams {
  /** the direction the robot should turn in. AUTO by default */
  AngularDirection direction = AngularDirection::AUTO;
  /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
  int maxSpeed = 127;
  // min speed to move to next task
  int chainSpeed = 0;
  // max error to move to next task
  float chainError = 0;
};
struct SwingToHeadingParams {
    /** the direction the robot should turn in. AUTO by default */
  AngularDirection direction = AngularDirection::AUTO;
  /** the maximum speed the robot can turn at. Value between 0-127. 127 by default */
  int maxSpeed = 127;
  // min speed to move to next task
  int chainSpeed = 0;
  // max error to move to next task
  float chainError = 0;
};
class Chassis {
public:
  Chassis(Drivetrain drivetrain, OdomSensors sensors, DriveCurve *throttleCurve, DriveCurve *steerCurve);
  // calibrate the drivetrain sensors
  void calibrate(bool calibrateIMU = true);
  // sets position of the chassis
  void setPose(float x, float y, float angle);
  // sets position to saved pose
  void setPose(Pose pose);
  // get position of chassis
  Pose getPose();
  // wait until chassis has travled a certain distanct along it's path
  void waitUntil(float dist);
  // wait until chassis has completed it's movement
  void waitUntilDone();
  // sets chassis brake mode
  void setBrakeMode(pros::motor_brake_mode_e mode);
  // turn to angle
  void turnToAngle(float angle, int timeout, TurnToHeadingParams params = {});
  // swing to ange
  void swingToAngle(float angle, DriveSide lockedSide, int timeout, SwingToHeadingParams params = {});

private:
  Drivetrain drivetrain;
  OdomSensors sensors;
  DriveCurve *throttleCurve;
  DriveCurve *steerCurve;
  float distTraveled = 0;
};
} // namespace atlas