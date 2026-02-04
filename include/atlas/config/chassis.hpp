#pragma once
#include "atlas/localization/odom.hpp"
#include "atlas/localization/pose.hpp"
#include "pros/motor_group.hpp"

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
/**
* Drivetrain constuctor for the physical traits of the drivetrain
*/

class OdomSensors;
class OPCurve {
public:
  // virtual process of a input and output
  virtual float curve(float input) = 0;
};

// see https://www.desmos.com/calculator/umicbymbnl for an interactive graph
class ExpoOPCurve : public OPCurve {
public:
  ExpoOPCurve(float deadband, float minOutput, float curve);
  // curve an input
  float curve(float input);

private:
  const float deadband = 0;
  const float minOutput = 0;
  const float curveGain = 1;
};

class Drivetrain {
public:
  float wheelDiameter;
  float rpm;
  float trackWidth;
  float horizontalDrift;
  pros::MotorGroup *leftMotors;
  pros::MotorGroup *rightMotors;
  Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, float trackWidth, float wheelDiameter,
             float rpm, float horizontalDrift);
};
class Chassis {
public:
  // global constants
  float turnKP;
  float turnKI;
  float turnKD;
  float turnAntiWindup;

  void setTurnConstants(float turnKP, float turnKI, float turnKD, float turnAntiWindup);

  // chassis structure
  Chassis(Drivetrain& drivetrain, OdomSensors& sensors, OPCurve *throttleCurve, OPCurve *steerCurve);
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
  Drivetrain& drivetrain;
  OdomSensors& sensors;
  OPCurve *throttleCurve;
  OPCurve *steerCurve;
  float distTraveled = 0;
};
} // namespace atlas