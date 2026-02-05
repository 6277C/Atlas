#include "atlas/config/chassis.hpp"
#include "atlas/config/constants.hpp"
#include "atlas/localization/odom.hpp"
#include "atlas/motion/controllers/PID.hpp"
#include "atlas/motion/controllers/exitConditions.hpp"
#include "atlas/util.hpp"
#include "pros/rtos.hpp"
#include <cmath>
namespace atlas {

void Chassis::turnToAngle(float angle, int timeout, TurnToAngleParams params, bool async) {

  this-> requestMotionStart();

  if(!this->motionRunning) return;

  if(async){
    pros::Task task ([&](){turnToAngle(angle, timeout, params, false);});
    this-> endMotion();
    pros::delay(10);
    return;
  }

  float targetAngle = angle;
  float deltaAngle;
  float motorPower;
  float startAngle = getPose().angle;
  bool settling = false;
  bool initialized = false;
  float prevRawDeltaAngle = 0;
  float prevDeltaAngle = 0;

  angularPID.setConstants(relativeAngularParams);
  distTraveled = 0;
  Timer timer(timeout);
  angularPID.reset();
  relativeAngularExit.reset();

  while (!timer.isDone() && !relativeAngularExit.getExit()) {
    // update postion and dist traveled
    Pose pose = getPose();
    distTraveled = fabs(angleError(pose.angle, targetAngle));

    // Always compute raw delta
    const float rawDeltaAngle = angleError(targetAngle, pose.angle, false);

    // Detect sign change (skip first loop)
    if (initialized && sgn(rawDeltaAngle) != sgn(prevRawDeltaAngle)) {
      settling = true;
    }

    // Choose how to compute deltaAngle
    const float deltaAngle = settling ? angleError(targetAngle, pose.angle, false)
                                      : angleError(targetAngle, pose.angle, false, params.direction);

    // Store previous values
    prevRawDeltaAngle = rawDeltaAngle;
    prevDeltaAngle = deltaAngle;
    initialized = true;

    motorPower = angularPID.compute(deltaAngle);
    relativeAngularExit.compute(deltaAngle);

    if (motorPower > params.maxSpeed)
      motorPower = params.maxSpeed;
    else if (motorPower < -params.maxSpeed)
      motorPower = params.maxSpeed;

    drivetrain.leftMotors->move(motorPower);
    drivetrain.rightMotors->move(-motorPower);

    pros::delay(10);
  }
  drivetrain.leftMotors->move(0);
  drivetrain.rightMotors->move(0);

  distTraveled = -1;
}
} // namespace atlas