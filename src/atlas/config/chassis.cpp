#include "atlas/config/chassis.hpp"
#include "atlas/util.hpp"
#include "pros/motor_group.hpp"
#include <math.h>

namespace atlas {
/**
* Drivetrain Contructor
* Used in the chassis struct
*/
atlas::Drivetrain::Drivetrain(pros::MotorGroup *leftMotors, pros::MotorGroup *rightMotors, float trackWidth,
                              float wheelDiameter, float rpm, float horizontalDrift)
    : leftMotors(leftMotors), rightMotors(rightMotors), trackWidth(trackWidth), wheelDiameter(wheelDiameter), rpm(rpm),
      horizontalDrift(horizontalDrift) {}


ExpoOPCurve::ExpoOPCurve(float deadband, float minOutput, float curve)
    : deadband(deadband), minOutput(minOutput), curveGain(curve) {}

float ExpoOPCurve::curve(float input) {
  // return 0 if input is within deadzone
  if (fabs(input) <= deadband)
    return 0;
  // g is the output of g(x) as defined in the Desmos graph
  const float g = fabs(input) - deadband;
  // g127 is the output of g(127) as defined in the Desmos graph
  const float g127 = 127 - deadband;
  // i is the output of i(x) as defined in the Desmos graph
  const float i = pow(curveGain, g - 127) * g * sgn(input);
  // i127 is the output of i(127) as defined in the Desmos graph
  const float i127 = pow(curveGain, g127 - 127) * g127;
  return (127.0 - minOutput) / (127) * i * 127 / i127 + minOutput * sgn(input);
}
Chassis::Chassis(Drivetrain& drivetrain, OdomSensors& sensors, OPCurve *throttleCurve, OPCurve *steerCurve)
    : drivetrain(drivetrain), sensors(sensors), throttleCurve(throttleCurve), steerCurve(steerCurve) {}
void calibrateIMU(OdomSensors &sensors) {
  int attempt = 1;
  bool calibrated = false;
  // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
  while (attempt <= 5) {
    sensors.imu->reset();
    // wait until IMU is calibrated
    do
      pros::delay(10);
    while (sensors.imu->get_status() != pros::ImuStatus::error && sensors.imu->is_calibrating());
    // exit if imu has been calibrated
    if (!isnanf(sensors.imu->get_heading()) && !isinf(sensors.imu->get_heading())) {
      calibrated = true;
      break;
    }
    // indicate error
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, "---");
  }
  // check if calibration attempts were successful
  if (attempt > 5) {
    sensors.imu = nullptr;
  }
}

void Chassis::calibrate(bool calibrateImu) {
  // calibrate the IMU if it exists and the user doesn't specify otherwise
  if (sensors.imu != nullptr && calibrateImu)
    calibrateIMU(sensors);
  // initialize odom
  if (sensors.vertical1 == nullptr)
    sensors.vertical1 = new TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter, -(drivetrain.trackWidth / 2),
                                          drivetrain.rpm);
  if (sensors.vertical2 == nullptr)
    sensors.vertical2 =
        new TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter, drivetrain.trackWidth / 2, drivetrain.rpm);
  sensors.vertical1->reset();
  sensors.vertical2->reset();
  if (sensors.horizontal1 != nullptr)
    sensors.horizontal1->reset();
  if (sensors.horizontal2 != nullptr)
    sensors.horizontal2->reset();
  setSensors(sensors, drivetrain);
  init();
  // rumble to controller to indicate success
  pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void Chassis::setPose(float x, float y, float angle) { setPose(Pose(x, y, angle)); }

void Chassis::setPose(Pose pose) { setPose(pose); }

Pose Chassis::getPose() {
  Pose pose = atlas::getPose(true);
  return pose;
}

void Chassis::waitUntil(float dist) {
  // do while to give the thread time to start
  do
    pros::delay(10);
  while (distTraveled <= dist && distTraveled != -1);
}

void Chassis::waitUntilDone() {
  do
    pros::delay(10);
  while (distTraveled != -1);
}

void Chassis::requestMotionStart() {
    if (this->isInMotion()) this->motionQueued = true; // indicate a motion is queued
    else this->motionRunning = true; // indicate a motion is running

    // wait until this motion is at front of "queue"
    this->mutex.take(TIMEOUT_MAX);

    // this->motionRunning should be true
    // and this->motionQueued should be false
    // indicating this motion is running
}

void Chassis::endMotion() {
    // move the "queue" forward 1
    this->motionRunning = this->motionQueued;
    this->motionQueued = false;

    // permit queued motion to run
    this->mutex.give();
}

void Chassis::cancelMotion() {
    this->motionRunning = false;
    pros::delay(10); // give time for motion to stop
}

void Chassis::cancelAllMotions() {
    this->motionRunning = false;
    this->motionQueued = false;
    pros::delay(10); // give time for motion to stop
}

bool Chassis::isInMotion() const { return this->motionRunning; }

void Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
  drivetrain.leftMotors->set_brake_mode_all(mode);
  drivetrain.rightMotors->set_brake_mode_all(mode);
}
} // namespace atlas