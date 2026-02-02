#include "atlas/config/chassis.hpp"
#include "atlas/config/drivetrain.hpp"
#include "atlas/config/opcurve.hpp"
#include "atlas/localization/odom/odom.hpp"
#include <math.h>

namespace atlas {
Chassis::Chassis(Drivetrain drivetrain, OdomSensors sensors, DriveCurve *throttleCurve, DriveCurve *steerCurve)
    : drivetrain(drivetrain), sensors(sensors), throttleCurve(throttleCurve), steerCurve(steerCurve) {}
void calibrateIMU(OdomSensors& sensors) {
    int attempt = 1;
    bool calibrated = false;
    // calibrate inertial, and if calibration fails, then repeat 5 times or until successful
    while (attempt <= 5) {
        sensors.imu->reset();
        // wait until IMU is calibrated
        do pros::delay(10);
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

void Chassis::calibrate(bool calibrateImu){
     // calibrate the IMU if it exists and the user doesn't specify otherwise
    if (sensors.imu != nullptr && calibrateImu) calibrateIMU(sensors);
    // initialize odom
    if (sensors.vertical1 == nullptr)
        sensors.vertical1 = new atlas::TrackingWheel(drivetrain.leftMotors, drivetrain.wheelDiameter,
                                                      -(drivetrain.trackWidth / 2), drivetrain.rpm);
    if (sensors.vertical2 == nullptr)
        sensors.vertical2 = new atlas::TrackingWheel(drivetrain.rightMotors, drivetrain.wheelDiameter,
                                                      drivetrain.trackWidth / 2, drivetrain.rpm);
    sensors.vertical1->reset();
    sensors.vertical2->reset();
    if (sensors.horizontal1 != nullptr) sensors.horizontal1->reset();
    if (sensors.horizontal2 != nullptr) sensors.horizontal2->reset();
    atlas::setSensors(sensors, drivetrain);
    init();
    // rumble to controller to indicate success
    pros::c::controller_rumble(pros::E_CONTROLLER_MASTER, ".");
}

void atlas::Chassis::setPose(float x, float y, float angle) {
    atlas::setPose(atlas::Pose(x, y, angle));
}

void atlas::Chassis::setPose(Pose pose) { atlas::setPose(pose); }

atlas::Pose atlas::Chassis::getPose() {
    Pose pose = atlas::getPose(true);
    return pose;
}

void atlas::Chassis::waitUntil(float dist) {
    // do while to give the thread time to start
    do pros::delay(10);
    while (distTraveled <= dist && distTraveled != -1);
}

void atlas::Chassis::waitUntilDone() {
    do pros::delay(10);
    while (distTraveled != -1);
}

void atlas::Chassis::setBrakeMode(pros::motor_brake_mode_e mode) {
    drivetrain.leftMotors->set_brake_mode_all(mode);
    drivetrain.rightMotors->set_brake_mode_all(mode);
}
} // namespace atlas