#pragma once
#include "atlas/config/drivetrain.hpp"
#include "atlas/config/opcurve.hpp"
#include "atlas/localization/odom/setup.hpp"
#include "atlas/localization/pose.hpp"
#include "pros/motors.h"

namespace atlas {
class Chassis {
public:
    Chassis(Drivetrain drivetrain,
                OdomSensors sensors, DriveCurve* throttleCurve,
                DriveCurve* steerCurve);
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
private:
        Drivetrain drivetrain;
        OdomSensors sensors;
        DriveCurve* throttleCurve;
        DriveCurve* steerCurve;
        float distTraveled = 0;
};
} // namespace atlas