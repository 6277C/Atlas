#pragma once
#include "atlas/config/drivetrain.hpp"
#include "atlas/localization/pose.hpp"
#include "atlas/localization/odom/setup.hpp"
namespace atlas {


void setSensors(atlas::OdomSensors sensors, atlas::Drivetrain drivetrain); // set dt sensors
Pose getPose(bool radians = false);
void setPose(Pose pose, bool radians = false);
Pose getSpeed(bool radians = false);
Pose getLocalSpeed(bool radians = false);
Pose estimatePose(float time, bool radians = false); //estimate pose after certain time
void update(); // update pose
void init();
} // namespace atlas