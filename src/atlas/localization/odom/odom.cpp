#include "atlas/localization/odom/setup.hpp"
#include "atlas/localization/odom/odom.hpp"
#include "atlas/config/drivetrain.hpp"
#include "atlas/localization/pose.hpp"
#include "atlas/util.hpp"
#include <math.h>
#include <numeric>



pros::Task *trackingTask = nullptr;

// global variables
atlas::OdomSensors odomSensors(nullptr, nullptr, nullptr, nullptr, nullptr); // the sensors to be used for odometry
atlas::Drivetrain drive(nullptr, nullptr, 0, 0, 0, 0);                       // the drivetrain to be used for odometry
atlas::Pose odomPose(0, 0, 0);                                               // the pose of the robot
atlas::Pose odomSpeed(0, 0, 0);                                              // the speed of the robot
atlas::Pose odomLocalSpeed(0, 0, 0);                                         // the local speed of the robot

float prevVertical = 0;
float prevVertical1 = 0;
float prevVertical2 = 0;
float prevHorizontal = 0;
float prevHorizontal1 = 0;
float prevHorizontal2 = 0;
float prevImu = 0;



void setSensors(atlas::OdomSensors& sensors, atlas::Drivetrain& drivetrain) {
  odomSensors = sensors;
  drive = drivetrain;
}
atlas::Pose atlas::getPose(bool radians) {
  if (radians)
    return odomPose;
  else
    return atlas::Pose(odomPose.x, odomPose.y, radToDeg(odomPose.angle));
}

void atlas::setPose(atlas::Pose pose, bool radians) {
  if (radians)
    odomPose = pose;
  else
    odomPose = atlas::Pose(pose.x, pose.y, degToRad(pose.angle));
}

atlas::Pose atlas::getSpeed(bool radians) {
  if (radians)
    return odomSpeed;
  else
    return atlas::Pose(odomSpeed.x, odomSpeed.y, radToDeg(odomSpeed.angle));
}

atlas::Pose atlas::getLocalSpeed(bool radians) {
  if (radians)
    return odomLocalSpeed;
  else
    return atlas::Pose(odomLocalSpeed.x, odomLocalSpeed.y, radToDeg(odomLocalSpeed.angle));
}

atlas::Pose atlas::estimatePose(float time, bool radians) {
  // get current position and speed
  Pose curPose = getPose(true);
  Pose localSpeed = getLocalSpeed(true);
  // calculate the change in local position
  Pose deltaLocalPose = localSpeed * time;

  // calculate the future pose
  float avgHeading = curPose.angle + deltaLocalPose.angle / 2;
  Pose futurePose = curPose;
  futurePose.x += deltaLocalPose.y * sin(avgHeading);
  futurePose.y += deltaLocalPose.y * cos(avgHeading);
  futurePose.x += deltaLocalPose.x * -cos(avgHeading);
  futurePose.y += deltaLocalPose.x * sin(avgHeading);
  if (!radians)
    futurePose.angle = radToDeg(futurePose.angle);

  return futurePose;
}

void atlas::update() {
  // TODO: add particle filter
  // get the current sensor values
  float vertical1Raw = 0;
  float vertical2Raw = 0;
  float horizontal1Raw = 0;
  float horizontal2Raw = 0;
  float imuRaw = 0;
  if (odomSensors.vertical1 != nullptr)
    vertical1Raw = odomSensors.vertical1->getDist();
  if (odomSensors.vertical2 != nullptr)
    vertical2Raw = odomSensors.vertical2->getDist();
  if (odomSensors.horizontal1 != nullptr)
    horizontal1Raw = odomSensors.horizontal1->getDist();
  if (odomSensors.horizontal2 != nullptr)
    horizontal2Raw = odomSensors.horizontal2->getDist();
  if (odomSensors.imu != nullptr)
    imuRaw = degToRad(odomSensors.imu->get_rotation());

  // calculate the change in sensor values
  float deltaVertical1 = vertical1Raw - prevVertical1;
  float deltaVertical2 = vertical2Raw - prevVertical2;
  float deltaHorizontal1 = horizontal1Raw - prevHorizontal1;
  float deltaHorizontal2 = horizontal2Raw - prevHorizontal2;
  float deltaImu = imuRaw - prevImu;

  // update the previous sensor values
  prevVertical1 = vertical1Raw;
  prevVertical2 = vertical2Raw;
  prevHorizontal1 = horizontal1Raw;
  prevHorizontal2 = horizontal2Raw;
  prevImu = imuRaw;

  // calculate the heading of the robot
  // Priority:
  // 1. Horizontal tracking wheels
  // 2. Vertical tracking wheels
  // 3. Inertial Sensor
  // 4. Drivetrain
  float heading = odomPose.angle;
  // calculate the heading using the horizontal tracking wheels
  if (odomSensors.horizontal1 != nullptr && odomSensors.horizontal2 != nullptr)
    heading -= (deltaHorizontal1 - deltaHorizontal2) /
               (odomSensors.horizontal1->getOffset() - odomSensors.horizontal2->getOffset());
  // else, if both vertical tracking wheels aren't substituted by the drivetrain, use the vertical tracking wheels
  else if (!odomSensors.vertical1->getType() && !odomSensors.vertical2->getType())
    heading -=
        (deltaVertical1 - deltaVertical2) / (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
  // else, if the inertial sensor exists, use it
  else if (odomSensors.imu != nullptr)
    heading += deltaImu;
  // else, use the the substituted tracking wheels
  else
    heading -=
        (deltaVertical1 - deltaVertical2) / (odomSensors.vertical1->getOffset() - odomSensors.vertical2->getOffset());
  float deltaHeading = heading - odomPose.angle;
  float avgHeading = odomPose.angle + deltaHeading / 2;

  // choose tracking wheels to use
  // Prioritize non-powered tracking wheels
  atlas::TrackingWheel *verticalWheel = nullptr;
  atlas::TrackingWheel *horizontalWheel = nullptr;
  if (!odomSensors.vertical1->getType())
    verticalWheel = odomSensors.vertical1;
  else if (!odomSensors.vertical2->getType())
    verticalWheel = odomSensors.vertical2;
  else
    verticalWheel = odomSensors.vertical1;
  if (odomSensors.horizontal1 != nullptr)
    horizontalWheel = odomSensors.horizontal1;
  else if (odomSensors.horizontal2 != nullptr)
    horizontalWheel = odomSensors.horizontal2;
  float rawVertical = 0;
  float rawHorizontal = 0;
  if (verticalWheel != nullptr)
    rawVertical = verticalWheel->getDist();
  if (horizontalWheel != nullptr)
    rawHorizontal = horizontalWheel->getDist();
  float horizontalOffset = 0;
  float verticalOffset = 0;
  if (verticalWheel != nullptr)
    verticalOffset = verticalWheel->getOffset();
  if (horizontalWheel != nullptr)
    horizontalOffset = horizontalWheel->getOffset();

  // calculate change in x and y
  float deltaX = 0;
  float deltaY = 0;
  if (verticalWheel != nullptr)
    deltaY = rawVertical - prevVertical;
  if (horizontalWheel != nullptr)
    deltaX = rawHorizontal - prevHorizontal;
  prevVertical = rawVertical;
  prevHorizontal = rawHorizontal;

  // calculate local x and y
  float localX = 0;
  float localY = 0;
  if (deltaHeading == 0) { // prevent divide by 0
    localX = deltaX;
    localY = deltaY;
  } else {
    localX = 2 * sin(deltaHeading / 2) * (deltaX / deltaHeading + horizontalOffset);
    localY = 2 * sin(deltaHeading / 2) * (deltaY / deltaHeading + verticalOffset);
  }

  // save previous pose
  atlas::Pose prevPose = odomPose;

  // calculate global x and y
  odomPose.x += localY * sin(avgHeading);
  odomPose.y += localY * cos(avgHeading);
  odomPose.x += localX * -cos(avgHeading);
  odomPose.y += localX * sin(avgHeading);
  odomPose.angle = heading;

  // calculate speed
  odomSpeed.x = fma((odomPose.x - prevPose.x) / 0.01, odomSpeed.x, 0.95);
  odomSpeed.y = fma((odomPose.y - prevPose.y) / 0.01, odomSpeed.y, 0.95);
  odomSpeed.angle = fma((odomPose.angle - prevPose.angle) / 0.01, odomSpeed.angle, 0.95);

  // calculate local speed
  odomLocalSpeed.x = fma(localX / 0.01, odomLocalSpeed.x, 0.95);
  odomLocalSpeed.y = fma(localY / 0.01, odomLocalSpeed.y, 0.95);
  odomLocalSpeed.angle = fma(deltaHeading / 0.01, odomLocalSpeed.angle, 0.95);
}

void atlas::init() {
  if (trackingTask == nullptr) {
    trackingTask = new pros::Task{[=] {
      while (true) {
        update();
        pros::delay(10);
      }
    }};
  }
}