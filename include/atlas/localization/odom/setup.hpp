#pragma once
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rotation.hpp"
namespace atlas {
    namespace Omniwheel {
const float NEW_2 = 2.125;
const float NEW_275 = 2.75;
const float OLD_275 = 2.75;
const float NEW_275_HALF = 2.744;
const float OLD_275_HALF = 2.74;
const float NEW_325 = 3.25;
const float OLD_325 = 3.25;
const float NEW_325_HALF = 3.246;
const float OLD_325_HALF = 3.246;
const float NEW_4 = 4;
const float OLD_4 = 4.18;
const float NEW_4_HALF = 3.995;
const float OLD_4_HALF = 4.175;
} // namespace Omniwheel

class TrackingWheel {
public:
  /**
   * @brief Create a new tracking wheel
   * @param encoder The rotation sensor to use
   * @param wheelDiameter The diameter of the wheel
   * @param distance Dist from tracking center 
   * @param gearRatio gear ratio of tracking wheel, default is 1
   */
  TrackingWheel(pros::Rotation *encoder, float wheelDiameter, float distance, float gearRatio = 1);
  /**
   * @brief Create a new tracking wheel
   * @param motors The motor group to use
   * @param wheelDiameter The diameter of the wheel
   * @param distance Half the track width in inches
   * @param rpm Max rpm of your drivetrain
   */
  TrackingWheel(pros::MotorGroup *motors, float wheelDiameter, float distance, float rpm);
  
  void reset();      // resets tracking wheel distance to 0
  
  float getDist();   // gets distance tracking wheel traveled
  float getOffset(); // gets offset of tracking wheel in inches
  bool getType();

private:
  float wheelDiameter;
  float distance;
  float rpm;
  pros::Rotation *rotation = nullptr;
  pros::MotorGroup *motors = nullptr;
  float gearRatio = 1;
};

class OdomSensors{
  public:
  OdomSensors(TrackingWheel* vertical1, TrackingWheel* vertical2, TrackingWheel* horizontal1,
                    TrackingWheel* horizontal2, pros::Imu* imu);
        TrackingWheel* vertical1;
        TrackingWheel* vertical2;
        TrackingWheel* horizontal1;
        TrackingWheel* horizontal2;
        pros::Imu* imu;
};
}