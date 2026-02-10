#include "main.h"
#include "atlas/atlas.hpp" // IWYU pragma: keep
#include "atlas/config/chassis.hpp"
#include "atlas/config/devices.hpp"
#include "atlas/localization/amcl.hpp"
#include "atlas/localization/fieldMap.hpp"
#include "atlas/localization/pose.hpp"
#include "atlas/util.hpp"
#include "liblvgl/llemu.hpp"
#include "pros/llemu.hpp"
#include "pros/rtos.hpp"

using namespace pros;
FILE *logFile = nullptr;
void logHeader() {
  if (logFile) {
    fprintf(logFile, "time_ms,odom_x,odom_y,odom_a,amcl_x,amcl_y,amcl_a\n");
    fflush(logFile);
  }
}

void logData(uint32_t timeMs, const atlas::Pose &odomPose, const atlas::Pose &amclPose) {
  if (logFile) {
    fprintf(logFile, "%u,%.3f,%.3f,%.2f,%.3f,%.3f,%.2f\n", timeMs, odomPose.x, odomPose.y,
            atlas::radToDeg(odomPose.angle), amclPose.x, amclPose.y, atlas::radToDeg(amclPose.angle));
    fflush(logFile);
  }
}
/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
  atlas::Pose start(32, 6, 0);
  lcd::initialize();
  chassis.calibrate();
  // logFile = fopen("/usd/amcl_test.csv", "w");
  amcl.initialize(start, 20, .25);

  // pros::Task screen_task([&]() {
  //     while (true) {
          
  //         pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
  //         pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
  //         pros::lcd::print(2, "Angle: %f", chassis.getPose().angle); // heading
  //         pros::lcd::print(3, "Rotation Sensor: %i", horizontalEnc.get_position());
  //       pros::lcd::print(4, "Rotation Sensor: %i", verticalEnc.get_position());
  //         // delay to save resources
  //         pros::delay(100);
  //     }
  // });
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */
void autonomous() {}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */
void opcontrol() {
  uint32_t startTime = pros::millis();
  logHeader();

  while (true) {
    //float apsdkdsa = atlas::testMap.raycast(0, 0, atlas::degToRad(90));
    //lcd::print(0, "Raycast: %.2f", apsdkdsa);
    float dist = rightDistanceSensor.sensor->get_distance() * 0.03937f; // inches
    lcd::print(0, "Dist: %.2f", dist);
    amcl.updateDistance(dist, 90, 3, 6);
    float expected = atlas::testMap.raycast(38, 9, 0 + (atlas::degToRad(90)));
    // --- Record current estimate ---
    atlas::Pose amclPose = amcl.getEstimate(); // radians
    // uint32_t timeMs = pros::millis() - startTime;

    // lcd::print(0, "X: %.2f", curOdom.x);         // x
    // lcd::print(1, "Y: %.2f", curOdom.y);         // y
    // lcd::print(2, "Angle: %.2f", curOdom.angle); // angle
    //float predicted = atlas::testMap.raycast(amclPose.x + 6 * std::cos(amclPose.angle + M_PI/2), amclPose.y + 6 * std::sin(amclPose.angle + M_PI/2), atlas::radToDeg(amclPose.angle) + 90);
    //lcd::print(6, "Predicted dist: %.2f", predicted);
    lcd::print(3, "X: %.2f", amclPose.x);
    lcd::print(4, "Y: %.2f", amclPose.y);
    lcd::print(5, "Angle: %.2f", atlas::radToDeg(amclPose.angle));
    lcd::print(6, "Expected: %.2f", expected);
    pros::delay(50); // 50 Hz logging
  }

  fclose(logFile);
}