#include "autonomous.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include <sys/_intsup.h>

extern lemlib::Chassis chassis;

extern pros::Motor intake;
extern pros::Motor indexer;
extern pros::Motor scoring;
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;
extern pros::adi::Pneumatics tongue;
extern pros::adi::Pneumatics alignerLeft;
extern pros::adi::Pneumatics alignerRight;
extern pros::Distance horizontaldistance;

double readWallIn() {
    double mm = horizontaldistance.get();
    if (mm <= 0) return -1;
    return mm / 25.4;
}

void driveToWallDistance(double targetIn, int timeoutMs = 800) {
    const int dt = 10;
    const double kP = 6.0;
    const double maxCmd = 45;
    const double deadband = 0.3;

    int t = 0;
    while (t < timeoutMs) {
        double d = readWallIn();
        if (d < 0) { pros::delay(dt); t += dt; continue; }

        double err = targetIn - d;
        if (std::fabs(err) < deadband) break;

        double cmd = kP * err;
        if (cmd >  maxCmd) cmd =  maxCmd;
        if (cmd < -maxCmd) cmd = -maxCmd;

        chassis.tank(cmd, cmd);
        pros::delay(dt);
        t += dt;
    }
    chassis.tank(0, 0);
}


void cdrift(float lV, float rV, int timeout, bool cst = true){ (cst == true) ? (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST)) : (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE)); leftMotors.move(lV); rightMotors.move(rV); pros::delay(timeout); leftMotors.brake(); rightMotors.brake(); } void cdrift(float lV, float rV){ leftMotors.move(lV); rightMotors.move(rV); }

void auton_red_left() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 45, 1200, {.maxSpeed = 70});
    chassis.turnToHeading(-87, 800); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(-15, 45, 800, {.maxSpeed = 100});
    pros::delay(1000);
    cdrift(-70, -70, 600, true);
    // chassis.moveToPoint(3, 45, 800, {.maxSpeed = 127});
    // pros::delay(1000);
    cdrift(120, 120, 1000, true);

    chassis.turnToHeading(-87, 400);
    chassis.moveToPoint(-21, 45, 1000, {.forwards = false, .maxSpeed = 80});
    tongue.set_value(false);
    intake.move(0);
    pros::delay(600);
    scoring.move(127);
    intake.move(-127);
    indexer.move(-127);
    pros::delay(1400);
    scoring.move(0);
    intake.move(0);
    indexer.move(0);


    chassis.turnToHeading(220, 800);
    chassis.moveToPoint(-31, 20, 1200, {.maxSpeed = 65});
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    intake.move(0);
    indexer.move(0);
    
    chassis.turnToHeading(400, 600);
    chassis.moveToPoint(41,5, 2000, {.forwards = false, .maxSpeed = 90});
    pros::delay(400);
    scoring.move(127);
    indexer.move(127);
    pros::delay(2000);
    scoring.move(0);
    indexer.move(0);
}

void auton_red_right() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 46, 1200, {.maxSpeed = 70});
    chassis.turnToHeading(87, 800); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(15, 45, 1200, {.maxSpeed = 100});
    pros::delay(1000);
    cdrift(-70, -70, 600, true);
    // chassis.moveToPoint(-3, 45, 1200, {.forwards=false, .maxSpeed = 127});
    // pros::delay(1000);
    cdrift(120, 120, 1000, true);

    chassis.turnToHeading(87, 400);
    chassis.moveToPoint(-21, 46, 1000, {.forwards = false, .maxSpeed = 80});
    tongue.set_value(false);
    intake.move(0);
    pros::delay(600);
    scoring.move(127);
    intake.move(-127);
    indexer.move(-127);
    pros::delay(1200);
    scoring.move(0);
    intake.move(0);
    indexer.move(0);


    chassis.turnToHeading(220, 800);
    chassis.moveToPoint(-31, 20, 1200, {.maxSpeed = 65});
    intake.move(-127);
    indexer.move(-127);
    pros::delay(1000);
    intake.move(0);
    indexer.move(0);

    chassis.moveToPoint(-40,4, 1000, {.maxSpeed = 55});
    pros::delay(400);
    scoring.move(-127);
    indexer.move(-127);
    intake.move(127);
    pros::delay(1200);
    scoring.move(0);
    indexer.move(0);
    intake.move(0);
}

void auton_blue_left() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 45, 1200, {.maxSpeed = 70});
    chassis.turnToHeading(87, 800); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(-15, 45, 800, {.maxSpeed = 100});
    pros::delay(1000);
    cdrift(-70, -70, 600, true);
    // chassis.moveToPoint(3, 45, 800, {.maxSpeed = 127});
    pros::delay(1000);
    cdrift(100, 100, 1000, true);

    chassis.turnToHeading(-86, 400);
    chassis.moveToPoint(23, 46, 1000, {.forwards = false, .maxSpeed = 80});
    tongue.set_value(false);
    intake.move(0);
    pros::delay(600);
    scoring.move(127);
    intake.move(-127);
    indexer.move(-127);
    pros::delay(1400);
    scoring.move(0);
    intake.move(0);
    indexer.move(0);


    chassis.turnToHeading(-225, 800);
    chassis.moveToPoint(18, 33, 1200, {.maxSpeed = 65});
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    intake.move(0);
    indexer.move(0);
    
    chassis.turnToHeading(-400, 600);
    chassis.moveToPoint(39,5, 2000, {.forwards = false, .maxSpeed = 90});
    pros::delay(400);
    scoring.move(127);
    indexer.move(127);
    pros::delay(2000);
    scoring.move(0);
    indexer.move(0);
}

void auton_blue_right() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 45, 1200, {.maxSpeed = 70});
    chassis.turnToHeading(86, 800); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(10, 45, 1200, {.maxSpeed = 100});
    pros::delay(1000);
    cdrift(-70, -70, 600, true);
    // chassis.moveToPoint(15, 46, 1200, {.maxSpeed = 127});
    // pros::delay(1000);
    cdrift(100, 100, 1000, true);

    chassis.turnToHeading(86, 400);
    chassis.moveToPoint(-21, 45, 1000, {.forwards = false, .maxSpeed = 80});
    tongue.set_value(false);
    intake.move(0);
    pros::delay(600);
    scoring.move(127);
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    scoring.move(0);
    intake.move(0);
    indexer.move(0);


    chassis.turnToHeading(226, 800);
    chassis.moveToPoint(-31, 18, 1200, {.maxSpeed = 65});
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    intake.move(0);
    indexer.move(0);

    chassis.moveToPoint(-41,7, 1000, {.maxSpeed = 55});
    pros::delay(490);
    scoring.move(-127);
    indexer.move(-127);
    intake.move(127);
    pros::delay(1200);
    scoring.move(0);
    indexer.move(0);
    intake.move(0);
}