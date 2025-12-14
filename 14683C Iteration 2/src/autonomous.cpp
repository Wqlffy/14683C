#include "autonomous.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"
#include <sys/_intsup.h>

ASSET(red_left_txt);
ASSET(red_right_txt);
ASSET(blue_left_txt);
ASSET(blue_right_txt);

extern lemlib::Chassis chassis;

extern pros::Motor intake;
extern pros::Motor indexer;
extern pros::Motor scoring;
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

static void wait_ms(int ms) {
    pros::delay(ms);
}

void auton_red_left() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 51, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(-92, 750); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(-12, 51, 1500, {.maxSpeed = 127});
    pros::delay(1500);
    chassis.moveToPoint(-1, 51, 1000, {.forwards = false});
    pros::delay(500);
    chassis.moveToPoint(-12, 51, 1500, {.maxSpeed = 127});
    pros::delay(1500);

    chassis.turnToHeading(-90.5, 500);
    chassis.moveToPoint(0, 51, 2000, {.forwards = false, .maxSpeed = 65});
    pros::delay(500);
    tongue.set_value(false);
    intake.move(0);

    chassis.moveToPoint(22.5, 52, 1500, {.forwards = false, .maxSpeed = 65});
    scoring.move(127);
    indexer.move(-127);
    pros::delay(4000);
    scoring.move(0);
    indexer.move(0);


    chassis.turnToHeading(-220, 750);
    // chassis.moveToPoint(13.5, 38, 1500, {.maxSpeed = 65});
    chassis.moveToPoint(24, 22, 1500, {.maxSpeed = 80});
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    intake.move(0);
    indexer.move(0);
    
    chassis.turnToHeading(-398, 750);
    chassis.moveToPoint(41,5, 2000, {.forwards = false, .maxSpeed = 55});
    pros::delay(500);
    scoring.move(127);
    indexer.move(127);
    pros::delay(2000);
    scoring.move(0);
    indexer.move(0);
}

void auton_red_right() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 51, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(90, 750); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(12, 50, 1500, {.maxSpeed = 127});
    pros::delay(1500);
    chassis.moveToPoint(1, 50, 1000, {.forwards = false});
    pros::delay(500);
    chassis.moveToPoint(12, 50, 1500, {.maxSpeed = 127});
    pros::delay(1500);

    chassis.turnToHeading(90.5, 500);
    chassis.moveToPoint(0, 51, 2000, {.forwards = false, .maxSpeed = 65});
    pros::delay(500);
    tongue.set_value(false);
    intake.move(0);

    chassis.moveToPoint(-22.5, 52, 1500, {.forwards = false, .maxSpeed = 65});
    scoring.move(127);
    indexer.move(-127);
    pros::delay(4000);
    scoring.move(0);
    indexer.move(0);


    chassis.turnToHeading(227, 750);
    chassis.moveToPoint(-17, 33.6, 1500, {.maxSpeed = 65});
    chassis.moveToPoint(-26, 22, 1500, {.maxSpeed = 80});
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    intake.move(0);
    indexer.move(0);
    
    chassis.moveToPoint(-36,9, 2000, {.maxSpeed = 55});
    pros::delay(500);
    scoring.move(-127);
    indexer.move(-127);
    intake.move(127);
    pros::delay(2000);
    scoring.move(0);
    indexer.move(0);
    intake.move(0);
}


void auton_blue_left() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 51, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(-92, 750); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(-12, 51, 1500, {.maxSpeed = 127});
    pros::delay(1500);
    chassis.moveToPoint(-1, 51, 1000, {.forwards = false});
    pros::delay(500);
    chassis.moveToPoint(-12, 51, 1500, {.maxSpeed = 127});
    pros::delay(1500);

    chassis.turnToHeading(-90.5, 500);
    chassis.moveToPoint(0, 51, 2000, {.forwards = false, .maxSpeed = 65});
    pros::delay(500);
    tongue.set_value(false);
    intake.move(0);

    chassis.moveToPoint(22.5, 52, 1500, {.forwards = false, .maxSpeed = 65});
    scoring.move(127);
    indexer.move(-127);
    pros::delay(4000);
    scoring.move(0);
    indexer.move(0);


    chassis.turnToHeading(-220, 750);
    // chassis.moveToPoint(13.5, 38, 1500, {.maxSpeed = 65});
    chassis.moveToPoint(24, 22, 1500, {.maxSpeed = 80});
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    intake.move(0);
    indexer.move(0);
    
    chassis.turnToHeading(-398, 750);
    chassis.moveToPoint(41,5, 2000, {.forwards = false, .maxSpeed = 55});
    pros::delay(500);
    scoring.move(127);
    indexer.move(127);
    pros::delay(2000);
    scoring.move(0);
    indexer.move(0);
}

void auton_blue_right() {
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 51, 1500, {.maxSpeed = 80});
    chassis.turnToHeading(90, 750); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(12, 50, 1500, {.maxSpeed = 127});
    pros::delay(1500);
    chassis.moveToPoint(1, 50, 1000, {.forwards = false});
    pros::delay(500);
    chassis.moveToPoint(12, 50, 1500, {.maxSpeed = 127});
    pros::delay(1500);

    chassis.turnToHeading(90.5, 500);
    chassis.moveToPoint(0, 51, 2000, {.forwards = false, .maxSpeed = 65});
    pros::delay(500);
    tongue.set_value(false);
    intake.move(0);

    chassis.moveToPoint(-22.5, 52, 1500, {.forwards = false, .maxSpeed = 65});
    scoring.move(127);
    indexer.move(-127);
    pros::delay(4000);
    scoring.move(0);
    indexer.move(0);


    chassis.turnToHeading(227, 750);
    chassis.moveToPoint(-17, 33.6, 1500, {.maxSpeed = 65});
    chassis.moveToPoint(-26, 22, 1500, {.maxSpeed = 80});
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    intake.move(0);
    indexer.move(0);
    
    chassis.moveToPoint(-36,9, 2000, {.maxSpeed = 55});
    pros::delay(500);
    scoring.move(-127);
    indexer.move(-127);
    intake.move(127);
    pros::delay(2000);
    scoring.move(0);
    indexer.move(0);
    intake.move(0);
}


