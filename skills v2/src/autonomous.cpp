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



void cdrift(float lV, float rV, int timeout, bool cst = true){ (cst == true) ? (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST)) : (leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE), rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE)); leftMotors.move(lV); rightMotors.move(rV); pros::delay(timeout); leftMotors.brake(); rightMotors.brake(); } void cdrift(float lV, float rV){ leftMotors.move(lV); rightMotors.move(rV); }

void skillsPart1(){
    chassis.setPose(0, 0, 0);
    chassis.moveToPoint(0, 46, 1200, {.maxSpeed = 70});
    chassis.turnToHeading(86, 800); 
    intake.move(-127);
    indexer.move(-127);
    tongue.set_value(true);
    
    pros::delay(800);

    chassis.moveToPoint(15, 46, 1200, {.maxSpeed = 100});
    pros::delay(1000);
    cdrift(-70, -70, 500, true);
    // chassis.moveToPoint(15, 46, 1200, {.maxSpeed = 127});
    // pros::delay(1000);
    cdrift(100, 100, 1000, true);

    chassis.turnToHeading(86, 400);
    chassis.moveToPoint(-21, 47, 1000, {.forwards = false, .maxSpeed = 80});
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

    chassis.turnToHeading(313, 800);
    chassis.moveToPoint(-40, 56, 1000, {.maxSpeed = 70});
    pros::delay(800);
    chassis.moveToPoint(47, 56, 1000, {.maxSpeed = 90});
    chassis.turnToHeading(-90, 800);
}

void skillsPart2(){
    pros::delay(800);
    chassis.setPose(47,56, 0);

    chassis.moveToPoint(47, 48, 1000, {.maxSpeed = 70});
    chassis.turnToHeading(-90, 800);
    pros::delay(500);
    intake.move(-127);
    tongue.set_value(true);
    indexer.move(-127);

    chassis.moveToPoint(63, 48, 1000, {.maxSpeed = 100});
    pros::delay(1000);
    cdrift(-70, -70, 500, true);
    cdrift(100, 100, 1000, true);
    intake.move(0);
    indexer.move(0);
    tongue.set_value(false);

    chassis.moveToPoint(29.5, 48, 1000, {.forwards = false, .maxSpeed = 80});
    pros::delay(600);
    scoring.move(127);
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    scoring.move(0);
    intake.move(0);
    indexer.move(0);

    chassis.turnToHeading(118, 800);
    chassis.moveToPoint(61, 30.745, 1000, {.maxSpeed = 90});
    pros::delay(400);
    chassis.turnToHeading(188, 800);
}

void skillsPart3(){
    chassis.setPose(61,30.745, 180);
    chassis.moveToPoint(61, 17, 1000, {.maxSpeed = 60});
    pros::delay(400);
    chassis.moveToPoint(61, -23, 4000, {.maxSpeed = 127});
    chassis.turnToHeading(150, 800);
    chassis.moveToPoint(47, -46, 1000, {.maxSpeed = 90});
    pros::delay(400);
    chassis.turnToHeading(90, 800);
}

void skillsPart4(){
    pros::delay(800);
    chassis.setPose(47,-48, 0);
    pros::delay(500);
    intake.move(-127);
    tongue.set_value(true);
    indexer.move(-127);

    chassis.moveToPoint(-63, -48, 1000, {.maxSpeed = 100});
    pros::delay(1000);
    cdrift(-70, -70, 500, true);
    cdrift(100, 100, 1000, true);
    intake.move(0);
    indexer.move(0);
    tongue.set_value(false);

    chassis.moveToPoint(29.5, -48, 1000, {.forwards = false, .maxSpeed = 80});
    pros::delay(600);
    scoring.move(127);
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    scoring.move(0);
    intake.move(0);
    indexer.move(0);

    chassis.turnToHeading(133, 800);
    chassis.moveToPoint(40, -58, 1000, {.maxSpeed = 70});
    pros::delay(400);
    chassis.turnToHeading(90, 800);
    chassis.moveToPoint(-47, -58, 3000, {.forwards = false, .maxSpeed = 70});
    pros::delay(400);
    chassis.turnToHeading(0, 800);
}

void skillsPart5(){
    pros::delay(800);
    chassis.setPose(-47,-58, 0);

    chassis.turnToHeading(-90, 800);
    pros::delay(500);
    intake.move(-127);
    tongue.set_value(true);
    indexer.move(-127);

    chassis.moveToPoint(-63, -48, 1500, {.maxSpeed = 100});
    pros::delay(1000);
    cdrift(-70, -70, 500, true);
    cdrift(100, 100, 1000, true);
    intake.move(0);
    indexer.move(0);
    tongue.set_value(false);

    chassis.moveToPoint(-29.5, -48, 1000, {.forwards = false, .maxSpeed = 80});
    pros::delay(600);
    scoring.move(127);
    intake.move(-127);
    indexer.move(-127);
    pros::delay(2000);
    scoring.move(0);
    intake.move(0);
    indexer.move(0);

    pros::delay(400);
    chassis.turnToHeading(326, 800);
}

void skillsPart6(){
    chassis.setPose(-29.5,-48, 0);
    chassis.moveToPoint(-62, -48, 2500, {.forwards = false, .maxSpeed = 127});
}

void skillsRun(){
    skillsPart1();
    skillsPart2();
    skillsPart3();
    skillsPart4();
    skillsPart5();
    skillsPart6();
}