#include "autons.hpp"

#include "lemlib/api.hpp"
#include "main.h"
#include "auton_recovery.hpp"
#include "robot_config.hpp"
#include "auton_recovery.hpp"
#include <sys/_intsup.h>

extern lemlib::Chassis chassis;

ASSET(seg1_43_txt);
ASSET(seg3_43_txt);
ASSET(seg1L_txt);
ASSET(seg2L_txt);
ASSET(seg3L_txt);
ASSET(seg7L_txt);
ASSET(seg1R_txt);
ASSET(seg2R_txt);
ASSET(seg3R_txt);
ASSET(seg7R_txt);

void auton_43_blue_left() {
    chassis.setPose(-48.000000, 6.000000, 0.000000);

    chassis.moveToPoint(-48.0, 48.0, 1377);
    pros::delay(400);
    chassis.turnToHeading(0.0, 427);
    chassis.turnToHeading(270.0, 767);
    AutonRecovery::runSegmentWithRecovery(
        [&]() {
            chassis.follow(seg1_43_txt, 829, 9.0);
            intakeMotor.move(127);
            chassis.waitUntil(6.96);
            matchloader.extend();
            chassis.waitUntilDone();
        },
        []() { return 0.65; }, 300, 270, true, 2, 20);
    pros::delay(2000);
    chassis.moveToPoint(-32.64, 47.76, 1158, {.forwards = false});
    chassis.waitUntilDone();
    AutonRecovery::snapRightToWall(595.376, 270);
    matchloader.retract();
    pros::delay(150);
    intakeMotor.move(127);
    outtakeMotor.move(127);
    pros::delay(2000);
    outtakeMotor.move(0);
    AutonRecovery::runSegmentWithRecovery(
        [&]() {
            chassis.follow(seg3_43_txt, 2171, 9.0);
            intakeMotor.move(127);
            matchloader.extend();
            chassis.waitUntilDone();
        },
        []() { return 0.55; }, 300, 270, true, 2, 20);
    pros::delay(1000);
    chassis.turnToHeading(317.663001, 987);
    chassis.moveToPoint(-13.92, 12.96, 1157, {.forwards = false});
    chassis.waitUntil(3.067172);
    matchloader.retract();
    chassis.waitUntil(14.578923);
    midgoal.extend();
    outtakeMotor.move(127);
    chassis.waitUntilDone();
    intakeMotor.move(0);
    outtakeMotor.move(0);
}

void auton_43_red_left() {
    chassis.setPose(-48.000000, 6.000000, 0.000000);

    chassis.moveToPoint(-48.0, 48.0, 1377);
    pros::delay(400);
    chassis.turnToHeading(0.0, 427);
    chassis.turnToHeading(270.0, 767);
    AutonRecovery::runSegmentWithRecovery(
        [&]() {
            chassis.follow(seg1_43_txt, 829, 9.0);
            chassis.waitUntil(6.96);
            intakeMotor.move(127);
            matchloader.extend();
            chassis.waitUntilDone();
        },
        []() { return 0.65; }, 300, 270, true, 2, 20);
    pros::delay(2000);
    chassis.moveToPoint(-32.64, 47.76, 1158, {.forwards = false});
    chassis.waitUntilDone();
    AutonRecovery::snapRightToWall(595.376, 270.0);
    matchloader.retract();
    pros::delay(150);
    intakeMotor.move(127);
    outtakeMotor.move(127);
    pros::delay(2000);
    outtakeMotor.move(0);
    AutonRecovery::runSegmentWithRecovery(
        [&]() {
            chassis.follow(seg3_43_txt, 2171, 9.0);
            intakeMotor.move(127);
            matchloader.extend();
            chassis.waitUntilDone();
        },
        []() { return 0.55; }, 300, 270, true, 2, 20);
    pros::delay(1000);
    chassis.turnToHeading(317.663001, 987);
    chassis.moveToPoint(-13.92, 12.96, 1157, {.forwards = false});
    chassis.waitUntil(3.067172);
    matchloader.retract();
    chassis.waitUntil(14.578923);
    midgoal.extend();
    outtakeMotor.move(127);
    chassis.waitUntilDone();
    intakeMotor.move(0);
    outtakeMotor.move(0);
}

void auton_9_long_blue_left() {
    chassis.setPose(-62.640000, 17.040000, 90.000000);

    chassis.moveToPoint(-48.0, 17.04, 850);
    pros::delay(50);
    chassis.turnToHeading(74.784562, 509);
    chassis.follow(seg1L_txt, 1265, 9.0);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.follow(seg2L_txt, 1072, 9.0);
    chassis.waitUntil(4.435911);
    matchloader.retract();
    chassis.waitUntil(19.765012);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.follow(seg3L_txt, 3880, 9.0, false);
    chassis.waitUntil(9.791251);
    intakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntil(98.237126);
    intakeMotor.move(127);
    outtakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(2000);
    outtakeMotor.move(0);
    chassis.moveToPoint(-61.2, 48.24, 1163);
    chassis.waitUntil(20.64);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.moveToPoint(-29.76, 48.0, 1369, {.forwards = false});
    AutonRecovery::snapRightToWall(595.376, 270);
    outtakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(1500);
    outtakeMotor.move(0);
    chassis.moveToPoint(-43.68, 48.0, 835);
    pros::delay(50);
    chassis.turnToHeading(342.552368, 708);
    chassis.follow(seg7L_txt, 1440, 9.0, false);
    chassis.waitUntil(28.472135);
    wing.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(270.0, 461);
    chassis.moveToPoint(-11.52, 36.24, 1301, {.forwards = false, .maxSpeed= 50});
}

void auton_9_long_blue_right() {
    chassis.setPose(-62.640000, -17.040000, 0.000000);

    chassis.turnToHeading(90.0, 767);
    chassis.moveToPoint(-48.0, -17.04, 850);
    pros::delay(50);
    chassis.turnToHeading(105.215438, 509);
    chassis.follow(seg1R_txt, 1265, 9.0);
    chassis.waitUntil(21.49791);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.follow(seg2R_txt, 1072, 9.0);
    chassis.waitUntil(4.435911);
    matchloader.retract();
    chassis.waitUntil(19.765012);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.follow(seg3R_txt, 3880, 9.0, false);
    chassis.waitUntil(9.791251);
    intakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntil(98.237126);
    intakeMotor.move(127);
    outtakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(2000);
    outtakeMotor.move(0);
    chassis.moveToPoint(-61.2, -48.24, 1163);
    chassis.waitUntil(20.64);
    outtakeMotor.move(0);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.moveToPoint(-29.76, -48.0, 1369, {.forwards = false});
    AutonRecovery::snapRightToWall(595.376, 270);
    outtakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(1500);
    outtakeMotor.move(0);
    chassis.moveToPoint(-44.16, -48.0, 845);
    pros::delay(50);
    chassis.turnToHeading(333.960685, 677);
    chassis.follow(seg7R_txt, 1403, 9.0, false);
    chassis.waitUntil(27.146024);
    wing.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(270.0, 462);
    chassis.moveToPoint(-6.0, -60.0, 1661, {.forwards = false, .maxSpeed = 50});
}

void auton_9_long_red_left() {
    chassis.setPose(-62.640000, 17.040000, 90.000000);

    chassis.moveToPoint(-48.0, 17.04, 850);
    pros::delay(50);
    chassis.turnToHeading(74.784562, 509);
    chassis.follow(seg1L_txt, 1265, 9.0);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.follow(seg2L_txt, 1072, 9.0);
    chassis.waitUntil(4.435911);
    matchloader.retract();
    chassis.waitUntil(19.765012);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.follow(seg3L_txt, 3880, 9.0, false);
    chassis.waitUntil(9.791251);
    intakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntil(98.237126);
    intakeMotor.move(127);
    outtakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(2000);
    outtakeMotor.move(0);
    chassis.moveToPoint(-61.2, 48.24, 1163);
    chassis.waitUntil(20.64);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.moveToPoint(-29.76, 48.0, 1369, {.forwards = false});
    AutonRecovery::snapRightToWall(595.376, 270);
    outtakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(1500);
    outtakeMotor.move(0);
    chassis.moveToPoint(-43.68, 48.0, 835);
    pros::delay(50);
    chassis.turnToHeading(342.552368, 708);
    chassis.follow(seg7L_txt, 1440, 9.0, false);
    chassis.waitUntil(28.472135);
    wing.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(270.0, 461);
    chassis.moveToPoint(-11.52, 36.24, 1301, {.forwards = false, .maxSpeed= 50});
}

void auton_9_long_red_right() {
    chassis.setPose(-62.640000, -17.040000, 0.000000);

    chassis.turnToHeading(90.0, 767);
    chassis.moveToPoint(-48.0, -17.04, 850);
    pros::delay(50);
    chassis.turnToHeading(105.215438, 509);
    chassis.follow(seg1R_txt, 1265, 9.0);
    chassis.waitUntil(21.49791);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.follow(seg2R_txt, 1072, 9.0);
    chassis.waitUntil(4.435911);
    matchloader.retract();
    chassis.waitUntil(19.765012);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(1000);
    chassis.follow(seg3R_txt, 3880, 9.0, false);
    chassis.waitUntil(9.791251);
    intakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntil(98.237126);
    intakeMotor.move(127);
    outtakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(2000);
    outtakeMotor.move(0);
    chassis.moveToPoint(-61.2, -48.24, 1163);
    chassis.waitUntil(20.64);
    outtakeMotor.move(0);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(2000);
    chassis.moveToPoint(-29.76, -48.0, 1369, {.forwards = false});
    AutonRecovery::snapRightToWall(595.376, 270);
    outtakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(1500);
    outtakeMotor.move(0);
    chassis.moveToPoint(-44.16, -48.0, 845);
    pros::delay(50);
    chassis.turnToHeading(333.960685, 677);
    chassis.follow(seg7R_txt, 1403, 9.0, false);
    chassis.waitUntil(27.146024);
    wing.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(270.0, 462);
    chassis.moveToPoint(-6.0, -60.0, 1661, {.forwards = false, .maxSpeed = 50});
}

void auton_awp_blue_right() {
    chassis.setPose(-48.000000, -6.000000, 180.000000);

    chassis.moveToPoint(-48.0, -48.0, 1377);
    pros::delay(50);
    chassis.turnToHeading(270.0, 767);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.moveToPoint(-62.88, -48.0, 856);
    pros::delay(1800);
    chassis.moveToPoint(-31.92, -48.24, 1199, {.forwards = false});
    AutonRecovery::snapRightToWall(595.376, 270);
    outtakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(2000);
    outtakeMotor.move(0);
    chassis.moveToPoint(-41.28, -48.24, 777);
    pros::delay(100);
    chassis.turnToHeading(35.48398, 841);
    chassis.moveToPoint(-24.0, -24.0, 1503);
    chassis.waitUntil(24.830799);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(0.0, 558);
    chassis.moveToPoint(-24.0, 24.0, 1935);
    chassis.waitUntil(5.76);
    matchloader.retract();
    chassis.waitUntil(44.64);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(315.643746, 598);
    chassis.moveToPoint(-13.44, 13.2, 860, {.forwards = false});
    chassis.waitUntil(4.919246);
    matchloader.retract();
    chassis.waitUntil(15.104754);
    outtakeMotor.move(127);
    midgoal.extend();
    chassis.waitUntilDone();
    pros::delay(1000);
    outtakeMotor.move(0);
    midgoal.retract();
    chassis.moveToPoint(-48.0, 48.0, 1522);
    chassis.waitUntil(5.430548);
    midgoal.retract();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(270.0, 602);
    chassis.moveToPoint(-63.12, 48.0, 860);
    chassis.waitUntil(7.29931);
    intakeMotor.move(127);
    matchloader.toggle();
    chassis.waitUntilDone();
    pros::delay(1500);
    chassis.moveToPoint(-31.92, 47.76, 1204, {.forwards = false});
    AutonRecovery::snapRightToWall(595.376, 270);
    outtakeMotor.move(127);
}

void auton_awp_red_right() {
    chassis.setPose(-48.000000, -6.000000, 180.000000);

    chassis.moveToPoint(-48.0, -48.0, 1377);
    pros::delay(50);
    chassis.turnToHeading(270.0, 767);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.moveToPoint(-62.88, -48.0, 856);
    pros::delay(1800);
    chassis.moveToPoint(-31.92, -48.24, 1199, {.forwards = false});
    AutonRecovery::snapRightToWall(595.376, 270);
    outtakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(2000);
    outtakeMotor.move(0);
    chassis.moveToPoint(-41.28, -48.24, 777);
    pros::delay(100);
    chassis.turnToHeading(35.48398, 841);
    chassis.moveToPoint(-24.0, -24.0, 1503);
    chassis.waitUntil(24.830799);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(0.0, 558);
    chassis.moveToPoint(-24.0, 24.0, 1935);
    chassis.waitUntil(5.76);
    matchloader.retract();
    chassis.waitUntil(44.64);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(315.643746, 598);
    chassis.moveToPoint(-13.44, 13.2, 860, {.forwards = false});
    chassis.waitUntil(4.919246);
    matchloader.retract();
    chassis.waitUntil(15.104754);
    outtakeMotor.move(127);
    midgoal.extend();
    chassis.waitUntilDone();
    pros::delay(1000);
    outtakeMotor.move(0);
    midgoal.retract();
    chassis.moveToPoint(-48.0, 48.0, 1522);
    chassis.waitUntil(5.430548);
    midgoal.retract();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(270.0, 602);
    chassis.moveToPoint(-63.12, 48.0, 860);
    chassis.waitUntil(7.29931);
    intakeMotor.move(127);
    matchloader.toggle();
    chassis.waitUntilDone();
    pros::delay(1500);
    chassis.moveToPoint(-31.92, 47.76, 1204, {.forwards = false});
    AutonRecovery::snapRightToWall(595.376, 270);
    outtakeMotor.move(127);
}

void skills_auton() {
    // TODO: implement skills auton.
}
