#include "autons.hpp"

#include "lemlib/api.hpp"
#include "main.h"
#include "auton_recovery.hpp"
#include "robot_config.hpp"
#include "auton_recovery.hpp"

extern lemlib::Chassis chassis;

ASSET(skills91_2_txt);
ASSET(skills91_6_txt);

void auton_43_blue_left() {
    // chassis.setPose(-48.000000, 6.000000, 0.000000);

    // //move to matchloader
    // // AutonRecovery::moveToPointWithWallAssist(-48, 48.24, 1377, AutonRecovery::WallSide::LEFT, 500, 0); //change targetMm
    // chassis.moveToPoint(-48, 42,1377);
    // pros::delay(400);
    // chassis.turnToHeading(90, 767);
    // matchloader.extend();
    // intakeMotor.move(127);

    // //enter matchloader
    // AutonRecovery::runSegmentWithRecovery(
    //     [&]() {
    //         chassis.moveToPoint(-57.6, 48.24, 829, {.maxSpeed = 90});
    //         chassis.waitUntilDone();
    //     },
    //     []() { return 0.65; }, 473, 270, false, 2, 20);
    // pros::delay(1000); //tune matchloader time

    // cdrift(-50, 300);
    // chassis.moveToPoint(-57.6, 48.24, 829);
    // chassis.waitUntilDone();
    // pros::delay(800);
    

    // //move to long goal
    // AutonRecovery::moveToPointWithWallAssist(-32.64, 47.76, 1158, AutonRecovery::WallSide::RIGHT, 400, 270, false); //change targetMm
    // // chassis.moveToPoint(-32.64, 47.76, 1158, {.forwards = false});
    // chassis.waitUntilDone();
    // AutonRecovery::snapRightToWall(473, 270);
    // matchloader.retract();
    // pros::delay(150);
    // intakeMotor.move(127);
    // outtakeMotor.move(127);
    // pros::delay(1500); //tune scoring time
    // outtakeMotor.move(0);

    // //move to middle goal
    // AutonRecovery::moveToPointWithWallAssist(-48, 42, 1241, AutonRecovery::WallSide::RIGHT, 473, 0); //change targetMm
    // // chassis.moveToPoint(-48.0, 42.0, 1241);
    // pros::delay(50);
    // chassis.turnToHeading(135.0, 994);
    // AutonRecovery::resetDriveDistance();
    // chassis.moveToPoint(-23.76, 23.76, 1409);
    // AutonRecovery::waitUntilDriveDistanceIn(21.375414, 1409);
    // matchloader.extend();
    // intakeMotor.move(127);
    // chassis.waitUntilDone();
    // pros::delay(1000);
    // matchloader.retract();

    // //score in middle goal
    // chassis.turnToHeading(317.663001, 987);
    // chassis.moveToPoint(-13.92, 12.96, 1157, {.forwards = false});
    // midgoal.retract();
    // outtakeMotor.move(-127);
    // chassis.waitUntilDone();
    // intakeMotor.move(0);
    
    // outtakeMotor.move(0);

    cdrift(50, 300);
    pros::delay(100);
}

void auton_43_red_left() {
    chassis.setPose(-48.000000, 6.000000, 0.000000);

    //move to matchloader
    AutonRecovery::moveToPointWithWallAssist(-48, 42, 1377, AutonRecovery::WallSide::LEFT, 482, 0); //change targetMm
    pros::delay(400);
    chassis.turnToHeading(270.0, 767);
    matchloader.extend();
    intakeMotor.move(127);

    //enter matchloader
    AutonRecovery::runSegmentWithRecovery(
        [&]() {
            chassis.moveToPoint(-57.6, 48.24, 829, {.maxSpeed = 90});
            chassis.waitUntilDone();
        },
        []() { return 0.65; }, 473, 270, false, 2, 20);
    pros::delay(1000); //tune matchloader time

    cdrift(-50, 300);
    chassis.moveToPoint(-57.6, 48.24, 829);
    chassis.waitUntilDone();
    pros::delay(800);
    

    //move to long goal
    AutonRecovery::moveToPointWithWallAssist(-32.64, 47.76, 1158, AutonRecovery::WallSide::RIGHT, 473, 270, false); //change targetMm
    // chassis.moveToPoint(-32.64, 47.76, 1158, {.forwards = false});
    chassis.waitUntilDone();
    AutonRecovery::snapRightToWall(440, 270);
    matchloader.retract();
    pros::delay(150);
    intakeMotor.move(127);
    outtakeMotor.move(127);
    pros::delay(1500); //tune scoring time
    outtakeMotor.move(0);

    //move to middle goal
    AutonRecovery::moveToPointWithWallAssist(-48, 42, 1241, AutonRecovery::WallSide::RIGHT, 473, 0); //change targetMm
    // chassis.moveToPoint(-48.0, 42.0, 1241);
    pros::delay(50);
    chassis.turnToHeading(135.0, 994);
    AutonRecovery::resetDriveDistance();
    chassis.moveToPoint(-23.76, 23.76, 1409);
    AutonRecovery::waitUntilDriveDistanceIn(21.375414, 1409);
    matchloader.extend();
    intakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(1000);
    matchloader.retract();

    //score in middle goal
    chassis.turnToHeading(317.663001, 987);
    chassis.moveToPoint(-13.92, 12.96, 1157, {.forwards = false});
    midgoal.retract();
    outtakeMotor.move(-127);
    chassis.waitUntilDone();
    intakeMotor.move(0);
    outtakeMotor.move(0);
}

void auton_9_long_blue_left() {
    chassis.setPose(-63.840000, 17.040000, 90.000000);

    //collect 3 blocks from field
    chassis.moveToPoint(-47.04, 17.04, 1028);
    pros::delay(50);
    chassis.turnToHeading(74.638384, 558);
    chassis.moveToPoint(-25.2, 23.04, 1161); //tune the distance, would probably need to use something else
    matchloader.extend();
    intakeMotor.move(127);

    //move to matchloader
    matchloader.retract();
    chassis.turnToHeading(317.589502, 966);
    chassis.moveToPoint(-48.0, 42.0, 1399);
    pros::delay(50);
    matchloader.extend();
    chassis.turnToHeading(270.0, 685);
    AutonRecovery::snapRightToWall(473, 270, 400, 20, 0.20);
    AutonRecovery::moveToPointWithWallAssist(-57.36, 48, 876, AutonRecovery::WallSide::RIGHT, 473, 270, true, 30, 20, 120, 300, 150); //tune targetMm
    // chassis.moveToPoint(-57.36, 48.0, 876);
    pros::delay(1000);

    cdrift(-50, 300);
    AutonRecovery::moveToPointWithWallAssist(-57.36, 48, 876, AutonRecovery::WallSide::RIGHT, 473, 270, true, 30, 20, 120, 300, 150); //tune targetMm
    chassis.waitUntilDone();
    pros::delay(800);

    //move to long goal
    AutonRecovery::moveToPointWithWallAssist(-31.68, 48, 1226, AutonRecovery::WallSide::RIGHT, 473, 270, false, 30, 20, 120, 300, 150); //tune targetMm
    // chassis.moveToPoint(-31.68, 48.0, 1226, {.forwards = false});
    // AutonRecovery::snapLeftToWall(473, 270, 400, 20, 0.20); //tune targetMm
    matchloader.retract();
    outtakeMotor.move(127);
    intakeMotor.move(127);
    pros::delay(3000);

    //wing
    outtakeMotor.move(0);
    chassis.turnToHeading(268.806511, 461);
    chassis.moveToPoint(-43.2, 47.76, 943);
    wing.extend();
    chassis.swingToHeading(332.931219, DriveSide::LEFT, 675, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
    chassis.moveToPose(-21.36, 34.8, 270.0, 1131, {.forwards = false, .lead = 0.6});
    wing.retract();
    pros::delay(50);
    chassis.turnToHeading(270.0, 1125);
    chassis.moveToPoint(-5.52, 34.8, 1004, {.forwards = false});
}

void auton_9_long_blue_right() {
    chassis.setPose(-63.840000, -17.040000, 90.000000);
    
    //collect 3 blocks from field
    chassis.moveToPoint(-47.04, -17.04, 1028);
    pros::delay(50);
    chassis.turnToHeading(105.361616, 558);
    chassis.moveToPoint(-25.2, -23.04, 1161); //tune the distance, would probably be something else
    pros::delay(50);
    intakeMotor.move(127);
    chassis.turnToHeading(222.410498, 966);

    //move to matchloader
    chassis.moveToPoint(-48.0, -48.0, 1399);
    pros::delay(50);
    chassis.turnToHeading(270.0, 685);
    matchloader.extend();
    AutonRecovery::snapLeftToWall(440, 270); //tune targetMm
    AutonRecovery::moveToPointWithWallAssist(-57.36, -48, 876, AutonRecovery::WallSide::LEFT, 440, 270); //tune targetMm
    // chassis.moveToPoint(-57.36, -48.0, 876);
    pros::delay(1000);

    cdrift(-50, 300);
    AutonRecovery::moveToPointWithWallAssist(-57.36, -48, 876, AutonRecovery::WallSide::LEFT, 440, 270); //tune targetMm
    // chassis.moveToPoint(-57.36, -48.0, 876, {.maxSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(800);

    //move to long goal
    intakeMotor.move(127);
    AutonRecovery::moveToPointWithWallAssist(-31.68, -48.0, 1226, AutonRecovery::WallSide::LEFT, 440, 270, false); //tune targetMm
    // chassis.moveToPoint(-31.68, -48.0, 1226, {.forwards = false});
    AutonRecovery::snapLeftToWall(440, 270);
    matchloader.retract();
    outtakeMotor.move(127);
    intakeMotor.move(127);
    pros::delay(3000);

    // //wing
    // outtakeMotor.move(0);
    // chassis.turnToHeading(268.830861, 461);
    // chassis.moveToPoint(-43.44, -48.24, 949);
    // pros::delay(50);
    // chassis.turnToHeading(4.763642, 890);
    // chassis.moveToPoint(-42.48, -36.72, 944);
    // wing.extend();
    // pros::delay(50);
    // chassis.turnToHeading(90.0, 850);
    // chassis.moveToPoint(-24.72, -36.72, 1051);
    // wing.retract();
    // pros::delay(500);
    // chassis.moveToPoint(-10.08, -36.72, 974);
}

void auton_9_long_red_left() {
    chassis.setPose(-63.840000, 17.040000, 90.000000);

    //collect 3 blocks from field
    chassis.moveToPoint(-47.04, 17.04, 1028);
    pros::delay(50);
    chassis.turnToHeading(74.638384, 558);
    chassis.moveToPoint(-25.2, 23.04, 1161); //tune the distance, would probably need to use something else
    matchloader.extend();
    pros::delay(50);
    intakeMotor.move(127);

    //move to matchloader
    chassis.turnToHeading(317.589502, 966);
    chassis.moveToPoint(-48.0, 42.0, 1399);
    pros::delay(50);
    matchloader.extend();
    chassis.turnToHeading(270.0, 685);
    AutonRecovery::snapRightToWall(440, 270);
    AutonRecovery::moveToPointWithWallAssist(-57.36, 48, 876, AutonRecovery::WallSide::RIGHT, 440, 270); //tune targetMm
    // chassis.moveToPoint(-57.36, 48.0, 876);
    pros::delay(1000);

    cdrift(-50, 300);
    AutonRecovery::moveToPointWithWallAssist(-57.36, 48, 876, AutonRecovery::WallSide::RIGHT, 440, 270); //tune targetMm
    chassis.waitUntilDone();
    pros::delay(800);

    //move to long goal
    AutonRecovery::moveToPointWithWallAssist(-31.68, 48, 1226, AutonRecovery::WallSide::RIGHT, 440, 270, false); //tune targetMm
    // chassis.moveToPoint(-31.68, 48.0, 1226, {.forwards = false});
    AutonRecovery::snapLeftToWall(440, 270); //tune targetMm
    matchloader.retract();
    outtakeMotor.move(127);
    intakeMotor.move(127);
    pros::delay(3000);

//     //wing
//     outtakeMotor.move(0);
//     chassis.turnToHeading(268.806511, 461);
//     chassis.moveToPoint(-43.2, 47.76, 943);
//     wing.extend();
//     chassis.swingToHeading(332.931219, DriveSide::LEFT, 675, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
//     chassis.moveToPose(-21.36, 34.8, 270.0, 1131, {.forwards = false, .lead = 0.6});
//     wing.retract();
//     pros::delay(50);
//     chassis.turnToHeading(270.0, 1125);
//     chassis.moveToPoint(-5.52, 34.8, 1004, {.forwards = false});
}

void auton_9_long_red_right() {
    chassis.setPose(-63.840000, -17.040000, 90.000000);
    
    //collect 3 blocks from field
    chassis.moveToPoint(-47.04, -17.04, 1028);
    pros::delay(50);
    chassis.turnToHeading(105.361616, 558);
    chassis.moveToPoint(-25.2, -23.04, 1161); //tune the distance, would probably be something else
    pros::delay(50);
    intakeMotor.move(127);
    chassis.turnToHeading(222.410498, 966);

    //move to matchloader
    chassis.moveToPoint(-48.0, -48.0, 1399);
    pros::delay(50);
    chassis.turnToHeading(270.0, 685);
    matchloader.extend();
    AutonRecovery::snapLeftToWall(440, 270); //tune targetMm
    AutonRecovery::moveToPointWithWallAssist(-57.36, -48, 876, AutonRecovery::WallSide::LEFT, 440, 270); //tune targetMm
    // chassis.moveToPoint(-57.36, -48.0, 876);
    pros::delay(1000);

    cdrift(-50, 300);
    AutonRecovery::moveToPointWithWallAssist(-57.36, -48, 876, AutonRecovery::WallSide::LEFT, 440, 270); //tune targetMm
    // chassis.moveToPoint(-57.36, -48.0, 876, {.maxSpeed = 90});
    chassis.waitUntilDone();
    pros::delay(800);

    //move to long goal
    intakeMotor.move(127);
    AutonRecovery::moveToPointWithWallAssist(-31.68, -48.0, 1226, AutonRecovery::WallSide::LEFT, 440, 270, false); //tune targetMm
    // chassis.moveToPoint(-31.68, -48.0, 1226, {.forwards = false});
    AutonRecovery::snapLeftToWall(440, 270);
    matchloader.retract();
    outtakeMotor.move(127);
    intakeMotor.move(127);
    pros::delay(3000);

    // //wing
    // outtakeMotor.move(0);
    // chassis.turnToHeading(268.830861, 461);
    // chassis.moveToPoint(-43.44, -48.24, 949);
    // pros::delay(50);
    // chassis.turnToHeading(4.763642, 890);
    // chassis.moveToPoint(-42.48, -36.72, 944);
    // wing.extend();
    // pros::delay(50);
    // chassis.turnToHeading(90.0, 850);
    // chassis.moveToPoint(-24.72, -36.72, 1051);
    // wing.retract();
    // pros::delay(500);
    // chassis.moveToPoint(-10.08, -36.72, 974);
}

void auton_awp_blue_right() {
    chassis.setPose(-48.000000, -6.000000, 180.000000);

    //move to matchloader
    chassis.moveToPoint(-48.0, -48.0, 1377);
    pros::delay(50);
    chassis.turnToHeading(270.0, 767);
    
    //enter right matchloader
    intakeMotor.move(127);
    matchloader.extend();
    chassis.moveToPoint(-62.88, -48.0, 856);
    pros::delay(1800); //tune matchloader time

    //move to right long goal
    chassis.moveToPoint(-31.92, -48.24, 1199, {.forwards = false});
    AutonRecovery::snapRightToWall(440, 270);
    outtakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(2000); //tune scoring time

    //move to middle goal
    outtakeMotor.move(0);
    chassis.moveToPoint(-41.28, -48.24, 777);
    pros::delay(100);
    chassis.turnToHeading(35.48398, 841);

    //collect 3 blocks from right field
    chassis.moveToPoint(-24.0, -24.0, 1503);
    chassis.waitUntil(24.830799);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);

    //collect 3 blocks from left field
    chassis.turnToHeading(0.0, 558);
    chassis.moveToPoint(-24.0, 24.0, 1935);
    chassis.waitUntil(5.76);
    matchloader.retract();
    chassis.waitUntil(44.64);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);

    //score in middle goal 
    chassis.turnToHeading(315.643746, 598);
    chassis.moveToPoint(-13.44, 13.2, 860, {.forwards = false});
    chassis.waitUntil(4.919246);
    matchloader.retract();
    chassis.waitUntil(15.104754);
    outtakeMotor.move(-127);
    midgoal.retract();
    chassis.waitUntilDone();
    pros::delay(1000); //tune scoring time

    //move to left matchloader
    outtakeMotor.move(0);
    midgoal.extend();
    chassis.moveToPoint(-48.0, 42.0, 1522);
    matchloader.extend();
    intakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(50);

    //enter left matchloader
    chassis.turnToHeading(270.0, 602);
    chassis.moveToPoint(-57.6, 48.24, 860, {.maxSpeed= 90}); //test speed
    chassis.waitUntilDone();
    pros::delay(1500); //tune matchloader time

    //score in left long goal
    chassis.moveToPoint(-31.92, 47.76, 1204, {.forwards = false});
    AutonRecovery::snapLeftToWall(440, 270);
    outtakeMotor.move(127);
}

void auton_awp_red_right() {
    chassis.setPose(-48.000000, -6.000000, 180.000000);

    //move to matchloader
    chassis.moveToPoint(-48.0, -48.0, 1377);
    pros::delay(50);
    chassis.turnToHeading(270.0, 767);
    
    //enter right matchloader
    intakeMotor.move(127);
    matchloader.extend();
    chassis.moveToPoint(-62.88, -48.0, 856);
    pros::delay(1800); //tune matchloader time

    //move to right long goal
    chassis.moveToPoint(-31.92, -48.24, 1199, {.forwards = false});
    AutonRecovery::snapRightToWall(440, 270);
    outtakeMotor.move(127);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(2000); //tune scoring time

    //move to middle goal
    outtakeMotor.move(0);
    chassis.moveToPoint(-41.28, -48.24, 777);
    pros::delay(100);
    chassis.turnToHeading(35.48398, 841);

    //collect 3 blocks from right field
    chassis.moveToPoint(-24.0, -24.0, 1503);
    chassis.waitUntil(24.830799);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);

    //collect 3 blocks from left field
    chassis.turnToHeading(0.0, 558);
    chassis.moveToPoint(-24.0, 24.0, 1935);
    chassis.waitUntil(5.76);
    matchloader.retract();
    chassis.waitUntil(44.64);
    intakeMotor.move(127);
    matchloader.extend();
    chassis.waitUntilDone();
    pros::delay(50);

    //score in middle goal 
    chassis.turnToHeading(315.643746, 598);
    chassis.moveToPoint(-13.44, 13.2, 860, {.forwards = false});
    chassis.waitUntil(4.919246);
    matchloader.retract();
    chassis.waitUntil(15.104754);
    outtakeMotor.move(-127);
    midgoal.retract();
    chassis.waitUntilDone();
    pros::delay(1000); //tune scoring time

    //move to left matchloader
    outtakeMotor.move(0);
    midgoal.extend();
    chassis.moveToPoint(-48.0, 42.0, 1522);
    matchloader.extend();
    intakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(50);

    //enter left matchloader
    chassis.turnToHeading(270.0, 602);
    chassis.moveToPoint(-57.6, 48.24, 860, {.maxSpeed= 90}); //test speed
    chassis.waitUntilDone();
    pros::delay(1500); //tune matchloader time

    //score in left long goal
    chassis.moveToPoint(-31.92, 47.76, 1204, {.forwards = false});
    AutonRecovery::snapLeftToWall(440, 270);
    outtakeMotor.move(127);
}

void skills_auton() {
    auton_skills();
}

static int clamp_motor_power(int power) {
    if (power > 127) {
        return 127;
    }
    if (power < -127) {
        return -127;
    }
    return power;
}

void cdrift(int power, int duration_ms) {
    const int clamped = clamp_motor_power(power);
    leftMotors.move(clamped);
    rightMotors.move(clamped);
    pros::delay(duration_ms);
    leftMotors.move(0);
    rightMotors.move(0);
}

// void test_91() {
//     chassis.setPose(-50.400000, 12.720000, 90.000000);
    
//     //move to left matchloader
//     chassis.turnToHeading(27.85293, 670);
//     chassis.moveToPose(-56.750187, 47.924576, 270.0, 1466, {.lead = 0.7});
//     chassis.waitUntil(18.571491);
//     matchloader.extend();
//     chassis.waitUntil(32.769549);
//     intakeMotor.move(127);
//     chassis.waitUntilDone();
//     pros::delay(2000);

//     //move to right long goal
//     chassis.moveToPoint(-48.0, 48.0, 760, {.forwards = false});
//     pros::delay(50);
//     matchloader.retract();
//     chassis.turnToHeading(139.490614, 853);
//     chassis.follow(skills91_2_txt, 2685, 6.56257);
//     chassis.waitUntil(26.870328);
//     matchloader.extend();
//     chassis.waitUntilDone();
//     pros::delay(50);
//     intakeMotor.move(127);

//     //score in right long goal
//     chassis.turnToHeading(270.0, 705);
//     chassis.moveToPoint(-31.0, -48.0, 706, {.forwards = false});
//     outtakeMotor.move(127);
//     pros::delay(3000);

//     //move to right matchloader
//     outtakeMotor.move(0);
//     chassis.moveToPoint(-56.75, -48.0, 1088);
//     intakeMotor.move(127);
//     pros::delay(2000);

//     //move to left long goal
//     chassis.moveToPoint(-48.0, -48.0, 760, {.forwards = false});
//     pros::delay(50);
//     matchloader.retract();
//     chassis.turnToHeading(39.573434, 852);
//     chassis.follow(skills91_6_txt, 2687, 11.25);
//     chassis.waitUntil(26.427919);
//     matchloader.extend();
//     chassis.waitUntilDone();
//     pros::delay(50);
//     matchloader.retract();

//     //score in left long goal
//     chassis.turnToHeading(270.0, 702);
//     chassis.moveToPoint(-31.0, 48.0, 721, {.forwards = false});
//     outtakeMotor.move(127);
//     pros::delay(3000);

//     //move to opposite side middle goal
//     outtakeMotor.move(0);
//     chassis.moveToPoint(-34.32, 48.0, 576, {.minSpeed = 44, .earlyExitRange = 1.99});
//     chassis.swingToHeading(
//         103.369265, DriveSide::RIGHT, 825,
//         {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
//     chassis.moveToPoint(24.0, 24.0, 1750);
//     chassis.waitUntil(53.99237);
//     matchloader.extend();
//     intakeMotor.move(127);
//     chassis.waitUntilDone();
//     pros::delay(1000);

//     //score in bottom middle goal
//     matchloader.retract();
//     chassis.turnToHeading(225.0, 830);
//     chassis.moveToPoint(14.4, 14.4, 827);
//     intakeMotor.move(-127);
//     pros::delay(2000);

//     //move to left matchloader
//     intakeMotor.move(0);
//     chassis.moveToPoint(48.0, 48.0, 1492, {.forwards = false});
//     pros::delay(50);
//     chassis.turnToHeading(90.0, 867);
//     matchloader.extend();
//     intakeMotor.move(127);
//     chassis.moveToPoint(56.75, 48.0, 760);
//     pros::delay(2000);

//     //score in left long goal
//     chassis.moveToPoint(31.0, 48.0, 1088, {.forwards = false});
//     matchloader.retract();
//     outtakeMotor.move(127);
//     pros::delay(3000);

//     //move to parking zone
//     outtakeMotor.move(0);
//     chassis.moveToPoint(42.48, 48.0, 833, {.minSpeed = 51, .earlyExitRange = 3.57});
//     chassis.swingToHeading(
//         123.015891, DriveSide::LEFT, 484,
//         {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
//     chassis.moveToPose(63.36, -0.0, 180.0, 1591, {.lead = 0.7});
//     chassis.waitUntil(37.955031);
//     intakeMotor.move(127);
//     chassis.waitUntilDone();
//     pros::delay(2000);

//     //wall reset
//     {
//         AutonRecovery::WallResetParams p{};
//         p.faceHeadingDeg = 270.0;
//         p.leftTargetMm = 440;
//         p.rightTargetMm = 440;
//         p.squareTimeoutMs = 800;
//         p.setTimeoutMs = 900;
//         p.tolMm = 10;
//         p.maxTurn = AutonRecovery::Tuning::maxTurn;
//         p.maxFwd = AutonRecovery::Tuning::maxFwd;
//         p.trySquare = true;
//         AutonRecovery::wallReset(p);
//     }

//     //move to right matchloader
//     chassis.moveToPose(50.16, -35.52, 270.0, 1362,
//                        {.lead = 0.35, .minSpeed = 41, .earlyExitRange = 2.97},
//                        true);
//     {
//         AutonRecovery::WallResetParams p{};
//         AutonRecovery::wallReset(p);
//     }
//     chassis.waitUntil(28.075482);
//     intakeMotor.move(0);
//     matchloader.extend();
//     chassis.waitUntilDone();
//     intakeMotor.move(127);
//     chassis.swingToHeading(
//         97.107189, DriveSide::RIGHT, 839,
//         {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
//     chassis.moveToPose(56.844985, -46.300579, 90.0, 781, {.lead = 0.1});
//     pros::delay(2000);

//     //score in right long goal
//     chassis.turnToHeading(86.237968, 463);
//     chassis.moveToPoint(31.0, -48.0, 1091, {.forwards = false});
//     matchloader.retract();
//     outtakeMotor.move(127);
//     pros::delay(3000);

//     //move to middle goal
//     outtakeMotor.move(0);
//     chassis.turnToHeading(90.0, 463);
//     chassis.moveToPoint(34.08, -48.0, 566, {.minSpeed = 52, .earlyExitRange = 1.85});
//     chassis.swingToHeading(
//         318.611344, DriveSide::RIGHT, 745,
//         {.direction = AngularDirection::CCW_COUNTERCLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
//     chassis.moveToPoint(24.0, -24.0, 985, {.minSpeed = 65, .earlyExitRange = 4.78});
//     chassis.waitUntil(15.629313);
//     matchloader.extend();
//     intakeMotor.move(127);
//     chassis.waitUntilDone();
//     chassis.swingToHeading(
//         16.583369, DriveSide::LEFT, 578,
//         {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
//     chassis.moveToPoint(24.48, -14.16, 654);
//     chassis.swingToHeading(135.308882, DriveSide::LEFT, 752, {.direction = AngularDirection::CW_CLOCKWISE});
//     pros::delay(50);
//     chassis.moveToPose(12.834936, -12.904316, 315.0, 693,
//                        {.forwards = false, .lead = 0.3});
//     matchloader.retract();
//     midgoal.retract();
//     outtakeMotor.move(-127);
//     pros::delay(2000);

//     //move to parking zone
//     outtakeMotor.move(0);
//     chassis.turnToHeading(134.23863, 461);
//     chassis.moveToPoint(15.48, -15.48, 592, {.minSpeed = 52, .earlyExitRange = 2.22});
//     chassis.swingToHeading(
//         255.792506, DriveSide::LEFT, 721,
//         {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
//     chassis.moveToPoint(-51.6, -40.32, 1858);
//     chassis.swingToHeading(
//         300.4526, DriveSide::LEFT, 531,
//         {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 40, .earlyExitRange = 7.0});
//     chassis.turnToHeading(301.007972, 460);
//     chassis.moveToPose(-63.6, -0.0, 0.0, 1372, {.lead = 0.86});
//     chassis.waitUntil(26.163392);
//     intakeMotor.move(127);
//     chassis.waitUntilDone();

//     intakeMotor.move(0);
//     outtakeMotor.move(0);
//     matchloader.retract();
// }
