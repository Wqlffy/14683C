#include "skills_segments.hpp"

#include "lemlib/api.hpp"
#include "robot_config.hpp"

extern lemlib::Chassis chassis;

ASSET(skills2_5_txt);
ASSET(skills2_7_txt);
ASSET(skills2_11_txt);

void skills2() {
    chassis.setPose(-65.520000, 30.000000, 180.000000);
    
    //collect blocks from left field
    chassis.turnToHeading(100.304846, 733);
    chassis.moveToPoint(-28.56, 23.28, 1286);
    pros::delay(50);
    chassis.turnToHeading(116.565051, 518);
    chassis.moveToPoint(-18.0, 18.0, 841);
    pros::delay(50);

    //score in middle goal
    chassis.turnToHeading(315.0, 938);
    chassis.moveToPoint(-12.0, 12.0, 753, {.forwards = false});
    outtakeMotor.move(-127);
    midgoal.retract();
    chassis.waitUntilDone();
    pros::delay(2000);

    //move to left matchloader
    chassis.moveToPoint(-48.24, 48.24, 1564);
    pros::delay(50);
    outtakeMotor.move(0);
    matchloader.extend();
    chassis.turnToHeading(270.0, 600);
    chassis.moveToPoint(-57.6, 48.24, 777);
    chassis.waitUntilDone();
    pros::delay(1200);

    //move to other side of field
    chassis.turnToHeading(226.107305, 596);
    chassis.follow(skills2_5_txt, 1452, 6.488714, false);
    matchloader.retract();
    chassis.waitUntilDone();
    pros::delay(50);
    chassis.turnToHeading(269.577161, 566);
    chassis.moveToPoint(29.04, 63.12, 1824, {.forwards = false});
    pros::delay(50);
    chassis.turnToHeading(305.07091, 558);

    //move to long goal
    chassis.follow(skills2_7_txt, 957, 6.147939, false);
    pros::delay(50);
    chassis.turnToHeading(90.0, 825);
    chassis.moveToPoint(32.16, 48.48, 729, {.forwards = false});
    outtakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(2300);

    //move to left matchloader
    chassis.turnToHeading(90.54051, 460);
    matchloader.extend();
    outtakeMotor.move(0);
    chassis.moveToPoint(57.6, 48.24, 1363);
    pros::delay(1200);

    //move to long goal
    chassis.turnToHeading(90.0, 460);
    chassis.moveToPoint(31.44, 48.24, 1386, {.forwards = false});
    matchloader.retract();
    outtakeMotor.move(127);
    chassis.waitUntilDone();
    pros::delay(2300);

    //enter parking zone
    chassis.follow(skills2_11_txt, 2184, 9.0);
    pros::delay(50);
    chassis.turnToHeading(180.0, 539);
    chassis.moveToPoint(61.2, -26.88, 1507);
}

