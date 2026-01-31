#include "skills_segments.hpp"

#include "lemlib/api.hpp"
#include "robot_config.hpp"

extern lemlib::Chassis chassis;

void skills1() {
    chassis.setPose(-62.160000, -17.520000, 0.000000);

    chassis.moveToPoint(-62.16, 27.6, 1442);
    intakeMotor.move(127);
    chassis.waitUntilDone();
}

