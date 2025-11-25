#include "main.h"

static void wait_ms(int ms) {
    pros::delay(ms);
}

void auton_red_left() {
    chassis.setPose(0, 0, 0); // x, y, heading

    chassis.moveToPoint(0,10,999999);
    wait_ms(100);

    // Turn, score, whatever
    // chassis.turnTo( ... );
    // intake.move(127);
    // wait_ms(500);
    // intake.move(0);
}

// Red Right
void auton_red_right() {
    chassis.setPose(0, 0, 0);
}

// Blue Left
void auton_blue_left() {
    chassis.setPose(0, 0, 0);

}

void auton_blue_right() {
    chassis.setPose(0, 0, 0);
}

// AWP variants
void auton_red_left_awp() {
    chassis.setPose(0, 0, 0);
}

void auton_red_right_awp() {
    chassis.setPose(0, 0, 0);
}

void auton_blue_left_awp() {
    chassis.setPose(0, 0, 0);
}

void auton_blue_right_awp() {
    chassis.setPose(0, 0, 0);
}

void auton_skills() {
    chassis.setPose(0, 0, 0);
}
