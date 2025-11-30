#include "autonomous.hpp"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

ASSET(red_left_txt);
ASSET(red_right_txt);
ASSET(blue_left_txt);
ASSET(blue_right_txt);

extern lemlib::Chassis chassis;

extern pros::Motor intake;
extern pros::Motor indexer;
extern pros::Motor scoring;
extern pros::adi::Pneumatics tongue;
extern pros::adi::Pneumatics aligner;


static void wait_ms(int ms) {
    pros::delay(ms);
}

void auton_red_left() {
    chassis.setPose(-46.224, 7.195, 0); 
    chassis.follow(red_left_txt, 12, 8000, true, true);

    //start intake + tongue out
    chassis.waitUntil(43.952);
    intake.move_voltage(12000);
    tongue.set_value(true);

    //wait 1.5 s
    chassis.waitUntil(59.951);
    wait_ms(1500);

    // stop intake + tongue in, aligner out
    chassis.waitUntil(77.973);
    intake.move_voltage(12000);
    tongue.set_value(false);
    aligner.set_value(true);

    //scorer + indexer on , wait 1.5 s 
    chassis.waitUntil(91.973);
    scoring.move_voltage(12000);
    indexer.move_voltage(12000);
    wait_ms(1500);

    //scorer + indexer off intake on
    chassis.waitUntil(103.483);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    aligner.set_value(false);
    intake.move_voltage(12000);

    //intake off
    chassis.waitUntil(123.479);
    intake.move_voltage(0);

    //scorer positive + indexer negative, wait 1.5 s
    chassis.waitUntil(134.026);
    scoring.move_voltage(12000);
    indexer.move_voltage(-12000);
    wait_ms(1500);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    
    chassis.waitUntilDone();

    intake.move_voltage(0);
    indexer.move_voltage(0);
    scoring.move_voltage(0);
}

void auton_red_right() {
    chassis.setPose(-46.224, -7.195, 0);
    chassis.follow(red_right_txt, 12, 8000, true, true);

    //start intake + tongue out
    chassis.waitUntil(43.952);
    intake.move_voltage(12000);
    tongue.set_value(true);

    //wait 1.5 s
    chassis.waitUntil(59.951);
    wait_ms(1500);

    // stop intake + tongue in, aligner out
    chassis.waitUntil(77.973);
    intake.move_voltage(0);
    tongue.set_value(false);
    aligner.set_value(true);

    //scorer + indexer on , wait 1.5 s 
    chassis.waitUntil(91.973);
    scoring.move_voltage(12000);
    indexer.move_voltage(12000);
    wait_ms(1500);

    //scorer + indexer off intake on
    chassis.waitUntil(103.483);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    aligner.set_value(false);
    intake.move_voltage(12000);

    //intake off
    chassis.waitUntil(123.479);
    intake.move_voltage(0);

    //intake negative + indexer negative, wait 1.5 s
    chassis.waitUntil(134.026);
    scoring.move_voltage(-12000);
    intake.move(-12000);
    wait_ms(1500);
    scoring.move_voltage(0);
    intake.move_voltage(0);
    
    chassis.waitUntilDone();

    intake.move_voltage(0);
    indexer.move_voltage(0);
    scoring.move_voltage(0);
}

void auton_blue_left() {
    chassis.setPose(46.224, -7.195, 0);
    chassis.follow(blue_left_txt, 12, 8000, true, true);

    //start intake + tongue out
    chassis.waitUntil(43.952);
    intake.move_voltage(12000);
    tongue.set_value(true);

    //wait 1.5 s
    chassis.waitUntil(59.951);
    wait_ms(1500);

    // stop intake + tongue in, aligner out
    chassis.waitUntil(77.973);
    intake.move_voltage(0);
    tongue.set_value(false);
    aligner.set_value(true);

    //scorer + indexer on , wait 1.5 s 
    chassis.waitUntil(91.973);
    scoring.move_voltage(12000);
    indexer.move_voltage(12000);
    wait_ms(1500);

    //scorer + indexer off intake on
    chassis.waitUntil(103.483);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    aligner.set_value(false);
    intake.move_voltage(12000);

    //intake off
    chassis.waitUntil(123.479);
    intake.move_voltage(0);

    //scorer positive + indexer negative, wait 1.5 s
    chassis.waitUntil(134.026);
    scoring.move_voltage(12000);
    indexer.move_voltage(-12000);
    wait_ms(1500);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    
    chassis.waitUntilDone();

    intake.move_voltage(0);
    indexer.move_voltage(0);
    scoring.move_voltage(0);
}

void auton_blue_right() {
    chassis.setPose(46.224, 7.195, 0);
    chassis.follow(blue_right_txt, 12, 8000, true, true);

    //start intake + tongue out
    chassis.waitUntil(43.952);
    intake.move_voltage(12000);
    tongue.set_value(true);

    //wait 1.5 s
    chassis.waitUntil(59.951);
    wait_ms(1500);

    // stop intake + tongue in, aligner out
    chassis.waitUntil(77.973);
    intake.move_voltage(0);
    tongue.set_value(false);
    aligner.set_value(true);

    //scorer + indexer on , wait 1.5 s 
    chassis.waitUntil(91.973);
    scoring.move_voltage(12000);
    indexer.move_voltage(12000);
    wait_ms(1500);

    //scorer + indexer off intake on
    chassis.waitUntil(103.483);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    aligner.set_value(false);
    intake.move_voltage(12000);

    //intake off
    chassis.waitUntil(123.479);
    intake.move_voltage(0);

    //intake negative + indexer negative, wait 1.5 s
    chassis.waitUntil(134.026);
    scoring.move_voltage(-12000);
    intake.move(-12000);
    wait_ms(1500);
    scoring.move_voltage(0);
    intake.move_voltage(0);
    
    chassis.waitUntilDone();

    intake.move_voltage(0);
    indexer.move_voltage(0);
    scoring.move_voltage(0);
}


