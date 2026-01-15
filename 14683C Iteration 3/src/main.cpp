#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "auton_selector.hpp"
#include "auton_recovery.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "robot_config.hpp"
#include "pros/distance.hpp" // IWYU pragma: keep
#include "pros/motors.h"
#include "ui/ui_root.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

constexpr double JOYSTICK_SCALE = 127.0;
constexpr double DEADBAND = 0.05;
constexpr double THROTTLE_EXPO = 1.6;
constexpr double TURN_EXPO = 1.8;
constexpr double TURN_AT_FULL = 0.55;
constexpr double BASE_STEER = 0.15;
constexpr double TURN_IN_PLACE_GAIN = 0.75;
constexpr double TURN_IN_PLACE_THROTTLE = 0.10;

static double clamp(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}

static double apply_deadband_rescale(double value, double db) {
    const double mag = std::abs(value);
    if (mag <= db) {
        return 0.0;
    }
    const double scaled = (mag - db) / (1.0 - db);
    return std::copysign(scaled, value);
}

static double expo(double value, double e) {
    const double mag = std::pow(std::abs(value), e);
    return std::copysign(mag, value);
}

static std::pair<double, double> arcDrive(double throttle, double turn) {
    throttle = apply_deadband_rescale(throttle, DEADBAND);
    turn = apply_deadband_rescale(turn, DEADBAND);

    throttle = expo(throttle, THROTTLE_EXPO);
    turn = expo(turn, TURN_EXPO);

    double left = 0.0;
    double right = 0.0;

    if (std::abs(throttle) < TURN_IN_PLACE_THROTTLE) {
        const double angular = turn * TURN_IN_PLACE_GAIN;
        left = angular;
        right = -angular;
    } else {
        const double angular =
            turn * (TURN_AT_FULL * std::abs(throttle) + BASE_STEER);
        left = throttle + angular;
        right = throttle - angular;
    }

    const double maxMag = std::max(std::abs(left), std::abs(right));
    if (maxMag > 1.0) {
        left /= maxMag;
        right /= maxMag;
    }

    left = clamp(left, -1.0, 1.0);
    right = clamp(right, -1.0, 1.0);
    return {left, right};
}


pros::Controller master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors, 
                              11, 
                              lemlib::Omniwheel::NEW_325, 
                              450, 
                              2 
);

lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(nullptr,
                            nullptr,
                            nullptr,
                            nullptr, 
                            &imu 
);

lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void initialize() {
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    
    chassis.calibrate();
    load_auton_state();

    ui_root::init();
    static pros::Task ui_task([] {
        while (true) {
            ui_root::update_fast();
            pros::delay(100);
        }
    });
            pros::delay(50);
        }
void disabled() {

}
void competition_initialize() {
    
}
void autonomous() {
    run_selected_auton();
    /*
    // Example usage (inside a contact-heavy auton segment):
    // AutonRecovery::recoverAfterContact(300, 0.0);
    //
    // Example wrapper with monitoring:
    // AutonRecovery::runSegmentWithRecovery(
    //     [&]() { chassis.moveToPoint(24, 0, 1500); },
    //     [&]() { return 0.6; },
    //     300, 0.0, true, 2, 20);
    */
}


void opcontrol() {
    chassis.cancelAllMotions();
    bool flagStateLoader = false;
    bool flagStateWing = false;
    bool flagStateDescore = false;

    bool lastA = false;
    bool lastB = false;
    bool lastDown = false;

    while (true) {
        double throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / JOYSTICK_SCALE;
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / JOYSTICK_SCALE;
        auto [left, right] = arcDrive(throttle, turn);
        leftMotors.move(static_cast<int>(left * JOYSTICK_SCALE));
        rightMotors.move(static_cast<int>(right * JOYSTICK_SCALE));

        int intake = 0;
        int outtake = 0;

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake = 127;
            outtake = 127;
            midgoal.set_value(true); 
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake = 127;
            outtake = 127;
            midgoal.set_value(false);
        }
        else {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake = 127;
                outtake = -40; 
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake = -127;
                outtake = -127;
            }
            midgoal.set_value(false);
        }

        intakeMotor.move(intake);
        outtakeMotor.move(outtake);

        bool currA = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);

        if (currA && !lastA) {
            flagStateLoader = !flagStateLoader;
            if (flagStateLoader)
                matchloader.extend();
            else
                matchloader.retract();
        }

        lastA = currA;

        bool currB = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (currB && !lastB) {
            flagStateWing = !flagStateWing;
            if (flagStateWing)
                wing.extend();
            else
                wing.retract();
        }

        lastB = currB;

        bool currDown = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        if (currDown && !lastDown) {
            flagStateDescore = !flagStateDescore;
            if (flagStateDescore)
                middescore.extend();
            else
                middescore.retract();
        }

        lastDown = currDown;

        pros::delay(10);
    }
}
