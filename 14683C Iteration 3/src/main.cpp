#include "main.h"
#include "autons.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "auton_selector.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/screen.hpp"
#include "robot_config.hpp"
#include "pros/motors.h"
#include "ui/ui_root.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <utility>

constexpr double JOYSTICK_SCALE = 127.0;
constexpr double DEADBAND = 0.03;
constexpr double THROTTLE_EXPO = 1.05;
constexpr double TURN_EXPO = 1.05;
constexpr double TURN_AT_FULL = 0.70;
constexpr double BASE_STEER = 0.20;
constexpr double TURN_IN_PLACE_GAIN = 0.90;
constexpr double TURN_IN_PLACE_THROTTLE = 0.05;

constexpr std::uint32_t kLvTickMs = 5;

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
                              11.5625, 
                              lemlib::Omniwheel::NEW_325, 
                              450, 
                              2 
);

lemlib::ControllerSettings linearController(14, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            8, // derivative gain (kD)
                                            3, // anti windup
                                            0.75, // small error range, in inches
                                            140, // small error range timeout, in milliseconds
                                            2.5, // large error range, in inches
                                            450, // large error range timeout, in milliseconds
                                            15 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(3, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             14, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             120, // small error range timeout, in milliseconds
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
    
    matchloader.set_value(false);
    wing.set_value(false);
    middescore.set_value(false);
    midgoal.set_value(true);
    pros::delay(30);
    
    chassis.calibrate();
    load_auton_state();

    ui_root::init();
    static pros::Task ui_task([] {
        while (true) {
            lv_tick_inc(kLvTickMs);
            ui_root::update_fast();
            pros::delay(kLvTickMs);
        }
    });
}
void disabled() {

}
void competition_initialize() {
    
}
void autonomous() {
    matchloader.set_value(false);
    wing.set_value(false);
    middescore.set_value(false);
    midgoal.set_value(true);
    run_selected_auton();
}


void opcontrol() {
    midgoal.set_value(true);
    chassis.cancelAllMotions();
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    bool flagStateLoader = false;
    bool flagStateDescore = false;

    bool lastA = false;
    bool lastDown = false;

    while (true) {
        double throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / JOYSTICK_SCALE;
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / JOYSTICK_SCALE;
        auto [left, right] = arcDrive(throttle, turn);
        leftMotors.move(static_cast<int>(left * JOYSTICK_SCALE));
        rightMotors.move(static_cast<int>(right * JOYSTICK_SCALE));

        int intake = 0;
        int outtake = 0;

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake = 127;
            outtake = -127;
            midgoal.set_value(true); 
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake = 127;
            outtake = 127;
            midgoal.set_value(false);
        }
        else {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake = 127;
                outtake = 80; 
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake = -127;
                outtake = -127;
            }
            midgoal.set_value(true);
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

        const bool wingHeld =
            master.get_digital(pros::E_CONTROLLER_DIGITAL_B);
        if (wingHeld) {
            wing.extend();
        } else {
            wing.retract();
        }

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
