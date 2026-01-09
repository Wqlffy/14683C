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

constexpr double PI = 3.14159265358979323846;
constexpr double DRIVE_DEADBAND = 0.04;
constexpr double DRIVE_SLEW = 0.135;
constexpr double CD_TURN_NONLINEARITY = 0.55;
constexpr double CD_NEG_INERTIA_SCALAR = 2.15;
constexpr double CD_SENSITIVITY = 1.42;
constexpr double JOYSTICK_SCALE = 127.0;
constexpr double TIP_THRESHOLD = 0.14; // 0.12–0.18 range
constexpr double TIP_CREEP_SCALE = 0.35;   // 0.25–0.45 typical


constexpr double MIN_TURN_GAIN = 0.93;
constexpr double TURN_IN_PLACE_GAIN = 1.18;

static double _clamp(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}

static double _turnRemapping(double iturn) {
    double denominator = std::sin(PI / 2 * CD_TURN_NONLINEARITY);
    double first =
        std::sin(PI / 2 * CD_TURN_NONLINEARITY * iturn) / denominator;
    return std::sin(PI / 2 * CD_TURN_NONLINEARITY * first) / denominator;
}

static double quickStopAccumlator = 0.0;
static double negInertiaAccumlator = 0.0;
static void _updateAccumulators() {
    if (negInertiaAccumlator > 1) negInertiaAccumlator -= 1;
    else if (negInertiaAccumlator < -1) negInertiaAccumlator += 1;
    else negInertiaAccumlator = 0;

    if (quickStopAccumlator > 1) quickStopAccumlator -= 1;
    else if (quickStopAccumlator < -1) quickStopAccumlator += 1;
    else quickStopAccumlator = 0.0;
}

static double prevTurn = 0.0;
static double prevThrottle = 0.0;

static std::pair<double, double> cheesyDrive(double ithrottle, double iturn) {
    bool turnInPlace = false;
    double linearCmd = ithrottle;

    if (std::abs(ithrottle) < TIP_THRESHOLD && std::abs(iturn) > DRIVE_DEADBAND) {
        turnInPlace = true;
    }
    if (linearCmd - prevThrottle > DRIVE_SLEW) {
        linearCmd = prevThrottle + DRIVE_SLEW;
    } else if (linearCmd - prevThrottle < -(DRIVE_SLEW * 2)) {
        linearCmd = prevThrottle - (DRIVE_SLEW * 2);
    }

    double remappedTurn = _turnRemapping(iturn);

    double left = 0.0, right = 0.0;

    if (turnInPlace) {
        linearCmd *= TIP_CREEP_SCALE;
    }
    if (turnInPlace) {
        double angularCmd =
            (remappedTurn + negInertiaAccumlator)
            * CD_SENSITIVITY
            * TURN_IN_PLACE_GAIN;

        left  = linearCmd + angularCmd;
        right = linearCmd - angularCmd;

    } else {
        double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
        negInertiaAccumlator += negInertiaPower;

        double speed = std::abs(linearCmd);

        double turnGain =
            1.0 - (1.0 - MIN_TURN_GAIN) * speed;

        double angularCmd =
            turnGain * (remappedTurn + negInertiaAccumlator) * CD_SENSITIVITY
            - quickStopAccumlator;

        left  = linearCmd + angularCmd;
        right = linearCmd - angularCmd;

        _updateAccumulators();
    }

    left  = _clamp(left,  -1.0, 1.0);
    right = _clamp(right, -1.0, 1.0);

    prevTurn = iturn;
    prevThrottle = linearCmd;
    return {left, right};
}


pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::Rotation horizontalEnc(20);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);

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
                            &horizontal,
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
        std::pair<double, double> drive = cheesyDrive(throttle, turn);
        double left = _clamp(drive.first, -1.0, 1.0);
        double right = _clamp(drive.second, -1.0, 1.0);
        leftMotors.move(static_cast<int>(std::lround(left * JOYSTICK_SCALE)));
        rightMotors.move(static_cast<int>(std::lround(right * JOYSTICK_SCALE)));

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
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
            intake = 127;
            outtake = 40;
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
            intake = -127;
            outtake = -127;
        }
        else {
            midgoal.set_value(false);
        }

        // Send motor commands ONCE
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
