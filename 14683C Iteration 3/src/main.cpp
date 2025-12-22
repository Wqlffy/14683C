#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

#include <algorithm>
#include <cmath>
#include <utility>

constexpr double PI = 3.14159265358979323846;
constexpr double DRIVE_DEADBAND = 0.03;
constexpr double DRIVE_SLEW = 0.10;
constexpr double CD_TURN_NONLINEARITY = 0.45;
constexpr double CD_NEG_INERTIA_SCALAR = 2.3;
constexpr double CD_SENSITIVITY = 1.25;
constexpr double JOYSTICK_SCALE = 127.0;

constexpr double MIN_TURN_GAIN = 0.85;
constexpr double TURN_IN_PLACE_GAIN = 1.15;

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

    if (std::abs(ithrottle) < DRIVE_DEADBAND && std::abs(iturn) > DRIVE_DEADBAND) {
        linearCmd = 0.0;
        turnInPlace = true;
    } else if (ithrottle - prevThrottle > DRIVE_SLEW) {
        linearCmd = prevThrottle + DRIVE_SLEW;
    } else if (ithrottle - prevThrottle < -(DRIVE_SLEW * 2)) {
        linearCmd = prevThrottle - (DRIVE_SLEW * 2);
    }

    double remappedTurn = _turnRemapping(iturn);

    double left = 0.0, right = 0.0;

    if (turnInPlace) {
        left  = remappedTurn * TURN_IN_PLACE_GAIN;
        right = -remappedTurn * TURN_IN_PLACE_GAIN;
    } else {
        double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
        negInertiaAccumlator += negInertiaPower;

        double speed = std::abs(linearCmd);

        // Keep strong turning even at speed (this is the "uncap" fix)
        double turnGain = 1.0 - (1.0 - MIN_TURN_GAIN) * speed; // 1.0 -> 0.85

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


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-5, 4, -3}, pros::MotorGearset::blue); 
pros::MotorGroup rightMotors({6, -9, 7}, pros::MotorGearset::blue);

pros::Imu imu(10);

pros::Rotation horizontalEnc(20);
lemlib::TrackingWheel horizontal(&horizontalEnc, lemlib::Omniwheel::NEW_275, -5.75);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors, 
                              10, 
                              lemlib::Omniwheel::NEW_4, 
                              360, 
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
    pros::lcd::initialize();
    chassis.calibrate(); 

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::delay(50);
        }
    });
}
void disabled() {}
void competition_initialize() {}
void autonomous() {
}


void opcontrol() {
    while (true) {
        double throttle = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / JOYSTICK_SCALE;
        double turn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / JOYSTICK_SCALE;
        std::pair<double, double> drive = cheesyDrive(throttle, turn);
        double left = _clamp(drive.first, -1.0, 1.0);
        double right = _clamp(drive.second, -1.0, 1.0);
        leftMotors.move(static_cast<int>(std::lround(left * JOYSTICK_SCALE)));
        rightMotors.move(static_cast<int>(std::lround(right * JOYSTICK_SCALE)));
        pros::delay(10);
    }
}
