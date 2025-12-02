#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "liblvgl/core/lv_obj_tree.h"
#include "liblvgl/display/lv_display.h"
#include "liblvgl/misc/lv_types.h"
#include "liblvgl/widgets/image/lv_image.h"
#include "pros/adi.hpp"
#include "pros/llemu.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/apix.h" // IWYU pragma: keep
#include <cmath>
#include <utility>
#include "autonomous.hpp"


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({15, -16, -17}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup rightMotors({-5, 6, 7}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::Motor intake(20, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::Motor indexer(-18, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
pros::Motor scoring(19, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
pros::adi::Pneumatics tongue('A', false); 
pros::adi::Pneumatics aligner('B', false); 

pros::Imu imu(9);
pros::Rotation verticalEnc(11); 
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, 0.5);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              15.25, // length between left and right wheels (middle of wheels)
                              lemlib::Omniwheel::NEW_325, 
                              450,
                              2 // traction wheel is usually 2
);

lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            0, // derivative gain (kD)
                                            0, // anti windup
                                            0, // small error range, in inches
                                            0, // small error range timeout, in milliseconds
                                            0, // large error range, in inches
                                            0, // large error range timeout, in milliseconds
                                            0 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(2,// proportional gain (kP)
                                             0, // integral gain (kI)
                                             0, // derivative gain (kD)
                                             0, // anti windup
                                             0, // small error range, in degrees
                                             0, // small error range timeout, in milliseconds
                                             0, // large error range, in degrees
                                             0, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(8, 12, 1.02);

lemlib::ExpoDriveCurve steerCurve(10, 15, 1.02);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

extern int selectedAuton;
extern const lv_image_dsc_t img_14683C;
void build_auton_selector();
void build_base_screen();
extern void auton_controller_task(void* param);
pros::Task autonSelTask(auton_controller_task, nullptr, "Auton Selector");


void initialize() {
    chassis.calibrate();
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    scoring.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);

    build_base_screen();
}

void disabled() {}

void competition_initialize() {
    build_base_screen();
    build_auton_selector();
    
    while (pros::competition::is_disabled()) {
        pros::delay(20);
    }

    lv_obj_clean(lv_screen_active());

    lv_obj_t* bg = lv_image_create(lv_screen_active());
    lv_image_set_src(bg, &img_14683C);
    lv_obj_align(bg, LV_ALIGN_CENTER, 0, 0);
}

extern int selectedAuton;
void autonomous() {
    switch (selectedAuton) {
        case 1:
            auton_red_left();
            break;
        case 2:
            auton_red_right();
            break;
        case 3:
            auton_blue_left();
            break;
        case 4:
            auton_blue_right();
            break;
        default:
            break;
    }
}

static int deadbandInt(int val, int threshold) {
    return (std::abs(val) < threshold) ? 0 : val;
}

constexpr double PI = 3.141592653589793;
constexpr double CD_TURN_NONLINEARITY = 0.65;
constexpr double CD_NEG_INERTIA_SCALAR = 2.5;  // strength of 'flick' boost
constexpr double CD_SENSITIVITY = 1.2;
constexpr double DRIVE_DEADBAND = 0.03;
constexpr double DRIVE_SLEW_UP = 0.05;
constexpr double DRIVE_SLEW_DOWN = 0.10;

static double quickStopAccumlator  = 0.0;
static double negInertiaAccumlator = 0.0;
static double prevTurn = 0.0;
static double prevThrottle = 0.0;

static double turnRemapping(double iturn) {
    double denominator = std::sin(PI / 2.0 * CD_TURN_NONLINEARITY);
    if (denominator == 0.0) return iturn; 
    double first = std::sin(PI / 2.0 * CD_TURN_NONLINEARITY * iturn) / denominator;
    return std::sin(PI / 2.0 * CD_TURN_NONLINEARITY * first) / denominator;
}

static void updateAccumulators() {
    if (negInertiaAccumlator > 1.0) {
        negInertiaAccumlator -= 1.0;
    } 
    else if (negInertiaAccumlator < -1.0) {
        negInertiaAccumlator += 1.0;
    } 
    else {
        negInertiaAccumlator = 0.0;
    }

    if (quickStopAccumlator > 1.0) {
        quickStopAccumlator -= 1.0;
    } 
    else if (quickStopAccumlator < -1.0) {
        quickStopAccumlator += 1.0;
    } 
    else {
        quickStopAccumlator = 0.0;
    }
}

static std::pair<double, double> cheesyArcade(double ithrottle, double iturn) {
    bool turnInPlace = false;
    double linearCmd = ithrottle;

    if (std::fabs(ithrottle) < DRIVE_DEADBAND && std::fabs(iturn) > DRIVE_DEADBAND) {
        linearCmd  = 0.0;
        turnInPlace = true;
    }
    else {
        double delta = ithrottle - prevThrottle;
        if (delta > DRIVE_SLEW_UP) {
            linearCmd = prevThrottle + DRIVE_SLEW_UP;
        } 
        else if (delta < -DRIVE_SLEW_DOWN) {
            linearCmd = prevThrottle - DRIVE_SLEW_DOWN;
        } 
        else {
            linearCmd = ithrottle;
        }
    }

    double remappedTurn = turnRemapping(iturn);

    double forwardOut;
    double turnOut;

    if (turnInPlace) {
        double x = remappedTurn;
        forwardOut = 0.0;
        turnOut = x * std::fabs(x); 
    } 
    else {
        double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
        negInertiaAccumlator += negInertiaPower;

        double speedFactor = std::fabs(linearCmd);
        if (speedFactor < 0.5) {
            speedFactor = 0.5;
        }
        
        double angularCmd = speedFactor * (remappedTurn + negInertiaAccumlator) * CD_SENSITIVITY - quickStopAccumlator;

        forwardOut = linearCmd;
        turnOut = angularCmd;

        updateAccumulators();
    }

    prevTurn = iturn;
    prevThrottle = linearCmd;

    forwardOut = std::fmax(-1.0, std::fmin(1.0, forwardOut));
    turnOut = std::fmax(-1.0, std::fmin(1.0, turnOut));

    return {forwardOut, turnOut};
}

void opcontrol() {
    chassis.cancelAllMotions();

    bool flagStateTongue = false;
    bool flagStateAligner = false;
    while (true) {

        int rawFwd = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rawTurn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        rawFwd = deadbandInt(rawFwd, 5);
        rawTurn = deadbandInt(rawTurn, 5);

        double ithrottle = rawFwd  / 127.0;
        double iturn = rawTurn / 127.0;

        auto driveCmd = cheesyArcade(ithrottle, iturn);
        double fwdCmd = driveCmd.first;
        double turnCmd = driveCmd.second;

        int fwdPower = static_cast<int>(fwdCmd  * 127.0);
        int turnPower  = static_cast<int>(turnCmd * 127.0);

        chassis.arcade(fwdPower, turnPower);

        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (r1 && r2) {
            intake.move(127);
            scoring.move(-127);
            indexer.move(0);
        }
        else if (l1 && l2) {
            intake.move(0);
            scoring.move(-127);
            indexer.move(0);
        }
        else {
            if (r2) {
                intake.move(127);
                indexer.move(-127);
            }
            else if (r1) {
                intake.move(-127);
            }
            else {
                intake.move(0);
            }

            if (l2) {
                scoring.move(127);
                indexer.move(127);
            }
            else if (l1) {
                scoring.move(127);
                indexer.move(-127);
            }
            else {
                scoring.move(0);
                indexer.move(0);
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            flagStateTongue = !flagStateTongue;
            if (flagStateTongue) 
                tongue.extend();
            else 
                tongue.retract();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            flagStateAligner = !flagStateAligner;
            if (flagStateAligner) 
                aligner.extend();
            else 
                aligner.retract();
        }

        pros::delay(10);
    }
}
