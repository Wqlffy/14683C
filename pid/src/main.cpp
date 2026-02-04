#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"
#include <cmath> // FIX: for std::isfinite, std::sin/cos

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-11, -12, -14}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::MotorGroup rightMotors({20, 19, 18}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::Imu imu(21);

pros::Distance distanceSensorL(1);
pros::Distance distanceSensorR(10);
pros::Distance distanceSensorF(16);

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

lemlib::OdomSensors sensors(nullptr, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(8, 12, 1.02);

lemlib::ExpoDriveCurve steerCurve(10, 15, 1.02);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

// FIX: odometry safety + debug
#ifndef ODOM_DEBUG
#define ODOM_DEBUG 0
#endif

#if ODOM_DEBUG
#define ODOM_LOG(...) printf(__VA_ARGS__)
#else
#define ODOM_LOG(...) do { } while (0)
#endif

namespace {
constexpr double kPi = 3.14159265358979323846;
constexpr double kDistEps = 1e-4;   // inches
constexpr double kThetaEps = 1e-3;  // degrees

struct OdomPose {
    double x = 0.0;
    double y = 0.0;
    double theta = 0.0; // degrees
};

pros::Mutex odomMutex;
OdomPose odomPose;

// FIX: normalize heading to [-180, 180)
double normalizeAngle(double degrees) {
    while (degrees >= 180.0) degrees -= 360.0;
    while (degrees < -180.0) degrees += 360.0;
    return degrees;
}

// FIX: finite guard
bool isFinite(double v) {
    return std::isfinite(v);
}

double degToRad(double deg) { return deg * (kPi / 180.0); }

OdomPose getOdomPose() {
    OdomPose copy;
    odomMutex.take();
    copy = odomPose;
    odomMutex.give();
    return copy;
}

void setOdomPose(const OdomPose& pose) {
    odomMutex.take();
    odomPose = pose;
    odomMutex.give();
}

void odomTaskFn() {
    double prevLeftDeg = leftMotors.get_position();
    double prevRightDeg = rightMotors.get_position();
    double prevHeadingDeg = normalizeAngle(imu.get_rotation());

    if (!isFinite(prevLeftDeg) || !isFinite(prevRightDeg) || !isFinite(prevHeadingDeg)) {
        ODOM_LOG("[ODOM] init: non-finite sensor read, zeroing pose\n");
        prevLeftDeg = 0.0;
        prevRightDeg = 0.0;
        prevHeadingDeg = 0.0;
    }

    OdomPose pose;
    pose.x = 0.0;
    pose.y = 0.0;
    pose.theta = prevHeadingDeg;
    setOdomPose(pose);

    while (true) {
        if (imu.is_calibrating()) {
            // FIX: wait for IMU readiness
            ODOM_LOG("[ODOM] guard: IMU calibrating, skipping update\n");
            pros::delay(10);
            continue;
        }

        const double leftDeg = leftMotors.get_position();
        const double rightDeg = rightMotors.get_position();
        const double imuDeg = imu.get_rotation();

        if (!isFinite(leftDeg) || !isFinite(rightDeg) || !isFinite(imuDeg)) {
            // FIX: skip update if any sensor read is invalid
            ODOM_LOG("[ODOM] guard: non-finite sensor read, skipping update\n");
            pros::delay(10);
            continue;
        }

        const double wheelCircumference = kPi * drivetrain.wheelDiameter;
        const double deltaLeft = (leftDeg - prevLeftDeg) / 360.0 * wheelCircumference;
        const double deltaRight = (rightDeg - prevRightDeg) / 360.0 * wheelCircumference;
        const double deltaDist = (deltaLeft + deltaRight) * 0.5;

        double headingDeg = normalizeAngle(imuDeg);
        double deltaThetaDeg = normalizeAngle(headingDeg - prevHeadingDeg);

        double localDx = 0.0;
        double localDy = 0.0;

        if (std::fabs(deltaDist) < kDistEps) {
            // FIX: avoid curvature when distance is ~0
            localDx = 0.0;
            localDy = 0.0;
            ODOM_LOG("[ODOM] guard: deltaDist ~= 0, straight/rotate-only update\n");
        } else if (std::fabs(deltaThetaDeg) < kThetaEps) {
            // FIX: straight-line approximation
            localDx = deltaDist;
            localDy = 0.0;
        } else {
            // FIX: arc-based update without curvature division by ~0
            const double deltaThetaRad = degToRad(deltaThetaDeg);
            const double radius = deltaDist / deltaThetaRad;
            localDx = radius * std::sin(deltaThetaRad);
            localDy = radius * (1.0 - std::cos(deltaThetaRad));
        }

        const double midThetaDeg = normalizeAngle(prevHeadingDeg + deltaThetaDeg * 0.5);
        const double midThetaRad = degToRad(midThetaDeg);
        const double globalDx = localDx * std::cos(midThetaRad) - localDy * std::sin(midThetaRad);
        const double globalDy = localDx * std::sin(midThetaRad) + localDy * std::cos(midThetaRad);

        pose = getOdomPose();
        pose.x += globalDx;
        pose.y += globalDy;
        pose.theta = normalizeAngle(headingDeg);

        if (!isFinite(pose.x) || !isFinite(pose.y) || !isFinite(pose.theta)) {
            // FIX: self-heal NaN/Inf pose
            ODOM_LOG("[ODOM] guard: pose non-finite, resetting to (0,0,heading)\n");
            pose.x = 0.0;
            pose.y = 0.0;
            pose.theta = normalizeAngle(headingDeg);
        }

        setOdomPose(pose);
        chassis.setPose(static_cast<float>(pose.x), static_cast<float>(pose.y),
                        static_cast<float>(pose.theta), false);

        ODOM_LOG("[ODOM] dL=%f dR=%f dDist=%f dThetaDeg=%f thetaDeg=%f x=%f y=%f\n",
                 deltaLeft, deltaRight, deltaDist, deltaThetaDeg, pose.theta, pose.x, pose.y);

        prevLeftDeg = leftDeg;
        prevRightDeg = rightDeg;
        prevHeadingDeg = headingDeg;

        pros::delay(10);
    }
}
pros::Task* odomTask = nullptr; // FIX: start after IMU calibration
pros::Task* lemlibOdomTask = nullptr;
}


void initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "Calibrating IMU...");
    // FIX: explicitly calibrate IMU before odom starts
    imu.reset();
    while (imu.is_calibrating()) pros::delay(20);
    chassis.calibrate(false); // FIX: IMU already calibrated above
    // Prefer LemLib odometry updates. Keep custom odom available for debug only.
#ifndef USE_CUSTOM_ODOM
#define USE_CUSTOM_ODOM 0
#endif
    if (USE_CUSTOM_ODOM && odomTask == nullptr) {
        odomTask = new pros::Task(odomTaskFn, "Odom Task");
    }
    if (lemlibOdomTask == nullptr) {
        lemlibOdomTask = new pros::Task([] {
            while (true) {
                chassis.update();
                pros::delay(10);
            }
        }, "LemLib Odom");
    }

    pros::Task screenTask([&]() {
        while (true) {
            auto pose = chassis.getPose();
            pros::lcd::print(0, "X: %f", pose.x); // x
            pros::lcd::print(1, "Y: %f", pose.y); // y
            pros::lcd::print(2, "Theta: %f", pose.theta); // heading (deg)
            pros::lcd::print(3, "Distance LEFT: %d mm\n", distanceSensorL.get_distance());
            pros::lcd::print(4, "Distance RIGHT: %d mm\n", distanceSensorR.get_distance());
            pros::lcd::print(5, "Distance FRONT: %d mm\n", distanceSensorF.get_distance());
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
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
        // get joystick positions
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}
