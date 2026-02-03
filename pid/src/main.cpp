#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/distance.hpp"

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


void initialize() {
    pros::lcd::initialize();
    pros::lcd::print(0, "Calibrating IMU...");
    chassis.calibrate();
    while (imu.is_calibrating()) {
        pros::delay(20);
    }

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            pros::lcd::print(3, "Distance: %f", distanceSensorL.get()); // distance sensor
            pros::lcd::print(4, "Distance: %f", distanceSensorR.get()); // distance sensor
            pros::lcd::print(5, "Distance: %f", distanceSensorF.get()); // distance sensor
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
