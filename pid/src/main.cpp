#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({15, -16, -17}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::MotorGroup rightMotors({-5, 6, 7}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::Imu imu(9);
pros::Rotation verticalEnc(11); 
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, 1);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              12.4, // length between left and right wheels (middle of wheels)
                              lemlib::Omniwheel::NEW_325, 
                              450,
                              2 // traction wheel is usually 2
);

lemlib::ControllerSettings linearController(5.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            18, // derivative gain (kD)
                                            10, // anti windup
                                            1.0, // small error range, in inches
                                            250, // small error range timeout, in milliseconds
                                            3.0, // large error range, in inches
                                            2500, // large error range timeout, in milliseconds
                                            75 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(3.8,// proportional gain (kP)
                                             0, // integral gain (kI)
                                             22, // derivative gain (kD)
                                             5 , // anti windup
                                             2.0, // small error range, in degrees
                                             220, // small error range timeout, in milliseconds
                                             7.0, // large error range, in degrees
                                             2500, // large error range timeout, in milliseconds
                                             55 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(8, 12, 1.02);

lemlib::ExpoDriveCurve steerCurve(10, 15, 1.02);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);


void initialize() {
    pros::lcd::initialize();
    chassis.calibrate();

    pros::Task screenTask([&]() {
        while (true) {
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            pros::delay(50);
        }
    });
}

void disabled() {}

void competition_initialize() {}

void autonomous() {
	chassis.setPose(0, 0, 0);
	chassis.moveToPoint(0,15,99999);
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