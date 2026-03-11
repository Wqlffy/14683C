#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep

pros::Controller master(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({-17, -12, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({20, 4, 18}, pros::MotorGearset::blue);

pros::Motor intakeMotor(15, pros::MotorGearset::blue);
pros::Motor outtakeMotor(3, pros::MotorGearset::blue);

pros::Imu imu(21);

pros::Distance leftDist(1);
pros::Distance rightDist(10);

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
    pros::lcd::initialize(); // initialize brain screen
    chassis.calibrate(); // calibrate sensors

    // the default rate is 50. however, if you need to change the rate, you
    // can do the following.
    // lemlib::bufferedStdout().setRate(...);
    // If you use bluetooth or a wired connection, you will want to have a rate of 10ms

    // for more information on how the formatting for the loggers
    // works, refer to the fmtlib docs

    // thread to for brain screen and position logging
    pros::Task screenTask([&]() {
        while (true) {
            // print robot location to the brain screen
            pros::lcd::print(0, "X: %f", chassis.getPose().x); // x
            pros::lcd::print(1, "Y: %f", chassis.getPose().y); // y
            pros::lcd::print(2, "Theta: %f", chassis.getPose().theta); // heading
            // log position telemetry
            lemlib::telemetrySink()->info("Chassis pose: {}", chassis.getPose());
            // delay to save resources
            pros::lcd::print(4, "Left Dist: %f", leftDist.get());
            pros::lcd::print(5, "Right Dist: %f", rightDist.get());
            pros::delay(50);
        }
    });
}

/**
 * Runs while the robot is disabled
 */
void disabled() {}

/**
 * runs after initialize if the robot is connected to field control
 */
void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

/**
 * Runs during auto
 *
 * This is an example autonomous routine which demonstrates a lot of the features LemLib has to offer
 */
void autonomous() {
//     // Move to x: 20 and y: 15, and face heading 90. Timeout set to 4000 ms
//     chassis.moveToPose(20, 15, 90, 4000);
//     // Move to x: 0 and y: 0 and face heading 270, going backwards. Timeout set to 4000ms
//     chassis.moveToPose(0, 0, 270, 4000, {.forwards = false});
//     // cancel the movement after it has traveled 10 inches
//     chassis.waitUntil(10);
//     chassis.cancelMotion();
//     // Turn to face the point x:45, y:-45. Timeout set to 1000
//     // dont turn faster than 60 (out of a maximum of 127)
//     chassis.turnToPoint(45, -45, 1000, {.maxSpeed = 60});
//     // Turn to face a direction of 90º. Timeout set to 1000
//     // will always be faster than 100 (out of a maximum of 127)
//     // also force it to turn clockwise, the long way around
//     chassis.turnToHeading(90, 1000, {.direction = AngularDirection::CW_CLOCKWISE, .minSpeed = 100});
//     // Follow the path in path.txt. Lookahead at 15, Timeout set to 4000
//     // following the path with the back of the robot (forwards = false)
//     // see line 116 to see how to define a path
//     chassis.follow(example_txt, 15, 4000, false);
//     // wait until the chassis has traveled 10 inches. Otherwise the code directly after
//     // the movement will run immediately
//     // Unless its another movement, in which case it will wait
//     chassis.waitUntil(10);
//     pros::lcd::print(4, "Traveled 10 inches during pure pursuit!");
//     // wait until the movement is done
//     chassis.waitUntilDone();
//     pros::lcd::print(4, "pure pursuit finished!");
}

/**
 * Runs in driver control
 */
void opcontrol() {
    // controller
    // loop to continuously update motors
    while (true) {
        // get joystick positions
        int leftY = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        // move the chassis with curvature drive
        chassis.arcade(leftY, rightX);
        // delay to save resources
        pros::delay(10);
    }
}