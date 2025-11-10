#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/motors.hpp"
#include "pros/optical.hpp"
#include "pros/rtos.hpp"


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({15, -16, -17}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup rightMotors({-5, 6, 7}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::Motor intake(20, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees); //change the port later
pros::Motor indexer(19, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees); //change the port later
pros::Motor scoring(-9, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees); //change the port later
pros::adi::Pneumatics tounge('A', false); //change port later
pros::adi::Pneumatics aligner('B', false); //change port later

pros::Imu imu(1); //change the port later
pros::Optical opticalSensor(8); //change the port later

pros::Rotation verticalEnc(6);
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, 0);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              18, // length between left and right wheels (middle of wheels)
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

lemlib::OdomSensors sensors(&vertical,
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
	opticalSensor.set_led_pwm(100); // set optical sensor led brightness

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
            pros::delay(50);
        }
    });
}


void disabled() {}

void competition_initialize() {}

// get a path used for pure pursuit
// this needs to be put outside a function
ASSET(example_txt); // '.' replaced with "_" to make c++ happy

void autonomous() {
	chassis.setPose(0, 0, 0); 
	chassis.moveToPoint(0, 10, 999999); // tuning Linear PID
	
	// chassis.turnToHeading(180, 999999); //tuning Angular PID
	// chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 750); // fast turns (speed)
	// chassis.turnToHeading(45, 750); // slower turns (accuracy)

	// async false makes the code finish the timeout before going to the next line
}

void opcontrol() {
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        chassis.arcade(leftY, rightX);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
			intake.move_velocity(600);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
			intake.move_velocity(-600);
		}
		else {
			intake.move_velocity(0);
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			scoring.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			indexer.move(-127);
		}
		else {
			indexer.move(0);
		}

		if (opticalSensor.get_hue() > 190 && opticalSensor.get_hue() < 220){ 
			// if blue is detected
			pros::delay(500);
			indexer.move(127);
		}else if (opticalSensor.get_hue() > 5 && opticalSensor.get_hue() < 30) {
			// if red is detected
			pros::delay(500);
			indexer.move(-127);
		}else {
			indexer.move(127);
		}

		bool flagStateTounge = false; // Variable to track the state of the Piston for the Tounge
		bool flagStateAligner = false; // Variable to track the state of the Piston for the Aligner

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)){
			if (flagStateTounge == false){
				tounge.extend();
				flagStateTounge = true;
			}else {
				tounge.retract();
				flagStateTounge = false;
			}
		}

		if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)){
			if (flagStateAligner == false){
				aligner.extend();
				flagStateAligner = true;
			}else {
				aligner.retract();
				flagStateAligner = false;
			}
		}
			pros::delay(20);
		}
    }