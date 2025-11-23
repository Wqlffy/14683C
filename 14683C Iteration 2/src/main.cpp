#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({15, -16, -17}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup rightMotors({-5, 6, 7}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::Motor intake(20, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::Motor indexer(-18, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
pros::Motor scoring(19, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees); //change the port later
pros::adi::Pneumatics tounge('A', false); //change port later
pros::adi::Pneumatics aligner('B', false); //change port later

pros::Imu imu(9);
pros::Rotation verticalEnc(11); 
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
    chassis.calibrate();     // calibrate sensors
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    scoring.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);


    pros::Task screenTask([&]() {
        while (true) {
            lemlib::Pose pose = chassis.getPose();
            pros::lcd::print(0, "X: %.2f", pose.x);     // x
            pros::lcd::print(1, "Y: %.2f", pose.y);     // y
            pros::lcd::print(2, "Th: %.2f", pose.theta); // heading

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
int slew(int current, int target, int accelStep, int decelStep) {
    int step = (std::abs(target) < std::abs(current)) ? decelStep : accelStep;

    if (current < target) {
        current += step;
        if (current > target) current = target;
    } else if (current > target) {
        current -= step;
        if (current < target) current = target;
    }
    return current;
}

int deadband(int v, int th = 5) {
    return (std::abs(v) < th) ? 0 : v;
}

int joystickCurve(int v) {
    double x = v / 127.0;
    return static_cast<int>(127.0 * x * x * x);
}


void opcontrol() {
    int fwdCmd = 0;
    int turnCmd = 0;
    while (true) {
        int leftY = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        leftY = joystickCurve(leftY);
        rightX = joystickCurve(rightX);
        
        leftY = deadband(leftY, 5);
        rightX = deadband(rightX, 5);

        fwdCmd = slew(fwdCmd, leftY, 4, 8);
        turnCmd = slew(turnCmd, rightX, 4, 8);
        chassis.arcade(fwdCmd, turnCmd);

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2)){
		    scoring.move(127);
            intake.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1)){
            scoring.move(-127);
            intake.move(-127);
		}
		else {
			intake.move(0);
            scoring.move(0);
		}

		if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2)){
			scoring.move(127);
            indexer.move(127);
		}
		else if (controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1)){
			scoring.move(127);
            indexer.move(-127);
		}
		else {
			scoring.move(0);
            indexer.move(0);
		}
        
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
			pros::delay(10);
		}