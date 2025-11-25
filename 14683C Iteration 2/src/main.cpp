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


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({15, -16, -17}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup rightMotors({-5, 6, 7}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::Motor intake(20, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::Motor indexer(-18, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
pros::Motor scoring(19, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
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

lemlib::ExpoDriveCurve throttleCurve(8, // joystick deadband out of 127
                                     12, // minimum output where drivetrain will move out of 127
                                     1.02 // expo curve gain
);

lemlib::ExpoDriveCurve steerCurve(10, // joystick deadband out of 127
                                  15, // minimum output where drivetrain will move out of 127
                                  1.02 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

extern int selectedAuton;
extern const lv_image_dsc_t img_14683C;
void build_auton_selector();


void initialize() {
    chassis.calibrate();
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_BRAKE);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    scoring.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
}

void disabled() {}

void competition_initialize() {
    build_auton_selector();
    
    while (pros::competition::is_disabled()) {
        pros::delay(20);
    }

    lv_obj_clean(lv_screen_active());

    lv_obj_t* bg = lv_image_create(lv_screen_active());
    lv_image_set_src(bg, &img_14683C);
    lv_obj_align(bg, LV_ALIGN_CENTER, 0, 0);
}

// get a path used for pure pursuit
// this needs to be put outside a function
// ASSET(example_txt); // '.' replaced with "_" to make c++ happy

// void autonomous() {
// 	chassis.setPose(0, 0, 0); 
// 	chassis.moveToPoint(0, 10, 999999); // tuning Linear PID
	
// 	// chassis.turnToHeading(180, 999999); //tuning Angular PID
// 	// chassis.swingToHeading(45, lemlib::DriveSide::RIGHT, 750); // fast turns (speed)
// 	// chassis.turnToHeading(45, 750); // slower turns (accuracy)

// 	// async false makes the code finish the timeout before going to the next line
// }

void autonomous() {
    switch (selectedAuton) {
        case 1: auton_red_left();        break;
        case 2: auton_red_right();       break;
        case 3: auton_blue_left();       break;
        case 4: auton_blue_right();      break;
        case 5: auton_red_left_awp();    break;
        case 6: auton_red_right_awp();   break;
        case 7: auton_blue_left_awp();   break;
        case 8: auton_blue_right_awp();  break;
        case 9: auton_skills();          break;
        default:
            break;
    }
}

int deadband(int v, int th = 5) {
    return (std::abs(v) < th) ? 0 : v;
}

int slew(int current, int target, int accelStep, int decelStep) {
    if ((current > 0 && target < 0) || (current < 0 && target > 0)) {
        target = 0;
    }
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

void opcontrol() {
    build_auton_selector(); // allow changing auton while in opcontrol for testing
    int fwdCmd = 0;
    int turnCmd = 0;
     while (true) {
        int leftY  = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rightX = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
        leftY  = deadband(leftY, 5);
        rightX = deadband(rightX, 5);
        leftY  = static_cast<int>(leftY  * 0.7); // raise cap if to slow 0.8 or 0.9
        rightX = static_cast<int>(rightX * 0.7);
        double f = leftY  / 127.0;
        double t = rightX / 127.0;
        double turnScale = 1.0 - 0.6 * std::abs(f);
        if (turnScale < 0.4) turnScale = 0.4;
        t *= turnScale;

        leftY  = static_cast<int>(f * 127.0);
        rightX = static_cast<int>(t * 127.0);

        fwdCmd  = slew(fwdCmd,  leftY,  2, 4); // if too slow increase accel 3, decel,6
        turnCmd = slew(turnCmd, rightX, 2, 4);
        chassis.curvature(fwdCmd, turnCmd, false);

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