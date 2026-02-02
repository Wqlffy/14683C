#include "robot_config.hpp"
#include "pros/adi.hpp"

pros::MotorGroup leftMotors({-11, -12, -14}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({20, 19, 18}, pros::MotorGearset::blue);

// Individual drivetrain motors for per-motor telemetry.
pros::Motor leftFront(-11, pros::MotorGearset::blue);
pros::Motor leftMid(-12, pros::MotorGearset::blue);
pros::Motor leftBack(-14, pros::MotorGearset::blue);
pros::Motor rightFront(20, pros::MotorGearset::blue);
pros::Motor rightMid(19, pros::MotorGearset::blue);
pros::Motor rightBack(18, pros::MotorGearset::blue);

pros::Motor intakeMotor(15, pros::MotorGearset::blue);
pros::Motor outtakeMotor(3, pros::MotorGearset::blue);

pros::Imu imu(21);

pros::Distance leftDist(1);
pros::Distance rightDist(10);
pros::Distance frontDist(16);

pros::adi::Pneumatics matchloader('A', false);
pros::adi::Pneumatics midgoal('C', false);
pros::adi::Pneumatics wing('E', false);
pros::adi::Pneumatics middescore('D', false);
