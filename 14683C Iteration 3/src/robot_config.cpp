#include "robot_config.hpp"
#include "pros/adi.hpp"

pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({20, 19, 18}, pros::MotorGearset::blue);

pros::Motor leftMotor1(-1, pros::MotorGearset::blue);
pros::Motor leftMotor2(-2, pros::MotorGearset::blue);
pros::Motor leftMotor3(-3, pros::MotorGearset::blue);
pros::Motor rightMotor1(20, pros::MotorGearset::blue);
pros::Motor rightMotor2(19, pros::MotorGearset::blue);
pros::Motor rightMotor3(18, pros::MotorGearset::blue);

pros::Motor intakeMotor(9, pros::MotorGearset::blue);
pros::Motor outtakeMotor(17, pros::MotorGearset::blue);

pros::Imu imu(21);

pros::Distance leftDist(1);
pros::Distance rightDist(10);

pros::adi::Pneumatics matchloader('A', false);
pros::adi::Pneumatics midgoal('B', false);
pros::adi::Pneumatics wing('C', false);
pros::adi::Pneumatics middescore('D', false);
