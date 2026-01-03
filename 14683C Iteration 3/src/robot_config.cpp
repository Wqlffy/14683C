#include "robot_config.hpp"

pros::MotorGroup leftMotors({-1, -2, -3}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({20, 19, 18}, pros::MotorGearset::blue);

pros::Motor intakeMotor(9, pros::MotorGearset::blue);
pros::Motor outtakeMotor(17, pros::MotorGearset::blue);

pros::Imu imu(10);

pros::Distance leftDist(7);
pros::Distance rightDist(8);
