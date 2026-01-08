#include "robot_config.hpp"
#include "pros/adi.hpp"

pros::MotorGroup leftMotors({-11, -12, -13}, pros::MotorGearset::blue);
pros::MotorGroup rightMotors({20, 19, 18}, pros::MotorGearset::blue);

pros::Motor intakeMotor(14, pros::MotorGearset::blue);
pros::Motor outtakeMotor(-3, pros::MotorGearset::blue);

pros::Imu imu(21);

pros::Distance leftDist(1);
pros::Distance rightDist(10);

pros::adi::Pneumatics matchloader('A', false);
pros::adi::Pneumatics midgoal('B', false);
pros::adi::Pneumatics wing('C', false);
pros::adi::Pneumatics middescore('D', false);
