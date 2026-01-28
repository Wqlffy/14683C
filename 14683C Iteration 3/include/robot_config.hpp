#pragma once

#include "main.h"

// Hardware single source of truth. UI accesses these via extern.
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor leftMotor1;
extern pros::Motor leftMotor2;
extern pros::Motor leftMotor3;
extern pros::Motor rightMotor1;
extern pros::Motor rightMotor2;
extern pros::Motor rightMotor3;

extern pros::Motor leftFront;
extern pros::Motor leftMid;
extern pros::Motor leftBack;
extern pros::Motor rightFront;
extern pros::Motor rightMid;
extern pros::Motor rightBack;

extern pros::Motor intakeMotor;
extern pros::Motor outtakeMotor;

extern pros::Imu imu;

extern pros::Distance leftDist;
extern pros::Distance rightDist;

extern pros::adi::Pneumatics matchloader;
extern pros::adi::Pneumatics midgoal;
extern pros::adi::Pneumatics wing;
extern pros::adi::Pneumatics middescore;
