#pragma once

#include "main.h"

// Hardware single source of truth. UI accesses these via extern.
extern pros::MotorGroup leftMotors;
extern pros::MotorGroup rightMotors;

extern pros::Motor intakeMotor;
extern pros::Motor outtakeMotor;

extern pros::Imu imu;

extern pros::Distance leftDist;
extern pros::Distance rightDist;
