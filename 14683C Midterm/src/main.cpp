#include "main.h"
#include <cmath>
#include "config.hpp"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/chassis.hpp" // IWYU pragma: keep
#include "pros/llemu.hpp" // IWYU pragma: keep

namespace {
constexpr int kLoopDelayMs = 10;
constexpr float kForwardPreferenceDeadband = 5.0f;
}

void disabled() {

}

void competition_initialize() {
// lemlib::Drivetrain drivetrain(&config::leftDrive, config::auxleft,
//                               &config::rightDrive, config::auxright,
//                               10, // 10 inch track width
//                               lemlib::Omniwheel::NEW_4, // using new 4" omnis
//                               600, // drivetrain rpm is 600
//                               2 // horizontal drift is 2 (for now)
// );
}

void autonomous() {

}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  config::leftDrive.set_brake_mode_all(pros::MotorBrake::brake);
  config::rightDrive.set_brake_mode_all(pros::MotorBrake::brake);
  config::auxleft.set_brake_mode(pros::MotorBrake::hold);
  config::auxright.set_brake_mode(pros::MotorBrake::hold);
  config::intakeGroup.set_brake_mode_all(pros::MotorBrake::hold);

  while (true) {
    const float rawFwd = static_cast<float>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127.0f;
    const float rawTurn = static_cast<float>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) / 127.0f;
    float fwd = config::squareKeepSign(config::deadband(rawFwd));
    float turn = config::squareKeepSign(config::deadband(rawTurn));

    float forwardPct = config::clamp(fwd * 100.0f, -100.0f, 100.0f);
    const float turnPct = config::clamp(turn * 100.0f, -100.0f, 100.0f);

    if (std::fabs(forwardPct) < kForwardPreferenceDeadband) {
      forwardPct = 0.0f;
    }

    float leftDrivePct = 0.0f;
    float rightDrivePct = 0.0f;
    float auxLeftPct = 0.0f;
    float auxRightPct = 0.0f;

    if (forwardPct != 0.0f) {
      leftDrivePct = forwardPct;
      rightDrivePct = forwardPct;
      auxLeftPct = forwardPct;
      auxRightPct = forwardPct;
    } else if (turnPct < 0.0f) {
      leftDrivePct = turnPct;
      auxLeftPct = turnPct;
    } else if (turnPct > 0.0f) {
      rightDrivePct = turnPct;
      auxRightPct = turnPct;
    }

    if (leftDrivePct == 0.0f) {
      config::leftDrive.brake();
    } else {
      config::leftDrive.move_voltage(config::pctToMillivolts(leftDrivePct));
    }

    if (rightDrivePct == 0.0f) {
      config::rightDrive.brake();
    } else {
      config::rightDrive.move_voltage(config::pctToMillivolts(rightDrivePct));
    }

    if (auxLeftPct == 0.0f) {
      config::auxleft.brake();
    } else {
      config::auxleft.move_voltage(config::pctToMillivolts(auxLeftPct));
    }

    if (auxRightPct == 0.0f) {
      config::auxright.brake();
    } else {
      config::auxright.move_voltage(config::pctToMillivolts(auxRightPct));
    }

    int intakePct = 0;
    if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
      intakePct = 100;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      intakePct = -100;
    }

    if (intakePct == 0) {
      config::intakeGroup.brake();
    } else {
      config::intakeGroup.move_voltage(config::pctToMillivolts(static_cast<float>(intakePct)));
    }

    pros::delay(kLoopDelayMs);
  }
}
