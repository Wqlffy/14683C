#include "main.h"

#include <cmath>

#include "config.hpp"

namespace {
constexpr int kLoopDelayMs = 10;
constexpr double kForwardPreferenceDeadband = 5.0;
constexpr double kCurvatureReduction = 0.6;
constexpr double kMinTurnGain = 0.2;
constexpr double kDriveSlewStepPct = 5.0;
constexpr double kAuxSlewStepPct = 5.0;
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
  config::auxleft.set_brake_mode(pros::MotorBrake::brake);
  config::auxright.set_brake_mode(pros::MotorBrake::brake);
  config::intakeGroup.set_brake_mode_all(pros::MotorBrake::hold);

  while (true) {
    const double rawThrottlePct = -static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127.0 * 100.0;
    const double rawWheelPct = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) / 127.0 * 100.0;

    const double shapedThrottle = config::squareInputPct(config::applyDeadbandPct(rawThrottlePct, 5.0));
    const double shapedWheel = config::squareInputPct(config::applyDeadbandPct(rawWheelPct, 5.0));

    double throttle = config::clamp(shapedThrottle, -100.0, 100.0);
    double wheel = config::clamp(shapedWheel, -100.0, 100.0);

    if (std::fabs(throttle) < kForwardPreferenceDeadband) {
      throttle = 0.0;
    }

    double leftTargetPct = 0.0;
    double rightTargetPct = 0.0;

    if (std::fabs(wheel) >= std::fabs(throttle)) {
      leftTargetPct = config::clamp(wheel, -100.0, 100.0);
      rightTargetPct = config::clamp(-wheel, -100.0, 100.0);
    } else {
      const double speedScalar = std::fabs(throttle) / 100.0;
      double turnGain = 1.0 - kCurvatureReduction * speedScalar;
      if (turnGain < kMinTurnGain) {
        turnGain = kMinTurnGain;
      }
      const double adjustedWheel = config::clamp(wheel * turnGain, -100.0, 100.0);
      leftTargetPct = config::clamp(throttle + adjustedWheel, -100.0, 100.0);
      rightTargetPct = config::clamp(throttle - adjustedWheel, -100.0, 100.0);
    }

    static double leftPctPrev = 0.0;
    static double rightPctPrev = 0.0;
    static double auxLeftPrev = 0.0;
    static double auxRightPrev = 0.0;

    const double leftDrivePct = config::clamp(config::slew(leftTargetPct, leftPctPrev, kDriveSlewStepPct), -100.0, 100.0);
    const double rightDrivePct = config::clamp(config::slew(rightTargetPct, rightPctPrev, kDriveSlewStepPct), -100.0, 100.0);
    const double auxLeftPct = config::clamp(config::slew(leftTargetPct, auxLeftPrev, kAuxSlewStepPct), -100.0, 100.0);
    const double auxRightPct = config::clamp(config::slew(rightTargetPct, auxRightPrev, kAuxSlewStepPct), -100.0, 100.0);

    leftPctPrev = leftDrivePct;
    rightPctPrev = rightDrivePct;
    auxLeftPrev = auxLeftPct;
    auxRightPrev = auxRightPct;

    auto applyCommand = [](pros::MotorGroup& group, double pct) {
      if (std::fabs(pct) < 1e-3) {
        group.brake();
      } else {
        group.move_voltage(config::pctToMillivolts(pct));
      }
    };

    applyCommand(config::leftDrive, leftDrivePct);
    applyCommand(config::rightDrive, rightDrivePct);

    if (std::fabs(auxLeftPct) < 1e-3) {
      config::auxleft.brake();
    } else {
      config::auxleft.move_voltage(config::pctToMillivolts(auxLeftPct));
    }

    if (std::fabs(auxRightPct) < 1e-3) {
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
      config::intakeGroup.move_voltage(config::pctToMillivolts(static_cast<double>(intakePct)));
    }

    pros::delay(kLoopDelayMs);
  }
}
