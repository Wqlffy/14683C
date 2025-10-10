#include "main.h"

#include <cmath>
#include <cstdint>

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
  config::roller.set_brake_mode(pros::MotorBrake::hold);
  config::loaders.set_brake_mode_all(pros::MotorBrake::hold);

  bool scoringActive = false;
  bool jamActive = false;
  std::uint32_t scoringEndAt = 0;
  std::uint32_t jamEndAt = 0;
  double rollerPrev = 0.0;
  double loaderPrev = 0.0;


  while (true) {
    const double rawThrottlePct = static_cast<double>(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) / 127.0 * 100.0;
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
    const std::uint32_t now = pros::c::millis();

    const double leftDrivePct = config::clamp(config::slew(leftTargetPct, leftPctPrev, kDriveSlewStepPct), -100.0, 100.0);
    const double rightDrivePct = config::clamp(config::slew(rightTargetPct, rightPctPrev, kDriveSlewStepPct), -100.0, 100.0);
    const double auxLeftPct = config::clamp(config::slew(leftTargetPct, auxLeftPrev, kAuxSlewStepPct), -100.0, 100.0);
    const double auxRightPct = config::clamp(config::slew(rightTargetPct, auxRightPrev, kAuxSlewStepPct), -100.0, 100.0);

    leftPctPrev = leftDrivePct;
    rightPctPrev = rightDrivePct;
    auxLeftPrev = auxLeftPct;
    auxRightPrev = auxRightPct;

    if (jamActive && now >= jamEndAt) {
      jamActive = false;
    }
    if (scoringActive && now >= scoringEndAt) {
      scoringActive = false;
    }

    if (!jamActive && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      jamActive = true;
      jamEndAt = now + config::scoring::JAM_CLEAR_MS;
      scoringActive = false;
    }

    double rollerTargetPct = 0.0;
    double loaderTargetPct = 0.0;
    bool scoringMode = false;

    if (jamActive) {
      rollerTargetPct = config::scoring::JAM_REVERSE_PCT * config::scoring::ROLLER_DIR;
      loaderTargetPct = config::scoring::JAM_REVERSE_PCT * config::scoring::LOADER_DIR;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      rollerTargetPct = config::scoring::PURGE_REVERSE_PCT * config::scoring::ROLLER_DIR;
      loaderTargetPct = config::scoring::PURGE_REVERSE_PCT * config::scoring::LOADER_DIR;
      scoringActive = false;
    } else {
      if (!scoringActive) {
        if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L2)) {
          scoringActive = true;
          scoringEndAt = now + config::scoring::SCORE_LONG_MS;
        } else if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_L1)) {
          scoringActive = true;
          scoringEndAt = now + config::scoring::SCORE_MID_MS;
        }
      }

      if (scoringActive) {
        scoringMode = true;
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        rollerTargetPct = config::scoring::INTAKE_FEED_PCT * config::scoring::ROLLER_DIR;
        loaderTargetPct = config::scoring::INTAKE_ASSIST_PCT * config::scoring::LOADER_DIR;
      }
    }

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

    if (scoringMode) {
      rollerPrev = 0.0;
      loaderPrev = 0.0;

      config::moveVelSurface(config::roller, config::scoring::VR_MPS, config::scoring::ROLLER_DIR,
                             config::scoring::ROLLER_MAX_RPM, config::scoring::ROLLER_DIAMETER_M);
      config::moveVelSurface(config::loaders, config::scoring::VS_MPS, config::scoring::LOADER_DIR,
                             config::scoring::LOADER_MAX_RPM, config::scoring::LOADER_DIAMETER_M);
    } else {
      const double rollerPct = config::clamp(config::slew(rollerTargetPct, rollerPrev, kAuxSlewStepPct), -100.0, 100.0);
      const double loaderPct = config::clamp(config::slew(loaderTargetPct, loaderPrev, kAuxSlewStepPct), -100.0, 100.0);

      rollerPrev = rollerPct;
      loaderPrev = loaderPct;

      config::spinPct(config::roller, rollerPct);
      config::spinPct(config::loaders, loaderPct);
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
