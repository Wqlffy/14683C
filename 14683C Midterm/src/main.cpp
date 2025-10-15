#include "main.h"

#include <cmath>
#include <cstdint>
#include <cstdio>

#include "auton_skills.hpp"
#include "config.hpp"
#include "drive_utils.hpp"
#include "lem_setup.hpp"
#include "telemetry.hpp"
#include "pros/llemu.hpp"
#include "pros/misc.hpp"

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

// );
}

void autonomous() {
  telemetry::init();
  telemetry::setEnabled(true);
  if (!lem_setup::calibrateAndInit()) {
    telemetry::logEvent("IMU_CAL_FAIL");
    return;
  }
  // lem_setup::setPoseMM(0.0, 0.0, 0.0);  // TODO: replace with actual start pose.
  // telemetry::logEvent("AUTO_START");
  // auton_skills::runSkillsAuton();
}

void opcontrol() {
  pros::Controller master(pros::E_CONTROLLER_MASTER);

  drive_utils::reset();
  config::leftDrive.set_brake_mode_all(pros::MotorBrake::brake);
  config::rightDrive.set_brake_mode_all(pros::MotorBrake::brake);
  config::auxleft.set_brake_mode(pros::MotorBrake::brake);
  config::auxright.set_brake_mode(pros::MotorBrake::brake);
  config::intakeGroup.set_brake_mode_all(pros::MotorBrake::hold);
  config::roller.set_brake_mode(pros::MotorBrake::hold);
  config::loader1.set_brake_mode(pros::MotorBrake::hold);
  config::loader2.set_brake_mode(pros::MotorBrake::hold);

  bool jamActive = false;
  std::uint32_t jamEndAt = 0;
  double rollerPrev = 0.0;
  double loaderPrev = 0.0;


  bool coord_overlay = false;

  auto updateOverlay = [&]() {
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    if (!coord_overlay) {
      pros::lcd::clear_line(0);
      pros::lcd::clear_line(1);
      pros::lcd::clear_line(2);
#pragma clang diagnostic pop
      return;
    }
    const auto pose = lem_setup::getPoseMM();
    char buffer[64];
#pragma clang diagnostic push
#pragma clang diagnostic ignored "-Wdeprecated-declarations"
    std::snprintf(buffer, sizeof(buffer), "X:%6.1fmm", pose.x);
    pros::lcd::set_text(0, buffer);
    std::snprintf(buffer, sizeof(buffer), "Y:%6.1fmm", pose.y);
    pros::lcd::set_text(1, buffer);
    std::snprintf(buffer, sizeof(buffer), "θ:%6.1f°", pose.thetaDeg);
    pros::lcd::set_text(2, buffer);
#pragma clang diagnostic pop
  };

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

    static double auxLeftPrev = 0.0;
    static double auxRightPrev = 0.0;
    const std::uint32_t now = pros::c::millis();

    drive_utils::update(leftTargetPct, rightTargetPct);
    const double leftDrivePct = drive_utils::leftOutput();
    const double rightDrivePct = drive_utils::rightOutput();
    const double auxLeftPct = config::clamp(config::slew(leftDrivePct, auxLeftPrev, kAuxSlewStepPct), -100.0, 100.0);
    const double auxRightPct = config::clamp(config::slew(rightDrivePct, auxRightPrev, kAuxSlewStepPct), -100.0, 100.0);

    auxLeftPrev = auxLeftPct;
    auxRightPrev = auxRightPct;

    if (jamActive && now >= jamEndAt) {
      jamActive = false;
    }

    if (!jamActive && master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_X)) {
      jamActive = true;
      jamEndAt = now + config::scoring::JAM_CLEAR_MS;
    }

    double rollerTargetPct = 0.0;
    double loaderTargetPct = 0.0;
    bool scoringMode = false;
    const bool longGoalHeld = master.get_digital(pros::E_CONTROLLER_DIGITAL_L2);
    const bool midGoalHeld = master.get_digital(pros::E_CONTROLLER_DIGITAL_L1);

    if (jamActive) {
      rollerTargetPct = config::scoring::JAM_REVERSE_PCT * config::scoring::ROLLER_DIR;
      loaderTargetPct = config::scoring::JAM_REVERSE_PCT;
    } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
      rollerTargetPct = config::scoring::PURGE_REVERSE_PCT * config::scoring::ROLLER_DIR;
      loaderTargetPct = config::scoring::PURGE_REVERSE_PCT;
    } else {
      if (longGoalHeld || midGoalHeld) {
        scoringMode = true;
      } else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
        rollerTargetPct = config::scoring::INTAKE_FEED_PCT * config::scoring::ROLLER_DIR;
        loaderTargetPct = config::scoring::INTAKE_ASSIST_PCT;
      }
    }

    auto applyCommand = [](pros::MotorGroup& group, double pct) {
      if (std::fabs(pct) < 1e-3) {
        group.brake();
      } else {
        group.move_voltage(config::pctToMillivolts(pct));
      }
    };

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

    auto setLoaderPct = [&](double pct1, double pct2) {
      config::loader1.move_voltage(config::pctToMillivolts(pct1));
      config::loader2.move_voltage(config::pctToMillivolts(pct2));
    };

    if (scoringMode) {
      rollerPrev = 0.0;
      loaderPrev = 0.0;

      config::moveVelSurface(config::roller, config::scoring::VR_MPS, config::scoring::ROLLER_DIR,
                             config::scoring::ROLLER_MAX_RPM, config::scoring::ROLLER_DIAMETER_M);

      double loader1Dir = config::scoring::LOADER1_DIR;
      double loader2Dir = config::scoring::LOADER2_DIR;
      if (longGoalHeld && !midGoalHeld) {
        loader1Dir *= -1.0;
      }

      config::moveVelSurface(config::loader1, config::scoring::VS_MPS, loader1Dir,
                             config::scoring::LOADER_MAX_RPM, config::scoring::LOADER_DIAMETER_M);
      config::moveVelSurface(config::loader2, config::scoring::VS_MPS, loader2Dir,
                             config::scoring::LOADER_MAX_RPM, config::scoring::LOADER_DIAMETER_M);
    } else {
      const double rollerPct = config::clamp(config::slew(rollerTargetPct, rollerPrev, kAuxSlewStepPct), -100.0, 100.0);
      const double loaderPct = config::clamp(config::slew(loaderTargetPct, loaderPrev, kAuxSlewStepPct), -100.0, 100.0);

      rollerPrev = rollerPct;
      loaderPrev = loaderPct;

      config::spinPct(config::roller, rollerPct);
      setLoaderPct(loaderPct * config::scoring::LOADER1_DIR,
                   loaderPct * config::scoring::LOADER2_DIR);
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

    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_UP)) {
      coord_overlay = !coord_overlay;
      telemetry::setEnabled(coord_overlay);
      updateOverlay();
    }
    if (master.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_DOWN)) {
      lem_setup::setPoseMM(0.0, 0.0, 0.0);
      updateOverlay();
    }

    updateOverlay();

    pros::delay(kLoopDelayMs);
  }
}
