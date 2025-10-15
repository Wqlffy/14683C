#include "drive_utils.hpp"

#include <cmath>

#include "config.hpp"
#include "nav_params.hpp"

namespace drive_utils {
namespace {
  double left_prev = 0.0;
  double right_prev = 0.0;
  double drive_slew = nav_params::DRIVE_SLEW_STEP_PCT;
  double turn_slew = nav_params::TURN_SLEW_STEP_PCT;
  bool brake_mode = true;

  double applySlew(double target, double prev, double step) {
    const double delta = target - prev;
    if (delta > step) return prev + step;
    if (delta < -step) return prev - step;
    return target;
  }
}

void reset() {
  left_prev = 0.0;
  right_prev = 0.0;
}

void setDriveMode(bool braking) {
  if (brake_mode == braking) return;
  brake_mode = braking;
  const auto mode = braking ? pros::MotorBrake::brake : pros::MotorBrake::coast;
  config::leftDrive.set_brake_mode_all(mode);
  config::rightDrive.set_brake_mode_all(mode);
}

void update(double leftTargetPct, double rightTargetPct) {
  const double fwd = (leftTargetPct + rightTargetPct) * 0.5;
  const double turn = (leftTargetPct - rightTargetPct) * 0.5;

  double turn_ratio = std::clamp(std::fabs(turn) / 100.0, 0.0, 1.0);
  double speed_ratio = std::clamp(std::fabs(fwd) / 100.0, 0.0, 1.0);

  const double throttle_scale = 1.0 - 0.4 * turn_ratio;
  double scaled_left = fwd * throttle_scale + turn;
  double scaled_right = fwd * throttle_scale - turn;

  const double turn_step = nav_params::TURN_SLEW_STEP_PCT * (1.0 - turn_ratio * 0.5);
  const double drive_step = nav_params::DRIVE_SLEW_STEP_PCT * (1.0 - turn_ratio * 0.3);

  double left = applySlew(scaled_left, left_prev, drive_step);
  double right = applySlew(scaled_right, right_prev, drive_step);

  double diff = (left - right) * 0.5;
  const double max_turn = 60.0 - 20.0 * speed_ratio;
  if (diff > max_turn) {
    const double adjust = diff - max_turn;
    left -= adjust;
    right += adjust;
  } else if (diff < -max_turn) {
    const double adjust = -max_turn - diff;
    left += adjust;
    right -= adjust;
  }

  left_prev = std::clamp(left, -100.0, 100.0);
  right_prev = std::clamp(right, -100.0, 100.0);

  const bool braking = std::fabs(left_prev) < 1.0 && std::fabs(right_prev) < 1.0;
  setDriveMode(braking);

  config::leftDrive.move_voltage(config::pctToMillivolts(left_prev));
  config::rightDrive.move_voltage(config::pctToMillivolts(right_prev));
}

double leftOutput() { return left_prev; }
double rightOutput() { return right_prev; }

}

