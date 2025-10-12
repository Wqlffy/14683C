#include "smoothing.hpp"

#include <algorithm>
#include <cmath>

#include "lem_setup.hpp"
#include "nav_params.hpp"
#include "lemlib/api.hpp"

namespace smoothing {
namespace {
  double slew_step_pct = 5.0;
  bool precision_mode = false;
  bool clamp_pid = false;

  float toChassisSpeed(int mmps) {
    if (mmps <= 0 || nav_params::DRIVE_MAX_SPEED_MMPS <= 0) {
      return 127.0f;
    }
    double ratio = static_cast<double>(mmps) / static_cast<double>(nav_params::DRIVE_MAX_SPEED_MMPS);
    if (ratio > 1.0) ratio = 1.0;
    if (ratio < 0.0) ratio = 0.0;
    return static_cast<float>(ratio * 127.0);
  }
}

void setAutonSlewStepPct(double pct) {
  slew_step_pct = std::fabs(pct);
}

double applySlew(double targetPct, double prevPct) {
  const double delta = targetPct - prevPct;
  if (delta > slew_step_pct) {
    return prevPct + slew_step_pct;
  }
  if (delta < -slew_step_pct) {
    return prevPct - slew_step_pct;
  }
  return targetPct;
}

void precisionModeOn() { precision_mode = true; }
void precisionModeOff() { precision_mode = false; }
bool precisionMode() { return precision_mode; }

void moveToProfile(double x_mm, double y_mm, double heading_deg, int timeout_ms, int max_speed_mmps,
                   int max_accel_mmps2, bool async) {
  auto& chassis = lem_setup::chassis();
  const int speed = precision_mode ? nav_params::DRIVE_PRECISION_SPEED_MMPS : max_speed_mmps;
  lemlib::MoveToPointParams params;
  params.maxSpeed = toChassisSpeed(speed);
  chassis.moveToPoint(static_cast<float>(x_mm / 25.4), static_cast<float>(y_mm / 25.4), timeout_ms, params, async);
  if (!std::isnan(heading_deg)) {
    lemlib::TurnToHeadingParams heading_params;
    heading_params.maxSpeed = static_cast<int>(std::round(toChassisSpeed(speed)));
    chassis.turnToHeading(static_cast<float>(heading_deg), timeout_ms, heading_params, async);
  }
}

void turnToProfile(double x_mm, double y_mm, int timeout_ms, int max_speed_mmps,
                   int max_accel_mmps2, bool async) {
  auto& chassis = lem_setup::chassis();
  const int speed = precision_mode ? nav_params::DRIVE_PRECISION_SPEED_MMPS : max_speed_mmps;
  lemlib::TurnToPointParams params;
  params.maxSpeed = static_cast<int>(std::round(toChassisSpeed(speed)));
  chassis.turnToPoint(static_cast<float>(x_mm / 25.4), static_cast<float>(y_mm / 25.4), timeout_ms, params, async);
}

void clampPIDOutputNearTarget(bool enabled) {
  clamp_pid = enabled;
  lem_setup::chassis().setBrakeMode(enabled ? pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_HOLD
                                            : pros::motor_brake_mode_e_t::E_MOTOR_BRAKE_COAST);
}

}
