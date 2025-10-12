#include "path_correction.hpp"

#include <cmath>

#include "lem_setup.hpp"
#include "nav_params.hpp"

namespace path_correction {
namespace {
  bool enabled = false;
  double target_heading = 0.0;
  double error_integral = 0.0;
  double last_error = 0.0;
}

void reset(double targetHeadingDeg) {
  target_heading = targetHeadingDeg;
  error_integral = 0.0;
  last_error = 0.0;
}

void setTarget(double targetHeadingDeg) {
  target_heading = targetHeadingDeg;
}

void enable(bool on) {
  enabled = on;
  if (!enabled) {
    error_integral = 0.0;
    last_error = 0.0;
  }
}

bool active() { return enabled; }

double compute(double currentHeadingDeg, double forwardCmdPct) {
  if (!enabled || std::fabs(forwardCmdPct) < nav_params::PATH_MIN_FWD_THRESHOLD_PCT) {
    error_integral = 0.0;
    last_error = 0.0;
    return 0.0;
  }

  double heading = currentHeadingDeg;
  if (!std::isfinite(heading)) {
    heading = lem_setup::imu().get_heading();
  }

  double error = target_heading - heading;
  while (error > 180.0) error -= 360.0;
  while (error < -180.0) error += 360.0;

  if (std::fabs(error) < nav_params::PATH_HEADING_EPS_DEG) {
    error_integral = 0.0;
    last_error = error;
    return 0.0;
  }

  error_integral += error;
  const double derivative = error - last_error;
  last_error = error;

  double output = nav_params::PATH_KP * error + nav_params::PATH_KI * error_integral + nav_params::PATH_KD * derivative;
  if (output > nav_params::PATH_MAX_OUTPUT_PCT) output = nav_params::PATH_MAX_OUTPUT_PCT;
  if (output < -nav_params::PATH_MAX_OUTPUT_PCT) output = -nav_params::PATH_MAX_OUTPUT_PCT;
  return output;
}

}
