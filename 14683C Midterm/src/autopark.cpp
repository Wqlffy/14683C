#include "autopark.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"
#include "lem_setup.hpp"
#include "nav_params.hpp"
#include "pros/rtos.hpp"

namespace autopark {
namespace {
  bool active_flag = false;
  std::uint32_t end_time = 0;
  double target_heading = 0.0;
}

void start() {
  active_flag = true;
  end_time = pros::c::millis() + nav_params::AUTOPARK_TIMEOUT_MS;
  target_heading = lem_setup::imu().get_heading();
}

bool active() { return active_flag; }

void update() {
  if (!active_flag) return;
  const std::uint32_t now = pros::c::millis();
  if (now >= end_time) {
    active_flag = false;
    config::leftDrive.brake();
    config::rightDrive.brake();
    return;
  }

  const double dist = config::distBack.get_distance();
  if (dist <= 0) return;
  const double error = dist - nav_params::AUTOPARK_TARGET_DIST_MM;
  if (std::fabs(error) < 10.0) {
    active_flag = false;
    config::leftDrive.brake();
    config::rightDrive.brake();
    return;
  }

  const double heading = lem_setup::imu().get_heading();
  double heading_error = target_heading - heading;
  while (heading_error > 180.0) heading_error -= 360.0;
  while (heading_error < -180.0) heading_error += 360.0;
  double correction = heading_error * 2.0;
  if (correction > 30.0) correction = 30.0;
  if (correction < -30.0) correction = -30.0;
  const double drive_pct = std::clamp(error * 0.2, -50.0, 50.0);
  config::leftDrive.move_voltage(config::pctToMillivolts(drive_pct - correction));
  config::rightDrive.move_voltage(config::pctToMillivolts(drive_pct + correction));
}

}
