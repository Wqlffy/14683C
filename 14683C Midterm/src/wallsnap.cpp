#include "wallsnap.hpp"

#include <cmath>

#include "config.hpp"
#include "lem_setup.hpp"
#include "nav_params.hpp"

namespace wallsnap {
namespace {
  double filtered_right = 0.0;
  double filtered_back = 0.0;
  bool snapped_recently = false;

  bool validDistance(double mm) {
    return mm >= nav_params::DIST_MIN_MM && mm <= nav_params::DIST_MAX_MM;
  }

  double ema(double prev, double sample) {
    return nav_params::WALL_EMA_ALPHA * sample + (1.0 - nav_params::WALL_EMA_ALPHA) * prev;
  }

  double drivetrainSpeedMmps() {
    const double left_rpm = config::leftDrive.get_actual_velocity();
    const double right_rpm = config::rightDrive.get_actual_velocity();
    const double avg_rpm = (std::fabs(left_rpm) + std::fabs(right_rpm)) / 2.0;
    const double wheel_circ_mm = config::kPi * nav_params::DRIVE_WHEEL_DIAMETER_MM;
    return (avg_rpm * wheel_circ_mm) / 60.0;
  }

  bool canSnap() {
    return drivetrainSpeedMmps() <= nav_params::WALL_SNAP_MAX_SPEED_MMPS;
  }

  bool headingCardinal(double headingDeg) {
    headingDeg = std::fmod(headingDeg, 360.0);
    if (headingDeg < 0) headingDeg += 360.0;
    const double mod = std::fmod(headingDeg, 90.0);
    return mod <= nav_params::WALL_SNAP_HEADING_TOL_DEG ||
           mod >= 90.0 - nav_params::WALL_SNAP_HEADING_TOL_DEG;
  }
}

void updateDistanceFilters() {
  const double right = config::distRight.get_distance();
  const double back = config::distBack.get_distance();
  if (validDistance(right)) {
    filtered_right = ema(filtered_right, right);
  }
  if (validDistance(back)) {
    filtered_back = ema(filtered_back, back);
  }
}

bool wallSnapX() {
  snapped_recently = false;
  if (!canSnap()) return false;
  const auto pose = lem_setup::getPoseMM();
  const double heading = lem_setup::imu().get_heading();
  if (!headingCardinal(heading)) return false;
  if (!validDistance(filtered_right)) return false;
  const double new_x = filtered_right - nav_params::DIST_RIGHT_OFFSET_MM;
  lem_setup::setPoseMM(new_x, pose.y, heading);
  snapped_recently = true;
  return true;
}

bool wallSnapY() {
  snapped_recently = false;
  if (!canSnap()) return false;
  const auto pose = lem_setup::getPoseMM();
  const double heading = lem_setup::imu().get_heading();
  if (!headingCardinal(heading)) return false;
  if (!validDistance(filtered_back)) return false;
  const double new_y = filtered_back - nav_params::DIST_BACK_OFFSET_MM;
  lem_setup::setPoseMM(pose.x, new_y, heading);
  snapped_recently = true;
  return true;
}

bool wallSnapXY() {
  const bool x = wallSnapX();
  const bool y = wallSnapY();
  return x || y;
}

void zeroHeadingToNearestCardinalIfSnapped() {
  if (!snapped_recently) return;
  auto pose = lem_setup::getPoseMM();
  double heading = lem_setup::imu().get_heading();
  heading = std::round(heading / 90.0) * 90.0;
  lem_setup::setPoseMM(pose.x, pose.y, heading);
  snapped_recently = false;
}

double filteredRight() { return filtered_right; }

double filteredBack() { return filtered_back; }

}
