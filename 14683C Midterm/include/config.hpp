#pragma once

#include <cmath>
#include <cstdint>

#include "pros/abstract_motor.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"
#include "pros/distance.hpp"

namespace config {

inline constexpr double kPi = 3.14159265358979323846;

inline pros::MotorGroup leftDrive({-4, 2}, pros::MotorGears::blue, pros::MotorUnits::degrees);
inline pros::MotorGroup rightDrive({8, -9}, pros::MotorGears::blue, pros::MotorUnits::degrees);
inline pros::MotorGroup intakeGroup({6, -7}, pros::MotorGears::green, pros::MotorUnits::degrees);
inline pros::Motor auxleft(1, pros::MotorGears::green, pros::MotorUnits::degrees);
inline pros::Motor auxright(-10, pros::MotorGears::green, pros::MotorUnits::degrees);
inline pros::Motor roller(11, pros::MotorGears::red, pros::MotorUnits::degrees);
inline pros::MotorGroup loaders({12, 16}, pros::MotorGears::green, pros::MotorUnits::degrees);
inline pros::Distance distRight(14);
inline pros::Distance distBack(15);
inline pros::IMU IMU(3);

namespace scoring {
// TODO: verify direction signs after a quick field test (flip sign if motion is reversed).
constexpr double ROLLER_DIR = 1.0;
constexpr double LOADER_DIR = 1.0;

// TODO: replace with measured roller/loader diameters (meters) for accurate velocity control.
constexpr double ROLLER_DIAMETER_M = 0.0508;
constexpr double LOADER_DIAMETER_M = 0.0508;

constexpr double ROLLER_MAX_RPM = 100.0;
constexpr double LOADER_MAX_RPM = 200.0;  

// TODO: tune feed velocity and downstream ratio on the practice field.
constexpr double VR_MPS = 0.50;
constexpr double K_DOWNSTREAM = 1.20;
constexpr double VS_MPS = VR_MPS * K_DOWNSTREAM;

// Durations (ms) for system actions – adjust to taste.
constexpr std::uint32_t JAM_CLEAR_MS = 300;
constexpr std::uint32_t SCORE_MID_MS = 400;
constexpr std::uint32_t SCORE_LONG_MS = 650;

// Percentage commands used outside velocity mode.
constexpr double INTAKE_FEED_PCT = 80.0;
constexpr double INTAKE_ASSIST_PCT = 15.0;
constexpr double JAM_REVERSE_PCT = -70.0;
constexpr double PURGE_REVERSE_PCT = -100.0;
}  // namespace scoring

inline double clamp(double x, double lo, double hi) {
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

inline double applyDeadbandPct(double xPct, double dbPct) {
  return (std::fabs(xPct) <= dbPct) ? 0.0 : xPct;
}

inline double squareInputPct(double xPct) {
  const double sign = (xPct >= 0.0) ? 1.0 : -1.0;
  const double normalized = xPct / 100.0;
  const double squared = normalized * normalized;
  return sign * squared * 100.0;
}

inline double slew(double target, double current, double step) {
  const double delta = target - current;
  if (delta > step) {
    return current + step;
  }
  if (delta < -step) {
    return current - step;
  }
  return target;
}

inline int pctToMillivolts(double pct) {
  const double bounded = clamp(pct, -100.0, 100.0);
  return static_cast<int>(bounded * 120.0);
}

inline void spinPct(pros::Motor& motor, double pct) {
  if (std::fabs(pct) < 1e-3) {
    motor.brake();
  } else {
    motor.move_voltage(pctToMillivolts(pct));
  }
}

inline void spinPct(pros::MotorGroup& group, double pct) {
  if (std::fabs(pct) < 1e-3) {
    group.brake();
  } else {
    group.move_voltage(pctToMillivolts(pct));
  }
}

inline double surfaceToRpm(double surfaceMps, double diameterM) {
  if (diameterM <= 0.0) {
    return 0.0;
  }
  return (surfaceMps * 60.0) / (kPi * diameterM);
}

inline double clampRpm(double rpm, double maxRpm) {
  return clamp(rpm, -maxRpm, maxRpm);
}

inline void moveVelSurface(pros::Motor& motor, double surfaceMps, double dir, double maxRpm, double diameterM) {
  const double rpm = clampRpm(surfaceToRpm(surfaceMps, diameterM) * dir, maxRpm);
  motor.move_velocity(static_cast<int>(rpm));
}

inline void moveVelSurface(pros::MotorGroup& group, double surfaceMps, double dir, double maxRpm, double diameterM) {
  const double rpm = clampRpm(surfaceToRpm(surfaceMps, diameterM) * dir, maxRpm);
  group.move_velocity(static_cast<int>(rpm));
}

}  // namespace config
