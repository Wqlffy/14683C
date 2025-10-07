#pragma once

#include <cmath>

#include "pros/abstract_motor.hpp"
#include "pros/motor_group.hpp"
#include "pros/motors.hpp"


namespace config {

inline pros::MotorGroup leftDrive({-4, 2}, pros::MotorGears::blue, pros::MotorUnits::degrees);
inline pros::MotorGroup rightDrive({8, -9}, pros::MotorGears::blue, pros::MotorUnits::degrees);
inline pros::MotorGroup intakeGroup({6, 7}, pros::MotorGears::green, pros::MotorUnits::degrees);
inline pros::Motor auxleft(1, pros::MotorGears::green, pros::MotorUnits::degrees);
inline pros::Motor auxright(-10, pros::MotorGears::green, pros::MotorUnits::degrees);

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

}
