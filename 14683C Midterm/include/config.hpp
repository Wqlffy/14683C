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

inline float deadband(float x, float db = 0.05f) {
  return (std::fabs(x) < db) ? 0.0f : x;
}

inline float squareKeepSign(float x) {
  return (x >= 0.0f) ? (x * x) : -(x * x);
}

inline float clamp(float x, float lo, float hi) {
  if (x < lo) {
    return lo;
  }
  if (x > hi) {
    return hi;
  }
  return x;
}

inline int pctToMillivolts(float pct) {
  const float bounded = clamp(pct, -100.0f, 100.0f);
  return static_cast<int>(bounded * 120.0f);
}

}
