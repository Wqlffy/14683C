#include "barrier.hpp"

#include "config.hpp"
#include "nav_params.hpp"
#include "pros/rtos.hpp"

namespace barrier {
namespace {
  bool active = false;
  bool contact = false;
  std::uint32_t end_time = 0;
  std::uint32_t start_time = 0;
}

void pushForwardStart(int maxMs) {
  active = true;
  contact = false;
  start_time = pros::c::millis();
  end_time = start_time + maxMs;
  config::leftDrive.move_voltage(config::pctToMillivolts(100));
  config::rightDrive.move_voltage(config::pctToMillivolts(100));
}

bool pushActive() { return active; }

bool stoppedByContact() { return contact; }

void update() {
  if (!active) return;
  const std::uint32_t now = pros::c::millis();
  if (now >= end_time) {
    active = false;
    config::leftDrive.brake();
    config::rightDrive.brake();
    return;
  }
  if (now - start_time < nav_params::BARRIER_MIN_PUSH_MS) return;

  const double left_current = config::leftDrive.get_current_draw();
  const double right_current = config::rightDrive.get_current_draw();
  if (left_current > nav_params::BARRIER_CONTACT_CURRENT_A ||
      right_current > nav_params::BARRIER_CONTACT_CURRENT_A) {
    contact = true;
    active = false;
    config::leftDrive.brake();
    config::rightDrive.brake();
  }
}

}

