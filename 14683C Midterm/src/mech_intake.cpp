#include "mech_intake.hpp"

#include "config.hpp"
#include "nav_params.hpp"
#include "pros/rtos.hpp"

namespace mech_intake {
namespace {
  int target_pct = 0;
  int reverse_pct = 0;
  std::uint32_t reverse_end = 0;
  std::uint32_t forward_until = 0;
  bool active_flag = false;
}

void intakeOn(int pct) {
  target_pct = pct * nav_params::INTAKE_DIR;
  reverse_pct = 0;
  forward_until = 0;
  active_flag = true;
}

void intakeOff() {
  target_pct = 0;
  reverse_pct = 0;
  forward_until = 0;
  active_flag = false;
  config::intakeGroup.brake();
}

void intakeReverse(int pct, int ms) {
  reverse_pct = pct * nav_params::INTAKE_DIR;
  reverse_end = pros::c::millis() + ms;
}

void intakeOneBall(int pct, int ms) {
  intakeOn(pct);
  forward_until = pros::c::millis() + ms;
}

void intakeTaskUpdate() {
  const std::uint32_t now = pros::c::millis();

  if (reverse_pct != 0) {
    config::intakeGroup.move_voltage(config::pctToMillivolts(reverse_pct));
    if (now >= reverse_end) {
      reverse_pct = 0;
    }
    return;
  }

  if (!active_flag) {
    config::intakeGroup.brake();
    return;
  }

  config::intakeGroup.move_voltage(config::pctToMillivolts(target_pct));

  if (forward_until != 0 && now >= forward_until) {
    intakeOff();
  }
}

bool intakeActive() { return active_flag; }

}

