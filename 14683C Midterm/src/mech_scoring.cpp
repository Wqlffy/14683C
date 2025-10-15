#include "mech_scoring.hpp"

#include <algorithm>
#include <cmath>

#include "config.hpp"
#include "nav_params.hpp"
#include "pros/rtos.hpp"

namespace {
int purge_pct = 0;
bool scoring_state = false;
std::uint32_t action_end = 0;
}

namespace mech_scoring {
namespace {
  void stopAll() {
    config::roller.brake();
    config::loader1.brake();
    config::loader2.brake();
  }

  void spinPct(int pct) {
    config::roller.move_voltage(config::pctToMillivolts(pct * config::scoring::ROLLER_DIR));
    config::loader1.move_voltage(config::pctToMillivolts(pct * config::scoring::LOADER1_DIR));
    config::loader2.move_voltage(config::pctToMillivolts(pct * config::scoring::LOADER2_DIR));
  }

  void runVelocity(double roller_mps, double loader_mps) {
    const int roller_rpm = static_cast<int>((roller_mps * 60.0) /
                                            (config::scoring::ROLLER_DIAMETER_M * config::kPi));
    const int loader_rpm = static_cast<int>((loader_mps * 60.0) /
                                            (config::scoring::LOADER_DIAMETER_M * config::kPi));
    const int roller_cmd = std::clamp(roller_rpm * config::scoring::ROLLER_DIR,
                                      -config::scoring::ROLLER_MAX_RPM, config::scoring::ROLLER_MAX_RPM);
    const int loader1_cmd = std::clamp(loader_rpm * config::scoring::LOADER1_DIR,
                                       -config::scoring::LOADER_MAX_RPM, config::scoring::LOADER_MAX_RPM);
    const int loader2_cmd = std::clamp(loader_rpm * config::scoring::LOADER2_DIR,
                                       -config::scoring::LOADER_MAX_RPM, config::scoring::LOADER_MAX_RPM);
    config::roller.move_velocity(roller_cmd);
    config::loader1.move_velocity(loader1_cmd);
    config::loader2.move_velocity(loader2_cmd);
  }
}

void scoreMidStart() {
  scoring_state = true;
  purge_pct = 0;
  action_end = 0;
}

void scoreLongStart() {
  scoring_state = true;
  purge_pct = 0;
  action_end = 0;
}

void purgeStart(int pct) {
  scoring_state = false;
  purge_pct = pct;
  action_end = pros::c::millis() + nav_params::PURGE_TIMEOUT_MS;
}

void jamClearStart(int pct, int ms) {
  scoring_state = false;
  purge_pct = pct;
  action_end = pros::c::millis() + ms;
}

bool scoringActive() { return scoring_state; }

void update() {
  const std::uint32_t now = pros::c::millis();
  if (scoring_state) {
    runVelocity(config::scoring::VR_MPS, config::scoring::VS_MPS);
    return;
  }

  if (purge_pct != 0) {
    spinPct(purge_pct);
    if (now >= action_end) {
      purge_pct = 0;
      stopAll();
    }
    return;
  }

  stopAll();
}

}
