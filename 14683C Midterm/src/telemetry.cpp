#include "telemetry.hpp"

#include <cstdio>

#include "config.hpp"
#include "lem_setup.hpp"
#include "nav_params.hpp"
#include "wallsnap.hpp"

namespace telemetry {
namespace {
  bool enabled = true;
  std::uint32_t last_tick = 0;
}

void init() {
  last_tick = 0;
}

void setEnabled(bool on) { enabled = on; }

void update() {
  if (!enabled) return;
  const std::uint32_t now = pros::c::millis();
  if (now - last_tick < nav_params::TELEMETRY_INTERVAL_MS) return;
  last_tick = now;

  const auto pose = lem_setup::getPoseMM();
  const double dist_right = wallsnap::filteredRight();
  const double dist_back = wallsnap::filteredBack();
  std::printf("POSE %.1f %.1f %.1f | Dist %.1f %.1f\n", pose.x, pose.y, pose.thetaDeg, dist_right,
              dist_back);
}

void logEvent(const char* tag) {
  std::printf("EVENT: %s @%lu\n", tag, static_cast<unsigned long>(pros::c::millis()));
}

}

