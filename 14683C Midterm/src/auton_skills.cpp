#include "auton_skills.hpp"

#include <array>
#include "config.hpp"
#include "autopark.hpp"
#include "barrier.hpp"
#include "lem_setup.hpp"
#include "mech_intake.hpp"
#include "mech_scoring.hpp"
#include "nav_params.hpp"
#include "path_correction.hpp"
#include "pros/rtos.hpp"
#include "smoothing.hpp"
#include "telemetry.hpp"
#include "wallsnap.hpp"

namespace {
constexpr int kLoopDelayMs = 10;

struct Waypoint {
  double x_mm;
  double y_mm;
  double theta_deg;
  int profile_max_speed_mmps;
  int profile_max_accel_mmps2;
  bool snap_x;
  bool snap_y;
  int timeout_ms;
  int retry_max;
  enum class Action { None, ScoreMid, ScoreLong, Barrier, AutoPark } action;
};

// TODO: Populate real waypoints.
constexpr Waypoint makeWaypoint() {
  return Waypoint{0.0, 0.0, 0.0, nav_params::DRIVE_MAX_SPEED_MMPS, nav_params::DRIVE_MAX_ACCEL_MMPS2,
                  false, false, 2000, 0, Waypoint::Action::None};
}

constexpr std::array<Waypoint, 3> kRoute = {makeWaypoint(), makeWaypoint(), makeWaypoint()};
}

namespace auton_skills {

enum class State {
  StartPose,
  Transit1,
  Align1,
  ScoreMid1,
  Snap1,
  Transit2,
  Align2,
  ScoreLong1,
  Snap2,
  Transit3,
  Align3,
  ScoreMid2,
  Snap3,
  BarrierPush,
  BackOut,
  AutoPark,
  Abort,
  Done
};

void runSkillsAuton() {
  telemetry::init();
  telemetry::setEnabled(true);

  if (!lem_setup::imuReady()) {
    telemetry::logEvent("IMU_NOT_READY");
    return;
  }

  State state = State::StartPose;
  std::uint32_t state_start = pros::c::millis();
  std::uint32_t global_start = state_start;
  bool command_sent = false;
  int wp_index = 0;

  auto resetCommand = [&]() {
    command_sent = false;
    state_start = pros::c::millis();
  };

  auto advance = [&](State next) {
    state = next;
    resetCommand();
  };

  while (true) {
    const std::uint32_t now = pros::c::millis();
    wallsnap::updateDistanceFilters();
    mech_intake::intakeTaskUpdate();
    mech_scoring::update();
    barrier::update();
    autopark::update();
    telemetry::update();

    if (now - global_start > nav_params::SKILLS_GLOBAL_TIMEOUT_MS && state != State::AutoPark &&
        state != State::Done) {
      advance(State::AutoPark);
    }

    switch (state) {
      case State::StartPose: {
        // TODO: set actual start pose
        lem_setup::setPoseMM(0.0, 0.0, 0.0);
        advance(State::Transit1);
        break;
      }
      case State::Transit1: {
        if (!command_sent) {
          const auto& wp = kRoute[0];
          smoothing::moveToProfile(wp.x_mm, wp.y_mm, wp.theta_deg, wp.timeout_ms, wp.profile_max_speed_mmps,
                                   wp.profile_max_accel_mmps2, true);
          command_sent = true;
        }
        if (now - state_start > kRoute[0].timeout_ms) {
          advance(State::Align1);
        }
        break;
      }
      case State::Align1: {
        advance(State::ScoreMid1);
        break;
      }
      case State::ScoreMid1: {
        if (!command_sent) {
          mech_scoring::scoreMidStart();
          command_sent = true;
        }
        if (!mech_scoring::scoringActive()) {
          advance(State::Snap1);
        }
        break;
      }
      case State::Snap1: {
        wallsnap::wallSnapXY();
        wallsnap::zeroHeadingToNearestCardinalIfSnapped();
        advance(State::Transit2);
        break;
      }
      case State::Transit2: {
        if (!command_sent) {
          const auto& wp = kRoute[1];
          smoothing::moveToProfile(wp.x_mm, wp.y_mm, wp.theta_deg, wp.timeout_ms, wp.profile_max_speed_mmps,
                                   wp.profile_max_accel_mmps2, true);
          command_sent = true;
        }
        if (now - state_start > kRoute[1].timeout_ms) {
          advance(State::Align2);
        }
        break;
      }
      case State::Align2: {
        advance(State::ScoreLong1);
        break;
      }
      case State::ScoreLong1: {
        if (!command_sent) {
          mech_scoring::scoreLongStart();
          command_sent = true;
        }
        if (!mech_scoring::scoringActive()) {
          advance(State::Snap2);
        }
        break;
      }
      case State::Snap2: {
        wallsnap::wallSnapXY();
        wallsnap::zeroHeadingToNearestCardinalIfSnapped();
        advance(State::Transit3);
        break;
      }
      case State::Transit3: {
        if (!command_sent) {
          const auto& wp = kRoute[2];
          smoothing::moveToProfile(wp.x_mm, wp.y_mm, wp.theta_deg, wp.timeout_ms, wp.profile_max_speed_mmps,
                                   wp.profile_max_accel_mmps2, true);
          command_sent = true;
        }
        if (now - state_start > kRoute[2].timeout_ms) {
          advance(State::Align3);
        }
        break;
      }
      case State::Align3: {
        advance(State::ScoreMid2);
        break;
      }
      case State::ScoreMid2: {
        if (!command_sent) {
          mech_scoring::scoreMidStart();
          command_sent = true;
        }
        if (!mech_scoring::scoringActive()) {
          advance(State::Snap3);
        }
        break;
      }
      case State::Snap3: {
        wallsnap::wallSnapXY();
        wallsnap::zeroHeadingToNearestCardinalIfSnapped();
        advance(State::BarrierPush);
        break;
      }
      case State::BarrierPush: {
        if (!command_sent) {
          barrier::pushForwardStart(1500);
          command_sent = true;
        }
        if (!barrier::pushActive()) {
          advance(State::BackOut);
        }
        break;
      }
      case State::BackOut: {
        if (!command_sent) {
          smoothing::moveToProfile(lem_setup::getPoseMM().x - 200.0, lem_setup::getPoseMM().y - 200.0,
                                   lem_setup::getPoseMM().thetaDeg, 1000, nav_params::DRIVE_MAX_SPEED_MMPS,
                                   nav_params::DRIVE_MAX_ACCEL_MMPS2, true);
          command_sent = true;
        }
        if (now - state_start > 1000) {
          advance(State::AutoPark);
        }
        break;
      }
      case State::AutoPark: {
        if (!command_sent) {
          autopark::start();
          command_sent = true;
        }
        if (!autopark::active()) {
          advance(State::Done);
        }
        break;
      }
      case State::Abort: {
        mech_scoring::update();
        config::leftDrive.brake();
        config::rightDrive.brake();
        return;
      }
      case State::Done: {
        telemetry::logEvent("SKILLS_DONE");
        return;
      }
    }

    pros::delay(kLoopDelayMs);
  }
}

}
