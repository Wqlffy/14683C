#include "autons.hpp"

#include <cmath>

#include "auton_recovery.hpp"
#include "robot_config.hpp"
#include "skills_segments.hpp"

namespace {
double nearest_cardinal_deg(double heading_deg) {
    const int idx =
        static_cast<int>(std::floor((heading_deg + 45.0) / 90.0)) % 4;
    return static_cast<double>((idx < 0) ? (idx + 4) : idx) * 90.0;
}

void wall_reset_between_segment_1() {
    constexpr double kWallTargetMm = 595.376;
    AutonRecovery::WallResetParams p{};
    p.faceHeadingDeg = nearest_cardinal_deg(imu.get_heading());
    p.leftTargetMm = kWallTargetMm;
    p.rightTargetMm = kWallTargetMm;
    p.squareTimeoutMs = 800;
    p.setTimeoutMs = 900;
    p.tolMm = 10;
    p.maxTurn = AutonRecovery::Tuning::maxTurn;
    p.maxFwd = AutonRecovery::Tuning::maxFwd;
    p.trySquare = true;
    AutonRecovery::wallReset(p);
}

void wall_reset_between_segment_2() {
    constexpr double kWallTargetMm = 595.376;
    AutonRecovery::WallResetParams p{};
    p.faceHeadingDeg = nearest_cardinal_deg(imu.get_heading());
    p.leftTargetMm = kWallTargetMm;
    p.rightTargetMm = kWallTargetMm;
    p.squareTimeoutMs = 800;
    p.setTimeoutMs = 900;
    p.tolMm = 10;
    p.maxTurn = AutonRecovery::Tuning::maxTurn;
    p.maxFwd = AutonRecovery::Tuning::maxFwd;
    p.trySquare = true;
    AutonRecovery::wallReset(p);
}
}

void auton_skills() {
    skills1();
    wall_reset_between_segment_1();
    skills2();
    wall_reset_between_segment_2();
    skills3();
}

