#pragma once

#include <cstdint>
#include <functional>

#include "main.h"

namespace AutonRecovery {
namespace Tuning {
// Distance sensor validity + filter.
constexpr double distMinMm = 50.0;
constexpr double distMaxMm = 2000.0;
constexpr double distAlpha = 0.25;
constexpr int distUpdateMs = 20;

// Stall detection.
constexpr double stallCmdThresh = 0.20;
constexpr double stallVelThreshRpm = 5.0;
constexpr int stallTimeMs = 250;

// Wall alignment control.
constexpr double squareKp = 0.0025;
constexpr double setDistKp = 0.0025;
constexpr double headingKp = 0.010;
constexpr double maxTurn = 0.35;
constexpr double maxFwd = 0.35;
constexpr int squareStableSamples = 8;
constexpr int setStableSamples = 8;

// Encoder drive control.
constexpr double driveKp = 0.05;
constexpr double driveTolIn = 0.5;
constexpr int driveStableSamples = 4;

// Motor group sizes (update if drive changes).
constexpr int leftDriveCount = 3;
constexpr int rightDriveCount = 3;

// Brake behavior during stabilize.
constexpr pros::motor_brake_mode_e_t holdBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
constexpr pros::motor_brake_mode_e_t defaultBrakeMode = pros::E_MOTOR_BRAKE_BRAKE;
}

struct FilteredDistances {
    double leftMmFilt = 0.0;
    double rightMmFilt = 0.0;
    bool leftValid = false;
    bool rightValid = false;
};

enum class RecoverResult : uint8_t { SKIPPED_NO_WALL, DONE, TIMEOUT };

bool distValidMm(int mm);
double filt(double prev, double now, double alpha);
const FilteredDistances& get_filtered_distances();

bool detectStall(double driveCmd, int dtMs);
void stabilize(int holdMs = 150);

bool wallSquare(int timeoutMs = 800, int diffMmTol = 8,
                double maxTurn = Tuning::maxTurn);
bool wallSetDistance(double targetMm, int timeoutMs = 900, int tolMm = 10,
                     double maxFwd = Tuning::maxFwd,
                     double faceHeadingDeg = 0.0);

bool snapRightToWall(double targetMm,
                     double faceHeadingDeg,
                     int timeoutMs = 800,
                     int tolMm = 10,
                     double maxFwd = 0.25);

bool snapLeftToWall(double targetMm,
                    double faceHeadingDeg,
                    int timeoutMs = 800,
                    int tolMm = 10,
                    double maxFwd = 0.25);

bool driveDistanceHeading(double inches,
                          double faceHeadingDeg,
                          int timeoutMs,
                          double maxFwd = Tuning::maxFwd);

RecoverResult recoverAfterContact(double desiredWallDistMm,
                                  double faceHeadingDeg,
                                  bool doSetDistance = false);

bool runSegmentWithRecovery(const std::function<void()>& segmentFn,
                            const std::function<double()>& driveCmdFn,
                            double desiredWallDistMm,
                            double faceHeadingDeg,
                            bool doSetDistance = false,
                            int maxAttempts = 2,
                            int pollMs = Tuning::distUpdateMs);

struct WallResetParams {
    double faceHeadingDeg = 0.0;
    double leftTargetMm = 0.0;
    double rightTargetMm = 0.0;
    int squareTimeoutMs = 800;
    int setTimeoutMs = 900;
    int tolMm = 10;
    double maxTurn = Tuning::maxTurn;
    double maxFwd = Tuning::maxFwd;
    bool trySquare = true;
    bool useLeft = true;
    bool useRight = true;
};

bool wallReset(const WallResetParams& p);
} // namespace AutonRecovery
