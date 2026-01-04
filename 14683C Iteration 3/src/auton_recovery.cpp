#include "auton_recovery.hpp"

#include <atomic>
#include <cmath>
#include <cstdio>

#include "robot_config.hpp"

namespace AutonRecovery {
namespace {
FilteredDistances g_distances{};

double clamp(double v, double lo, double hi) {
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

double wrap_deg(double deg) {
    while (deg > 180.0) deg -= 360.0;
    while (deg <= -180.0) deg += 360.0;
    return deg;
}

void set_drive(double left, double right) {
    left = clamp(left, -1.0, 1.0);
    right = clamp(right, -1.0, 1.0);
    leftMotors.move(static_cast<int>(left * 127.0));
    rightMotors.move(static_cast<int>(right * 127.0));
}

double avg_group_velocity(const pros::MotorGroup& group, int count) {
    double total = 0.0;
    for (int i = 0; i < count; ++i) {
        total += std::fabs(group.get_actual_velocity(static_cast<uint8_t>(i)));
    }
    return total / static_cast<double>(count);
}

void update_filtered_distances() {
    const int left_mm = leftDist.get();
    const int right_mm = rightDist.get();

    const bool left_valid = distValidMm(left_mm);
    const bool right_valid = distValidMm(right_mm);

    if (left_valid) {
        if (!g_distances.leftValid) {
            g_distances.leftMmFilt = left_mm;
        } else {
            g_distances.leftMmFilt =
                filt(g_distances.leftMmFilt, left_mm, Tuning::distAlpha);
        }
    }
    if (right_valid) {
        if (!g_distances.rightValid) {
            g_distances.rightMmFilt = right_mm;
        } else {
            g_distances.rightMmFilt =
                filt(g_distances.rightMmFilt, right_mm, Tuning::distAlpha);
        }
    }

    g_distances.leftValid = left_valid;
    g_distances.rightValid = right_valid;
}

void set_drive_brake(pros::motor_brake_mode_e_t mode) {
    leftMotors.set_brake_mode_all(mode);
    rightMotors.set_brake_mode_all(mode);
    intakeMotor.set_brake_mode(mode);
    outtakeMotor.set_brake_mode(mode);
}
}

bool distValidMm(int mm) {
    return mm >= static_cast<int>(Tuning::distMinMm) &&
           mm <= static_cast<int>(Tuning::distMaxMm);
}

double filt(double prev, double now, double alpha) {
    return prev + alpha * (now - prev);
}

const FilteredDistances& get_filtered_distances() {
    return g_distances;
}

bool detectStall(double driveCmd, int dtMs) {
    static int stall_ms = 0;
    static bool latched = false;

    if (dtMs <= 0) {
        dtMs = Tuning::distUpdateMs;
    }

    const double avg_left = avg_group_velocity(leftMotors, Tuning::leftDriveCount);
    const double avg_right = avg_group_velocity(rightMotors, Tuning::rightDriveCount);
    const double avg_vel = 0.5 * (avg_left + avg_right);

    if (std::fabs(driveCmd) > Tuning::stallCmdThresh &&
        avg_vel < Tuning::stallVelThreshRpm) {
        stall_ms += dtMs;
    } else {
        stall_ms = 0;
        latched = false;
    }

    if (!latched && stall_ms >= Tuning::stallTimeMs) {
        latched = true;
        std::printf("RECOVERY: stall detected cmd=%.2f vel=%.2f\n",
                    driveCmd, avg_vel);
        return true;
    }
    return false;
}

void stabilize(int holdMs) {
    set_drive(0.0, 0.0);
    set_drive_brake(Tuning::holdBrakeMode);
    pros::delay(holdMs);
    set_drive_brake(Tuning::defaultBrakeMode);
}

bool wallSquare(int timeoutMs, int diffMmTol, double maxTurn) {
    update_filtered_distances();
    if (!g_distances.leftValid || !g_distances.rightValid) {
        std::printf("RECOVERY: wallSquare skipped (invalid distance)\n");
        return false;
    }

    int stable = 0;
    const int start = pros::millis();
    while (pros::millis() - start < timeoutMs) {
        update_filtered_distances();
        if (!g_distances.leftValid || !g_distances.rightValid) {
            set_drive(0.0, 0.0);
            std::printf("RECOVERY: wallSquare aborted (invalid distance)\n");
            return false;
        }

        const double error = g_distances.leftMmFilt - g_distances.rightMmFilt;
        const double turn = clamp(Tuning::squareKp * error, -maxTurn, maxTurn);
        set_drive(turn, -turn);

        if (std::fabs(error) <= diffMmTol) {
            ++stable;
        } else {
            stable = 0;
        }

        if (stable >= Tuning::squareStableSamples) {
            set_drive(0.0, 0.0);
            return true;
        }
        pros::delay(Tuning::distUpdateMs);
    }

    set_drive(0.0, 0.0);
    return false;
}

bool wallSetDistance(double targetMm, int timeoutMs, int tolMm, double maxFwd,
                     double faceHeadingDeg) {
    update_filtered_distances();
    if (!g_distances.leftValid || !g_distances.rightValid) {
        std::printf("RECOVERY: wallSetDistance skipped (invalid distance)\n");
        return false;
    }

    int stable = 0;
    const int start = pros::millis();
    while (pros::millis() - start < timeoutMs) {
        update_filtered_distances();
        if (!g_distances.leftValid || !g_distances.rightValid) {
            set_drive(0.0, 0.0);
            std::printf("RECOVERY: wallSetDistance aborted (invalid distance)\n");
            return false;
        }

        const double avg = 0.5 * (g_distances.leftMmFilt + g_distances.rightMmFilt);
        const double error = targetMm - avg;
        const double fwd = clamp(Tuning::setDistKp * error, -maxFwd, maxFwd);

        const double heading_err = wrap_deg(faceHeadingDeg - imu.get_heading());
        const double turn = clamp(Tuning::headingKp * heading_err,
                                  -Tuning::maxTurn, Tuning::maxTurn);
        set_drive(fwd + turn, fwd - turn);

        if (std::fabs(error) <= tolMm) {
            ++stable;
        } else {
            stable = 0;
        }

        if (stable >= Tuning::setStableSamples) {
            set_drive(0.0, 0.0);
            return true;
        }
        pros::delay(Tuning::distUpdateMs);
    }

    set_drive(0.0, 0.0);
    return false;
}

RecoverResult recoverAfterContact(double desiredWallDistMm,
                                  double faceHeadingDeg,
                                  bool doSetDistance) {
    update_filtered_distances();
    if (!g_distances.leftValid || !g_distances.rightValid) {
        std::printf("RECOVERY: sensors invalid, skipping\n");
        return RecoverResult::SKIPPED_NO_WALL;
    }

    std::printf("RECOVERY: start L=%.1f R=%.1f\n",
                g_distances.leftMmFilt, g_distances.rightMmFilt);

    stabilize(150);

    const bool squared = wallSquare(800, 8, Tuning::maxTurn);
    bool set_ok = true;
    if (doSetDistance) {
        set_ok = wallSetDistance(desiredWallDistMm, 900, 10, Tuning::maxFwd,
                                 faceHeadingDeg);
    }

    stabilize(100);

    if (!squared && !set_ok) {
        std::printf("RECOVERY: timeout\n");
        return RecoverResult::TIMEOUT;
    }

    std::printf("RECOVERY: done\n");
    return RecoverResult::DONE;
}

bool runSegmentWithRecovery(const std::function<void()>& segmentFn,
                            const std::function<double()>& driveCmdFn,
                            double desiredWallDistMm,
                            double faceHeadingDeg,
                            bool doSetDistance,
                            int maxAttempts,
                            int pollMs) {
    if (!segmentFn) {
        return false;
    }

    std::atomic<bool> done{false};
    pros::Task task([&]() {
        segmentFn();
        done = true;
    });

    int attempts = 0;
    while (!done) {
        const double cmd = driveCmdFn ? driveCmdFn() : 0.0;
        if (detectStall(cmd, pollMs)) {
            ++attempts;
            const RecoverResult res =
                recoverAfterContact(desiredWallDistMm, faceHeadingDeg,
                                    doSetDistance);
            if (attempts >= maxAttempts ||
                res == RecoverResult::SKIPPED_NO_WALL) {
                break;
            }
        }
        pros::delay(pollMs);
    }

    return done;
}
}
