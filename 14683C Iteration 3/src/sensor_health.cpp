#include "sensor_health.hpp"

#include <cmath>
#include <cstdint>

#include "robot_config.hpp"

namespace {
constexpr int kLeftDriveCount = 3;
constexpr int kRightDriveCount = 3;

constexpr double kImuStuckEpsilonDeg = 0.5;
constexpr int kImuStuckCycles = 20;

constexpr int kDistNoTargetMm = 9999;
constexpr int kDistNoTargetCycles = 4;
constexpr int kDistStuckCycles = 12;
constexpr int kDistStuckEpsilonMm = 2;
constexpr int kDistFaultCycles = 3;

constexpr double kStallVelocityRpm = 5.0;
constexpr int kStallCurrentMa = 2000;
constexpr int kStallCycles = 6;
constexpr double kMoveRpmThreshold = 10.0;
constexpr double kTurnRpmDeltaThreshold = 12.0;
constexpr int kCmdVoltageMvThreshold = 1500;

struct DistanceTracker {
    int last = 0;
    int same_count = 0;
    int zero_count = 0;
    int max_count = 0;
    int fault_count = 0;
    bool has_last = false;
};

struct MotorTracker {
    int stall_count = 0;
    MotorStatus status = MotorStatus::NORMAL;
};

SensorHealth g_health = {
    HealthState::WARNING, HealthState::OK, HealthState::OK,
    HealthState::OK,      HealthState::OK, HealthState::OK,
    HealthState::OK};

SensorDetail g_detail = {
    ImuStatus::CALIBRATING,
    DistanceStatus::OK,
    DistanceStatus::OK,
    MotorStatus::NORMAL,
    MotorStatus::NORMAL,
    MotorStatus::NORMAL,
    MotorStatus::NORMAL};

double angle_delta(double a, double b) {
    double diff = std::fabs(a - b);
    if (diff > 180.0) {
        diff = 360.0 - diff;
    }
    return diff;
}

void update_imu(bool expecting_turn) {
    if (imu.is_calibrating()) {
        g_detail.imu_status = ImuStatus::CALIBRATING;
        g_health.imu = HealthState::WARNING;
        return;
    }

    const double heading = imu.get_heading();
    if (!std::isfinite(heading)) {
        g_detail.imu_status = ImuStatus::FAULT;
        g_health.imu = HealthState::FAULT;
        return;
    }

    static double last_heading = heading;
    static int stuck_count = 0;
    const double delta = angle_delta(heading, last_heading);
    if (expecting_turn && delta <= kImuStuckEpsilonDeg) {
        ++stuck_count;
    } else {
        stuck_count = 0;
    }
    last_heading = heading;

    if (expecting_turn && stuck_count >= kImuStuckCycles) {
        g_detail.imu_status = ImuStatus::FAULT;
        g_health.imu = HealthState::FAULT;
    } else {
        g_detail.imu_status = ImuStatus::OK;
        g_health.imu = HealthState::OK;
    }
}

DistanceStatus update_distance(pros::Distance& dist, DistanceTracker& t,
                               HealthState& out_health,
                               bool expecting_translate) {
    const int value = dist.get();

    if (value < 0) {
        ++t.fault_count;
    } else {
        t.fault_count = 0;
    }

    if (value == 0) {
        ++t.zero_count;
    } else {
        t.zero_count = 0;
    }

    if (value >= kDistNoTargetMm) {
        ++t.max_count;
    } else {
        t.max_count = 0;
    }

    if (!expecting_translate) {
        t.same_count = 0;  // Idle robots should not be flagged as stuck.
    } else if (value > 0 && value < kDistNoTargetMm && t.has_last &&
               std::abs(value - t.last) <= kDistStuckEpsilonMm) {
        ++t.same_count;
    } else {
        t.same_count = 0;
    }

    t.last = value;
    t.has_last = true;

    if (t.fault_count >= kDistFaultCycles) {
        out_health = HealthState::FAULT;
        return DistanceStatus::FAULT;
    }
    if (t.same_count >= kDistStuckCycles) {
        out_health = HealthState::FAULT;
        return DistanceStatus::STUCK;
    }
    if (t.zero_count >= kDistNoTargetCycles ||
        t.max_count >= kDistNoTargetCycles) {
        out_health = HealthState::WARNING;
        return DistanceStatus::NO_TARGET;
    }

    out_health = HealthState::OK;
    return DistanceStatus::OK;
}

bool motor_is_stalled(pros::AbstractMotor& motor, std::uint8_t index) {
    const double vel = motor.get_actual_velocity(index);
    const int current = motor.get_current_draw(index);
    if (!std::isfinite(vel) || current < 0) {
        return false;
    }
    return std::fabs(vel) <= kStallVelocityRpm && current >= kStallCurrentMa;
}

bool motor_is_commanded(pros::AbstractMotor& motor, std::uint8_t index) {
    const int voltage = motor.get_voltage(index);
    return std::abs(voltage) >= kCmdVoltageMvThreshold;
}

MotorStatus update_motor_group(pros::AbstractMotor& motor, int count,
                               MotorTracker& tracker, HealthState& out_health) {
    bool stalled = false;
    bool commanded = false;
    for (int i = 0; i < count; ++i) {
        const auto idx = static_cast<std::uint8_t>(i);
        const bool is_cmd = motor_is_commanded(motor, idx);
        commanded = commanded || is_cmd;
        if (is_cmd && motor_is_stalled(motor, idx)) {
            stalled = true;
        }
    }

    if (commanded && stalled) {
        ++tracker.stall_count;
    } else {
        tracker.stall_count = 0;
    }

    if (commanded && tracker.stall_count >= kStallCycles) {
        tracker.status = MotorStatus::STALL;
        out_health = HealthState::FAULT;
    } else {
        tracker.status = MotorStatus::NORMAL;
        out_health = HealthState::OK;
    }
    return tracker.status;
}

MotorStatus update_motor(pros::AbstractMotor& motor, MotorTracker& tracker,
                         HealthState& out_health) {
    const bool commanded = motor_is_commanded(motor, 0);
    const bool stalled = commanded && motor_is_stalled(motor, 0);
    if (stalled) {
        ++tracker.stall_count;
    } else {
        tracker.stall_count = 0;
    }

    if (commanded && tracker.stall_count >= kStallCycles) {
        tracker.status = MotorStatus::STALL;
        out_health = HealthState::FAULT;
    } else {
        tracker.status = MotorStatus::NORMAL;
        out_health = HealthState::OK;
    }
    return tracker.status;
}
}

void sensor_health_update() {
    static DistanceTracker left_track{};
    static DistanceTracker right_track{};
    static MotorTracker left_drive_track{};
    static MotorTracker right_drive_track{};
    static MotorTracker intake_track{};
    static MotorTracker outtake_track{};

    double left_rpm = 0.0;
    double right_rpm = 0.0;
    for (int i = 0; i < kLeftDriveCount; ++i) {
        left_rpm += std::fabs(leftMotors.get_actual_velocity(
            static_cast<std::uint8_t>(i)));
    }
    for (int i = 0; i < kRightDriveCount; ++i) {
        right_rpm += std::fabs(rightMotors.get_actual_velocity(
            static_cast<std::uint8_t>(i)));
    }
    left_rpm /= static_cast<double>(kLeftDriveCount);
    right_rpm /= static_cast<double>(kRightDriveCount);

    const double avg_rpm = 0.5 * (left_rpm + right_rpm);
    const bool expecting_translate = avg_rpm > kMoveRpmThreshold;
    const bool expecting_turn =
        std::fabs(left_rpm - right_rpm) > kTurnRpmDeltaThreshold;

    update_imu(expecting_turn);

    g_detail.left_dist_status =
        update_distance(leftDist, left_track, g_health.leftDist,
                        expecting_translate);
    g_detail.right_dist_status =
        update_distance(rightDist, right_track, g_health.rightDist,
                        expecting_translate);

    g_detail.left_drive_status =
        update_motor_group(leftMotors, kLeftDriveCount, left_drive_track,
                           g_health.leftDrive);
    g_detail.right_drive_status =
        update_motor_group(rightMotors, kRightDriveCount, right_drive_track,
                           g_health.rightDrive);

    g_detail.intake_status =
        update_motor(intakeMotor, intake_track, g_health.intakeMotor);
    g_detail.outtake_status =
        update_motor(outtakeMotor, outtake_track, g_health.outtakeMotor);
}

const SensorHealth& get_sensor_health() {
    return g_health;
}

const SensorDetail& get_sensor_detail() {
    return g_detail;
}
