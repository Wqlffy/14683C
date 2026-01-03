#pragma once

#include <cstdint>

enum class HealthState : uint8_t {
    OK,
    WARNING,
    FAULT
};

enum class ImuStatus : uint8_t {
    OK,
    CALIBRATING,
    FAULT
};

enum class DistanceStatus : uint8_t {
    OK,
    NO_TARGET,
    STUCK,
    FAULT
};

enum class MotorStatus : uint8_t {
    NORMAL,
    STALL
};

struct SensorHealth {
    HealthState imu;
    HealthState leftDist;
    HealthState rightDist;
    HealthState leftDrive;
    HealthState rightDrive;
    HealthState intakeMotor;
    HealthState outtakeMotor;
};

struct SensorDetail {
    ImuStatus imu_status;
    DistanceStatus left_dist_status;
    DistanceStatus right_dist_status;
    MotorStatus left_drive_status;
    MotorStatus right_drive_status;
    MotorStatus intake_status;
    MotorStatus outtake_status;
};

void sensor_health_update();
const SensorHealth& get_sensor_health();
const SensorDetail& get_sensor_detail();
