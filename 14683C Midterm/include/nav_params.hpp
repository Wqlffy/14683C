#pragma once

#include <cstdint>

namespace nav_params {

// --- Robot Geometry -------------------------------------------------------
// TUNE: Measure track width (mm) between drivetrain wheel centers.
inline constexpr double TRACK_WIDTH_MM = 280.0;
// TUNE: Wheel diameter used by lemlib for kinematics (mm).
inline constexpr double DRIVE_WHEEL_DIAMETER_MM = 101.6;
// TUNE: Wheelbase (mm) between front and back axle centerlines.
inline constexpr double WHEELBASE_MM = 240.0;
// TUNE: IMU mounting offset from robot center (mm).
inline constexpr double IMU_OFFSET_X_MM = 0.0;
inline constexpr double IMU_OFFSET_Y_MM = 0.0;

// --- Sensor Offsets -------------------------------------------------------
// TUNE: Distance sensor mounting offsets (mm) relative to robot center.
inline constexpr double DIST_RIGHT_OFFSET_MM = 120.0;
inline constexpr double DIST_BACK_OFFSET_MM = -130.0;

// TUNE: Minimum and maximum valid wall distances (mm).
inline constexpr double DIST_MIN_MM = 50.0;
inline constexpr double DIST_MAX_MM = 1800.0;

// --- Motion Limits --------------------------------------------------------
// TUNE: Maximum drive speed (mm/s) used by motion profiles.
inline constexpr int DRIVE_MAX_SPEED_MMPS = 1800;
// TUNE: Maximum drive acceleration (mm/s^2).
inline constexpr int DRIVE_MAX_ACCEL_MMPS2 = 2200;
// TUNE: Reduced speed for precision mode (mm/s).
inline constexpr int DRIVE_PRECISION_SPEED_MMPS = 900;

// --- PID Gains ------------------------------------------------------------
// TUNE: Lateral PID (lemlib). Units: (kP, kI, kD).
inline constexpr double LATERAL_KP = 0.6;
inline constexpr double LATERAL_KI = 0.0;
inline constexpr double LATERAL_KD = 5.0;
// TUNE: Angular PID (lemlib).
inline constexpr double ANGULAR_KP = 3.0;
inline constexpr double ANGULAR_KI = 0.0;
inline constexpr double ANGULAR_KD = 32.0;

// --- Path Correction ------------------------------------------------------
// TUNE: Heading correction PID gains.
inline constexpr double PATH_KP = 0.8;
inline constexpr double PATH_KI = 0.0;
inline constexpr double PATH_KD = 6.0;
// TUNE: Maximum correction command (% voltage equivalent).
inline constexpr double PATH_MAX_OUTPUT_PCT = 30.0;
// TUNE: Minimum forward command (%) required before correction engages.
inline constexpr double PATH_MIN_FWD_THRESHOLD_PCT = 10.0;
// TUNE: Hysteresis band for target heading (deg).
inline constexpr double PATH_HEADING_EPS_DEG = 1.5;

// --- Wallsnap -------------------------------------------------------------
// TUNE: Exponential moving average alpha (0..1) for distance sensors.
inline constexpr double WALL_EMA_ALPHA = 0.3;
// TUNE: Max speed (mm/s) at which snapping is permitted.
inline constexpr double WALL_SNAP_MAX_SPEED_MMPS = 200.0;
// TUNE: Allowed heading error (deg) when snapping.
inline constexpr double WALL_SNAP_HEADING_TOL_DEG = 7.0;
// TUNE: Range tolerance (mm) to accept wall snap.
inline constexpr double WALL_SNAP_RANGE_TOL_MM = 40.0;

// --- Mechanism Settings ---------------------------------------------------
// TUNE: Roller diameter (m) for surface velocity calculations.
inline constexpr double ROLLER_DIAMETER_M = 0.0508;
// TUNE: Loader wheel diameter (m).
inline constexpr double LOADER_DIAMETER_M = 0.0508;
// TUNE: Roller max RPM (depends on cartridge).
inline constexpr int ROLLER_MAX_RPM = 100;
// TUNE: Loader max RPM (green cartridge).
inline constexpr int LOADER_MAX_RPM = 200;
// TUNE: Roller direction sign (1 or -1).
inline constexpr int ROLLER_DIR = 1;
// TUNE: Loader direction sign (1 or -1).
inline constexpr int LOADER_DIR = 1;
// TUNE: Intake direction sign (1 or -1).
inline constexpr int INTAKE_DIR = 1;

// TUNE: Intake jam detection: current spike (A) and duration (ms).
inline constexpr double INTAKE_JAM_CURRENT_A = 2.6;
inline constexpr int INTAKE_JAM_TIME_MS = 250;
// TUNE: Jam clear reverse percentage.
inline constexpr int INTAKE_JAM_REVERSE_PCT = -80;
// TUNE: Jam clear duration.
inline constexpr int INTAKE_JAM_REVERSE_MS = 200;

// --- Timing ---------------------------------------------------------------
// TUNE: Scoring durations.
inline constexpr int SCORE_MID_MS = 350;
inline constexpr int SCORE_LONG_MS = 550;
// TUNE: Jam clear duration for scoring.
inline constexpr int SCORE_JAM_MS = 300;
// TUNE: Purge duration safety timeout.
inline constexpr int PURGE_TIMEOUT_MS = 1000;

// --- Barrier --------------------------------------------------------------
// TUNE: Current threshold to detect barrier contact (A).
inline constexpr double BARRIER_CONTACT_CURRENT_A = 3.5;
// TUNE: Minimum time (ms) pushing before evaluating contact.
inline constexpr int BARRIER_MIN_PUSH_MS = 150;

// --- Autopark -------------------------------------------------------------
// TUNE: Desired distance from wall when parking (mm).
inline constexpr double AUTOPARK_TARGET_DIST_MM = 120.0;
// TUNE: Autopark heading tolerance (deg).
inline constexpr double AUTOPARK_HEADING_TOL_DEG = 2.0;
// TUNE: Autopark timeout (ms).
inline constexpr int AUTOPARK_TIMEOUT_MS = 4000;

// --- Telemetry ------------------------------------------------------------
inline constexpr int TELEMETRY_INTERVAL_MS = 10;

// --- Skills Routine -------------------------------------------------------
// TUNE: Global timeout for autonomous routine (ms).
inline constexpr int SKILLS_GLOBAL_TIMEOUT_MS = 58000;

}

