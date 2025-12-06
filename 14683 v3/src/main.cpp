#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/asset.hpp"
#include "lemlib/chassis/chassis.hpp"
#include "lemlib/chassis/trackingWheel.hpp"
#include "command_framework.hpp"
#include "mcl.hpp"
#include "motion_profile.hpp"
#include "ramsete.hpp"
#include "pros/adi.hpp"
#include "pros/llemu.hpp" // IWYU pragma: keep
#include "pros/misc.h"
#include "pros/motor_group.hpp"
#include "pros/motors.h"
#include "pros/motors.hpp"
#include "pros/rtos.hpp"
#include "pros/distance.hpp"
#include "pros/apix.h" // IWYU pragma: keep
#include <algorithm>
#include <cstdint>
#include <cmath>
#include <functional>
#include <memory>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <utility>


pros::Controller controller(pros::E_CONTROLLER_MASTER);

pros::MotorGroup leftMotors({15, 1-16, -17}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::MotorGroup rightMotors({-5, 6, 7}, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);

pros::Motor intake(20, pros::MotorGearset::blue, pros::MotorEncoderUnits::degrees);
pros::Motor indexer(-18, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
pros::Motor scoring(19, pros::MotorGearset::green, pros::MotorEncoderUnits::degrees);
pros::adi::Pneumatics tongue('A', false); 
pros::adi::Pneumatics aligner('B', false); 

pros::Imu imu(9);
pros::Rotation verticalEnc(11); 
lemlib::TrackingWheel vertical(&verticalEnc, lemlib::Omniwheel::NEW_325, 0);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors,
                              12.4, // length between left and right wheels (middle of wheels)
                              lemlib::Omniwheel::NEW_325, 
                              450,
                              2 // traction wheel is usually 2
);

lemlib::ControllerSettings linearController(5.5, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            18, // derivative gain (kD)
                                            10, // anti windup
                                            1.0, // small error range, in inches
                                            250, // small error range timeout, in milliseconds
                                            3.0, // large error range, in inches
                                            2500, // large error range timeout, in milliseconds
                                            75 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(3.8,// proportional gain (kP)
                                             0, // integral gain (kI)
                                             22, // derivative gain (kD)
                                             5 , // anti windup
                                             2.0, // small error range, in degrees
                                             220, // small error range timeout, in milliseconds
                                             7.0, // large error range, in degrees
                                             2500, // large error range timeout, in milliseconds
                                             55 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(&vertical, nullptr, nullptr, nullptr, &imu);

lemlib::ExpoDriveCurve throttleCurve(8, 12, 1.02);

lemlib::ExpoDriveCurve steerCurve(10, 15, 1.02);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

constexpr bool ENABLE_WALL_DISTANCE = false;  // set true once the wall distance sensor is wired
constexpr int WALL_DISTANCE_PORT = 8;         // smart port for the VEX Distance Sensor
pros::Distance wallDistanceSensor(WALL_DISTANCE_PORT);

constexpr robot::FieldWalls FIELD_WALLS = {-70.5, 70.5, -70.5, 70.5};  // 141\" square field
constexpr robot::WallDistanceSensor WALL_DISTANCE_MODEL = {
    6.0,   // offsetX: sensor sits 6\" forward of tracking center
    0.0,   // offsetY: centered left/right
    0.0,   // yawOffset: faces straight forward
    72.0,  // maxRange: clamp far readings to 6 ft
    1.0    // stdDev: tweak based on real sensor noise (inches)
};

constexpr double TRACK_WIDTH = 12.4;
constexpr double WHEEL_DIAMETER = 3.25;
constexpr double DRIVE_MAX_RPM = 450.0;
constexpr double AUTO_START_X_IN = -46.362;
constexpr double AUTO_START_Y_IN = 0.099;
constexpr double AUTO_START_HEADING_DEG = 140.0;
constexpr robot::MotionNoise AUTO_MOTION_NOISE = {0.25, 0.25, 0.02};
constexpr double AUTO_HEADING_STD = 0.03;

robot::CommandScheduler commandScheduler;
robot::MonteCarloLocalizer localizer(400);

constexpr double PI = 3.14159265358979323846;

static double degToRad(double deg) { return deg * (PI / 180.0); }
static double radToDeg(double rad) { return rad * (180.0 / PI); }
static double wrapAngle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

static robot::Pose2D toPose2D(const lemlib::Pose& pose) {
    return {pose.x, pose.y, degToRad(pose.theta)};
}

[[maybe_unused]] static lemlib::Pose toLemlibPose(const robot::Pose2D& pose) {
    return lemlib::Pose(static_cast<float>(pose.x), static_cast<float>(pose.y),
                        static_cast<float>(radToDeg(pose.theta)));
}

static robot::Pose2D readOdomPose() {
    return toPose2D(chassis.getPose());
}

static bool readWallDistanceInches(double& outDistanceInches) {
    if (!ENABLE_WALL_DISTANCE) return false;
    const double mm = wallDistanceSensor.get();
    if (mm <= 0.0) return false;
    outDistanceInches = mm / 25.4;
    if (outDistanceInches > WALL_DISTANCE_MODEL.maxRange) {
        outDistanceInches = WALL_DISTANCE_MODEL.maxRange;
    }
    return true;
}

static void seedLocalizerFromChassis(double spreadX = 1.0, double spreadY = 1.0, double spreadTheta = 0.05) {
    localizer.seedParticles(readOdomPose(), spreadX, spreadY, spreadTheta);
}

struct PathPoint {
    double x;
    double y;
    double headingDeg;
    bool hasHeading;
};

static std::vector<robot::Waypoint> loadPathjerryWaypoints(const std::string& filename) {
    std::ifstream file(filename);
    if (!file.is_open()) return {};

    std::vector<PathPoint> rawPoints;
    std::string line;
    while (std::getline(file, line)) {
        if (line.empty()) continue;
        if (line.rfind("#PATH.JERRYIO-DATA", 0) == 0) break;
        if (line[0] == '#') continue;

        for (char& c : line) {
            if (c == ',') c = ' ';
        }

        std::stringstream ss(line);
        double x = 0.0, y = 0.0, speed = 0.0, heading = 0.0;
        if (!(ss >> x >> y)) continue;
        bool hasHeading = false;
        if (ss >> speed) {
            if (ss >> heading) {
                hasHeading = true;
            }
        }
        rawPoints.push_back({x, y, heading, hasHeading});
    }

    if (rawPoints.size() < 2) return {};

    std::vector<robot::Waypoint> waypoints;
    waypoints.reserve(rawPoints.size());

    for (std::size_t i = 0; i < rawPoints.size(); ++i) {
        double headingRad;
        if (rawPoints[i].hasHeading) {
            headingRad = degToRad(rawPoints[i].headingDeg);
        } else {
            const std::size_t nextIdx = (i + 1 < rawPoints.size()) ? i + 1 : i;
            const double dx = rawPoints[nextIdx].x - rawPoints[i].x;
            const double dy = rawPoints[nextIdx].y - rawPoints[i].y;
            headingRad = std::atan2(dy, dx);
        }
        // PathJERRY uses cm for VEX GPS; convert to inches for lemlib.
        const double xInches = rawPoints[i].x / 2.54;
        const double yInches = rawPoints[i].y / 2.54;
        waypoints.push_back({xInches, yInches, headingRad});
    }

    return waypoints;
}

static void driveWithVoltage(double left, double right) {
    left = std::clamp(left, -12000.0, 12000.0);
    right = std::clamp(right, -12000.0, 12000.0);
    leftMotors.move_voltage(static_cast<int32_t>(left));
    rightMotors.move_voltage(static_cast<int32_t>(right));
}

[[maybe_unused]] static std::shared_ptr<robot::Command> makeTimedMotorCommand(pros::Motor& motor,
                                                                             int voltage,
                                                                             uint32_t durationMs) {
    auto hold = std::make_shared<robot::FunctionalCommand>(
        [voltage, &motor]() { motor.move_voltage(voltage); },
        [voltage, &motor]() { motor.move_voltage(voltage); },
        []() { return false; },
        [&motor](bool) { motor.move_voltage(0); });
    auto wait = std::make_shared<robot::WaitCommand>(durationMs);
    return std::make_shared<robot::ParallelRaceGroup>(
        std::vector<std::shared_ptr<robot::Command>>{hold, wait});
}

static void runLegacySkillsAuto();
static void runRamseteAuto();


void initialize() {
    chassis.calibrate();
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    intake.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    indexer.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    scoring.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    seedLocalizerFromChassis(2.0, 2.0, degToRad(3.0));
}

void disabled() {}

void competition_initialize() {
    while (pros::competition::is_disabled()) {
        pros::delay(20);
    }
}

ASSET(skills_txt)
static void runLegacySkillsAuto() {
    chassis.setPose(-46.362, 0.099, 86.645);
    chassis.follow(skills_txt, 14, 45000, true, true);

    chassis.waitUntil(53.971);
    intake.move_voltage(12000);
    tongue.set_value(true);

    // at 83.459: wait 3 seconds
    chassis.waitUntil(83.459);
    pros::delay(3000);

    // at 93.459: stop intake, pull tongue
    chassis.waitUntil(93.459);
    intake.move_voltage(0);
    tongue.set_value(false);

    // at 216.017:
    //   push aligner, start scoring + indexer,
    //   wait 4 seconds, then stop scoring/indexer, pull aligner
    chassis.waitUntil(216.017);
    aligner.set_value(true);
    scoring.move_voltage(12000);
    indexer.move_voltage(12000);
    pros::delay(4000);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    aligner.set_value(false);

    // at 234.017: push tongue, start intake
    chassis.waitUntil(234.017);
    tongue.set_value(true);
    intake.move_voltage(12000);

    // at 250.017: wait 3 seconds
    chassis.waitUntil(250.017);
    pros::delay(3000);

    // at 265.345: pull tongue, stop intake
    chassis.waitUntil(265.345);
    tongue.set_value(false);
    intake.move_voltage(0);

    // at 281.475:
    //   push aligner, start scoring + indexer,
    //   wait 4 seconds, then stop scoring/indexer, pull aligner
    chassis.waitUntil(281.475);
    aligner.set_value(true);
    scoring.move_voltage(12000);
    indexer.move_voltage(12000);
    pros::delay(4000);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    aligner.set_value(false);

    // at 383.032: push tongue, start intake
    chassis.waitUntil(383.032);
    tongue.set_value(true);
    intake.move_voltage(12000);

    // at 397.738: wait 3 seconds
    chassis.waitUntil(397.738);
    pros::delay(3000);

    // at 411.712: pull tongue, stop intake
    chassis.waitUntil(411.712);
    tongue.set_value(false);
    intake.move_voltage(0);

    // at 524.94:
    //   push aligner, start scoring + indexer,
    //   wait 4 seconds, then stop scoring/indexer, pull aligner
    chassis.waitUntil(524.94);
    aligner.set_value(true);
    scoring.move_voltage(12000);
    indexer.move_voltage(12000);
    pros::delay(4000);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    aligner.set_value(false);

    // at 545.366: push tongue, start intake
    chassis.waitUntil(545.366);
    tongue.set_value(true);
    intake.move_voltage(12000);

    // at 556.826: wait 3 seconds
    chassis.waitUntil(556.826);
    pros::delay(3000);

    // at 566.826: pull tongue, stop intake
    chassis.waitUntil(566.826);
    tongue.set_value(false);
    intake.move_voltage(0);

    // at 588.826:
    //   push aligner, start scoring + indexer,
    //   wait 4 seconds, then stop scoring/indexer, pull aligner
    chassis.waitUntil(588.826);
    aligner.set_value(true);
    scoring.move_voltage(12000);
    indexer.move_voltage(12000);
    pros::delay(4000);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
    aligner.set_value(false);

    chassis.waitUntilDone();

    intake.move_voltage(0);
    indexer.move_voltage(0);
    scoring.move_voltage(0);
    tongue.set_value(false);
    aligner.set_value(false);
}

static void runRamseteAuto() {
    chassis.cancelAllMotions();

    std::vector<robot::Waypoint> waypoints = loadPathjerryWaypoints("static/test.txt");
    if (!waypoints.empty()) {
        const auto& start = waypoints.front();
        chassis.setPose(start.x, start.y, radToDeg(start.heading));
        seedLocalizerFromChassis(1.0, 1.0, degToRad(2.5));
    } else {
        chassis.setPose(AUTO_START_X_IN, AUTO_START_Y_IN, AUTO_START_HEADING_DEG);
        seedLocalizerFromChassis(1.0, 1.0, degToRad(2.5));
        const robot::Pose2D start{AUTO_START_X_IN, AUTO_START_Y_IN, degToRad(AUTO_START_HEADING_DEG)};
        waypoints = {
            {start.x, start.y, start.theta},
            {start.x + 24.0, start.y, start.theta},
            {start.x + 36.0, start.y + 18.0, start.theta + degToRad(35.0)},
            {start.x + 20.0, start.y + 42.0, start.theta + degToRad(90.0)}};
    }

    const double maxVel = 40.0;     // in/s
    const double maxAccel = 120.0;  // in/s^2
    auto trajectory = robot::generateTrajectory(waypoints, maxVel, maxAccel, 0.02);
    if (trajectory.empty()) return;

    std::function<bool(double&)> wallDistanceGetter = {};
    if (ENABLE_WALL_DISTANCE) {
        wallDistanceGetter = [](double& distanceInches) {
            const double mm = wallDistanceSensor.get();
            if (mm <= 0.0) return false;
            distanceInches = mm / 25.4;
            if (distanceInches > WALL_DISTANCE_MODEL.maxRange) {
                distanceInches = WALL_DISTANCE_MODEL.maxRange;
            }
            return true;
        };
    }

    auto ramseteCommand = std::make_shared<robot::RamsetePathCommand>(
        trajectory, localizer, []() { return readOdomPose(); }, driveWithVoltage, TRACK_WIDTH,
        WHEEL_DIAMETER, DRIVE_MAX_RPM, AUTO_MOTION_NOISE, AUTO_HEADING_STD, FIELD_WALLS,
        WALL_DISTANCE_MODEL, wallDistanceGetter);

    auto intakeCycle = std::make_shared<robot::ParallelRaceGroup>(
        std::vector<std::shared_ptr<robot::Command>>{
            std::make_shared<robot::FunctionalCommand>(
                []() {
                    tongue.extend();
                    intake.move_voltage(12000);
                },
                []() { intake.move_voltage(12000); },
                []() { return false; },
                [](bool interrupted) {
                    (void)interrupted;
                    intake.move_voltage(0);
                    tongue.retract();
                }),
            std::make_shared<robot::WaitCommand>(1200u)});

    auto sequence = std::make_shared<robot::SequentialCommandGroup>(
        std::vector<std::shared_ptr<robot::Command>>{ramseteCommand, intakeCycle});

    commandScheduler.cancelAll();
    commandScheduler.schedule(sequence);
    while (!commandScheduler.empty() && pros::competition::is_autonomous()) {
        commandScheduler.run();
        pros::delay(10);
    }

    driveWithVoltage(0.0, 0.0);
    intake.move_voltage(0);
    scoring.move_voltage(0);
    indexer.move_voltage(0);
}

void autonomous() {
    constexpr bool useRamseteAuto = true;
    if (useRamseteAuto) {
        runRamseteAuto();
    } else {
        runLegacySkillsAuto();
    }
}

static int deadbandInt(int val, int threshold) {
    return (std::abs(val) < threshold) ? 0 : val;
}

constexpr double CD_TURN_NONLINEARITY = 0.5;
constexpr double CD_NEG_INERTIA_SCALAR = 2.5;  // strength of 'flick' boost
constexpr double CD_SENSITIVITY = 1.5;
constexpr double DRIVE_DEADBAND = 0.03;
constexpr double DRIVE_SLEW_UP = 0.05;
constexpr double DRIVE_SLEW_DOWN = 0.10;

static double quickStopAccumlator  = 0.0;
static double negInertiaAccumlator = 0.0;
static double prevTurn = 0.0;
static double prevThrottle = 0.0;

static double turnRemapping(double iturn) {
    double denominator = std::sin(PI / 2.0 * CD_TURN_NONLINEARITY);
    if (denominator == 0.0) return iturn; 
    double first = std::sin(PI / 2.0 * CD_TURN_NONLINEARITY * iturn) / denominator;
    return std::sin(PI / 2.0 * CD_TURN_NONLINEARITY * first) / denominator;
}

static void updateAccumulators() {
    if (negInertiaAccumlator > 1.0) {
        negInertiaAccumlator -= 1.0;
    } 
    else if (negInertiaAccumlator < -1.0) {
        negInertiaAccumlator += 1.0;
    } 
    else {
        negInertiaAccumlator = 0.0;
    }

    if (quickStopAccumlator > 1.0) {
        quickStopAccumlator -= 1.0;
    } 
    else if (quickStopAccumlator < -1.0) {
        quickStopAccumlator += 1.0;
    } 
    else {
        quickStopAccumlator = 0.0;
    }
}

static std::pair<double, double> cheesyArcade(double ithrottle, double iturn) {
    bool turnInPlace = false;
    double linearCmd = ithrottle;

    if (std::fabs(ithrottle) < DRIVE_DEADBAND && std::fabs(iturn) > DRIVE_DEADBAND) {
        linearCmd  = 0.0;
        turnInPlace = true;
    }
    else {
        double delta = ithrottle - prevThrottle;
        if (delta > DRIVE_SLEW_UP) {
            linearCmd = prevThrottle + DRIVE_SLEW_UP;
        } 
        else if (delta < -DRIVE_SLEW_DOWN) {
            linearCmd = prevThrottle - DRIVE_SLEW_DOWN;
        } 
        else {
            linearCmd = ithrottle;
        }
    }

    double remappedTurn = turnRemapping(iturn);

    double forwardOut;
    double turnOut;

    if (turnInPlace) {
        double x = remappedTurn;
        forwardOut = 0.0;
        turnOut = x * std::fabs(x); 
    } 
    else {
        double negInertiaPower = (iturn - prevTurn) * CD_NEG_INERTIA_SCALAR;
        negInertiaAccumlator += negInertiaPower;

        double speedFactor = std::fabs(linearCmd);
        if (speedFactor < 0.5) {
            speedFactor = 0.5;
        }
        
        double angularCmd = speedFactor * (remappedTurn + negInertiaAccumlator) * CD_SENSITIVITY - quickStopAccumlator;

        forwardOut = linearCmd;
        turnOut = angularCmd;

        updateAccumulators();
    }

    prevTurn = iturn;
    prevThrottle = linearCmd;

    forwardOut = std::fmax(-1.0, std::fmin(1.0, forwardOut));
    turnOut = std::fmax(-1.0, std::fmin(1.0, turnOut));

    return {forwardOut, turnOut};
}

void opcontrol() {
    chassis.cancelAllMotions();
    commandScheduler.cancelAll();

    bool flagStateTongue = false;
    bool flagStateAligner = false;
    robot::Pose2D teleopLastPose = readOdomPose();
    while (true) {
        auto odomPose = readOdomPose();
        const double dxField = odomPose.x - teleopLastPose.x;
        const double dyField = odomPose.y - teleopLastPose.y;
        const double prevHeading = teleopLastPose.theta;
        robot::MotionDelta delta;
        delta.dx = dxField * std::cos(prevHeading) + dyField * std::sin(prevHeading);
        delta.dy = -dxField * std::sin(prevHeading) + dyField * std::cos(prevHeading);
        delta.dtheta = wrapAngle(odomPose.theta - teleopLastPose.theta);
        localizer.predict(delta, {0.1, 0.1, degToRad(0.4)});
        localizer.applyHeadingObservation({odomPose.theta, degToRad(1.0)});
        double wallDistance = 0.0;
        if (readWallDistanceInches(wallDistance)) {
            localizer.applyWallDistanceObservation(wallDistance, FIELD_WALLS, WALL_DISTANCE_MODEL);
        }
        localizer.resample();
        teleopLastPose = odomPose;

        int rawFwd = controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
        int rawTurn = controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);

        rawFwd = deadbandInt(rawFwd, 5);
        rawTurn = deadbandInt(rawTurn, 5);

        double ithrottle = rawFwd  / 127.0;
        double iturn = rawTurn / 127.0;

        auto driveCmd = cheesyArcade(ithrottle, iturn);
        double fwdCmd = driveCmd.first;
        double turnCmd = driveCmd.second;

        int fwdPower = static_cast<int>(fwdCmd  * 127.0);
        int turnPower  = static_cast<int>(turnCmd * 127.0);

        chassis.arcade(fwdPower, turnPower);

        bool r1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R1);
        bool r2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_R2);
        bool l1 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L1);
        bool l2 = controller.get_digital(pros::E_CONTROLLER_DIGITAL_L2);

        if (r1 && r2) {
            intake.move(127);
            scoring.move(-127);
            indexer.move(0);
        }
        else if (l1 && l2) {
            intake.move(0);
            scoring.move(-127);
            indexer.move(0);
        }
        else {
            if (r2) {
                intake.move(127);
            }
            else if (r1) {
                intake.move(-127);
            }
            else {
                intake.move(0);
            }

            if (l2) {
                scoring.move(127);
                indexer.move(127);
            }
            else if (l1) {
                scoring.move(127);
                indexer.move(-127);
            }
            else {
                scoring.move(0);
                indexer.move(0);
            }
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_A)) {
            flagStateTongue = !flagStateTongue;
            if (flagStateTongue) 
                tongue.extend();
            else 
                tongue.retract();
        }

        if (controller.get_digital_new_press(pros::E_CONTROLLER_DIGITAL_LEFT)) {
            flagStateAligner = !flagStateAligner;
            if (flagStateAligner) 
                aligner.extend();
            else 
                aligner.retract();
        }

        pros::delay(10);
    }
}
