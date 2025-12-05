#pragma once

#include <vector>

#include "mcl.hpp"

namespace robot {

struct MotionProfilePoint {
    double time;        // seconds
    double position;    // inches
    double velocity;    // inches/second
    double acceleration;  // inches/second^2
};

struct Waypoint {
    double x;
    double y;
    double heading;  // radians
};

struct TrajectorySample {
    double time;
    Pose2D pose;
    double linearVelocity;
    double angularVelocity;
    double curvature;
};

class MotionProfileGenerator {
  public:
    static std::vector<MotionProfilePoint> trapezoid(double distance, double maxVel, double maxAccel,
                                                     double dt);
};

std::vector<TrajectorySample> generateTrajectory(const std::vector<Waypoint>& waypoints,
                                                 double maxVel, double maxAccel, double dt);

}  // namespace robot
