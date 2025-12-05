#include "motion_profile.hpp"

#include <cmath>
#include <utility>

namespace robot {

namespace {
constexpr double PI = 3.14159265358979323846;

double wrap(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

double lerp(double a, double b, double t) { return a + (b - a) * t; }
}  // namespace

std::vector<MotionProfilePoint> MotionProfileGenerator::trapezoid(double distance, double maxVel,
                                                                  double maxAccel, double dt) {
    std::vector<MotionProfilePoint> profile;
    if (dt <= 0.0 || maxVel <= 0.0 || maxAccel <= 0.0) return profile;

    const double direction = distance >= 0.0 ? 1.0 : -1.0;
    const double totalDistance = std::fabs(distance);
    if (totalDistance < 1e-6) {
        profile.push_back({0.0, 0.0, 0.0, 0.0});
        return profile;
    }

    double accelTime = maxVel / maxAccel;
    double accelDistance = 0.5 * maxAccel * accelTime * accelTime;

    double cruiseTime = 0.0;
    double peakVel = maxVel;
    if (2.0 * accelDistance > totalDistance) {
        accelTime = std::sqrt(totalDistance / maxAccel);
        accelDistance = 0.5 * maxAccel * accelTime * accelTime;
        peakVel = maxAccel * accelTime;
    } else {
        cruiseTime = (totalDistance - 2.0 * accelDistance) / maxVel;
    }

    const double totalTime = 2.0 * accelTime + cruiseTime;

    for (double t = 0.0; t <= totalTime + 1e-6; t += dt) {
        double pos = 0.0;
        double vel = 0.0;
        double acc = 0.0;
        if (t < accelTime) {
            acc = maxAccel;
            vel = acc * t;
            pos = 0.5 * acc * t * t;
        } else if (t < accelTime + cruiseTime) {
            acc = 0.0;
            vel = peakVel;
            pos = accelDistance + peakVel * (t - accelTime);
        } else {
            const double tDecel = t - accelTime - cruiseTime;
            acc = -maxAccel;
            vel = peakVel + acc * tDecel;
            const double tRemaining = totalTime - t;
            pos = totalDistance - 0.5 * maxAccel * tRemaining * tRemaining;
        }
        profile.push_back({t, direction * pos, direction * vel, direction * acc});
    }

    if (profile.empty() || std::fabs(profile.back().time - totalTime) > 1e-5) {
        profile.push_back({totalTime, direction * totalDistance, 0.0, 0.0});
    } else {
        profile.back() = {totalTime, direction * totalDistance, 0.0, 0.0};
    }

    return profile;
}

std::vector<TrajectorySample> generateTrajectory(const std::vector<Waypoint>& waypoints,
                                                 double maxVel, double maxAccel, double dt) {
    std::vector<TrajectorySample> trajectory;
    if (waypoints.size() < 2) return trajectory;

    std::vector<double> segmentLengths;
    segmentLengths.reserve(waypoints.size() - 1);
    double totalDistance = 0.0;
    for (std::size_t i = 0; i + 1 < waypoints.size(); ++i) {
        const double dx = waypoints[i + 1].x - waypoints[i].x;
        const double dy = waypoints[i + 1].y - waypoints[i].y;
        const double len = std::hypot(dx, dy);
        segmentLengths.push_back(len);
        totalDistance += len;
    }
    if (totalDistance < 1e-6) return trajectory;

    const auto profile = MotionProfileGenerator::trapezoid(totalDistance, maxVel, maxAccel, dt);
    trajectory.reserve(profile.size());

    for (const auto& point : profile) {
        double distance = point.position;
        double traveled = 0.0;
        Pose2D pose{waypoints.front().x, waypoints.front().y, waypoints.front().heading};
        double curvature = 0.0;

        for (std::size_t i = 0; i < segmentLengths.size(); ++i) {
            const double segLen = segmentLengths[i];
            const double nextTraveled = traveled + segLen;
            if (distance <= nextTraveled + 1e-6 || i == segmentLengths.size() - 1) {
                const double ratio = segLen > 1e-6 ? (distance - traveled) / segLen : 0.0;
                const auto& start = waypoints[i];
                const auto& end = waypoints[i + 1];
                pose.x = lerp(start.x, end.x, ratio);
                pose.y = lerp(start.y, end.y, ratio);
                const double dHeading = wrap(end.heading - start.heading);
                pose.theta = wrap(start.heading + dHeading * ratio);
                curvature = segLen > 1e-6 ? dHeading / segLen : 0.0;
                break;
            }
            traveled = nextTraveled;
        }

        const double angularVel = point.velocity * curvature;
        trajectory.push_back({point.time, pose, point.velocity, angularVel, curvature});
    }

    return trajectory;
}

}
