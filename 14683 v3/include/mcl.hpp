#pragma once

#include <random>
#include <utility>
#include <vector>

namespace robot {

struct Pose2D {
    double x;
    double y;
    double theta;
};

struct MotionDelta {
    double dx;
    double dy;
    double dtheta;
};

struct MotionNoise {
    double stdX;
    double stdY;
    double stdTheta;
};

struct RangeObservation {
    double landmarkX;
    double landmarkY;
    double measuredDistance;
    double stdDev;
};

struct FieldWalls {
    double minX;
    double maxX;
    double minY;
    double maxY;
};

struct WallDistanceSensor {
    double offsetX;     // inches, robot forward is +x
    double offsetY;     // inches, robot left is +y
    double yawOffset;   // radians, relative to robot forward
    double maxRange;    // inches, sensor clipping distance
    double stdDev;      // inches, measurement standard deviation
};

struct HeadingObservation {
    double heading;
    double stdDev;
};

class MonteCarloLocalizer {
  public:
    MonteCarloLocalizer(int particleCount, unsigned int seed = std::random_device{}());
    void seedParticles(const Pose2D& mean, double stdX, double stdY, double stdTheta);
    void predict(const MotionDelta& delta, const MotionNoise& noise);
    void applyHeadingObservation(const HeadingObservation& obs);
    void applyRangeObservation(const RangeObservation& obs);
    double expectedWallDistance(const Pose2D& particlePose, const FieldWalls& walls,
                                const WallDistanceSensor& sensor) const;
    void applyWallDistanceObservation(double measuredDistance, const FieldWalls& walls,
                                      const WallDistanceSensor& sensor);
    void resample();
    Pose2D getEstimate() const { return estimate_; }

  private:
    struct Particle {
        Pose2D pose;
        double weight;
    };

    void normalize();
    double wrap(double angle) const;

    std::vector<Particle> particles_;
    Pose2D estimate_{0.0, 0.0, 0.0};
    mutable std::mt19937 rng_;
    std::uniform_real_distribution<double> uniform_{0.0, 1.0};
};

}