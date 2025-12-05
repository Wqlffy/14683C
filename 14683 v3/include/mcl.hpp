#pragma once

#include <random>
#include <utility>
#include <vector>

namespace robot {

struct Pose2D {
    double x;
    double y;
    double theta;  // radians
};

struct MotionDelta {
    double dx;
    double dy;
    double dtheta;  // radians
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

}  // namespace robot
