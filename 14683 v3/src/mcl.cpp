#include "mcl.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>

namespace robot {

namespace {
constexpr double PI = 3.14159265358979323846;
double gaussian(double error, double stdDev) {
    const double variance = stdDev * stdDev;
    const double norm = std::sqrt(2.0 * PI * variance);
    if (norm < 1e-9) return 1.0;
    return std::exp(-(error * error) / (2.0 * variance)) / norm;
}
}  // namespace

MonteCarloLocalizer::MonteCarloLocalizer(int particleCount, unsigned int seed) : rng_(seed) {
    particles_.reserve(std::max(1, particleCount));
}

void MonteCarloLocalizer::seedParticles(const Pose2D& mean, double stdX, double stdY, double stdTheta) {
    std::normal_distribution<double> distX(mean.x, stdX);
    std::normal_distribution<double> distY(mean.y, stdY);
    std::normal_distribution<double> distTheta(mean.theta, stdTheta);

    particles_.clear();
    const int count = static_cast<int>(particles_.capacity());
    for (int i = 0; i < count; ++i) {
        particles_.push_back({{distX(rng_), distY(rng_), wrap(distTheta(rng_))}, 1.0});
    }
    normalize();
}

void MonteCarloLocalizer::predict(const MotionDelta& delta, const MotionNoise& noise) {
    std::normal_distribution<double> noiseX(0.0, noise.stdX);
    std::normal_distribution<double> noiseY(0.0, noise.stdY);
    std::normal_distribution<double> noiseTheta(0.0, noise.stdTheta);

    for (auto& particle : particles_) {
        const double theta = particle.pose.theta;
        const double dxField = delta.dx * std::cos(theta) - delta.dy * std::sin(theta);
        const double dyField = delta.dx * std::sin(theta) + delta.dy * std::cos(theta);
        particle.pose.x += dxField + noiseX(rng_);
        particle.pose.y += dyField + noiseY(rng_);
        particle.pose.theta = wrap(particle.pose.theta + delta.dtheta + noiseTheta(rng_));
    }
}

void MonteCarloLocalizer::applyHeadingObservation(const HeadingObservation& obs) {
    for (auto& particle : particles_) {
        const double err = wrap(obs.heading - particle.pose.theta);
        const double likelihood = gaussian(err, obs.stdDev);
        particle.weight *= likelihood;
    }
    normalize();
}

void MonteCarloLocalizer::applyRangeObservation(const RangeObservation& obs) {
    for (auto& particle : particles_) {
        const double dx = obs.landmarkX - particle.pose.x;
        const double dy = obs.landmarkY - particle.pose.y;
        const double predicted = std::sqrt(dx * dx + dy * dy);
        const double error = obs.measuredDistance - predicted;
        particle.weight *= gaussian(error, obs.stdDev);
    }
    normalize();
}

void MonteCarloLocalizer::normalize() {
    double weightSum = 0.0;
    for (const auto& particle : particles_) weightSum += particle.weight;
    if (weightSum < 1e-9) {
        const double resetWeight = 1.0 / std::max<std::size_t>(particles_.size(), 1);
        for (auto& particle : particles_) particle.weight = resetWeight;
        weightSum = 1.0;
    } else {
        for (auto& particle : particles_) particle.weight /= weightSum;
    }

    double x = 0.0, y = 0.0, sinTheta = 0.0, cosTheta = 0.0;
    for (const auto& particle : particles_) {
        x += particle.pose.x * particle.weight;
        y += particle.pose.y * particle.weight;
        sinTheta += std::sin(particle.pose.theta) * particle.weight;
        cosTheta += std::cos(particle.pose.theta) * particle.weight;
    }
    estimate_ = {x, y, std::atan2(sinTheta, cosTheta)};
}

void MonteCarloLocalizer::resample() {
    std::vector<Particle> newParticles;
    newParticles.reserve(particles_.size());

    double step = 1.0 / std::max<double>(particles_.size(), 1.0);
    double r = uniform_(rng_) * step;
    double c = particles_.front().weight;
    std::size_t i = 0;
    for (std::size_t m = 0; m < particles_.size(); ++m) {
        double u = r + m * step;
        while (u > c && i < particles_.size() - 1) {
            ++i;
            c += particles_[i].weight;
        }
        newParticles.push_back(particles_[i]);
        newParticles.back().weight = 1.0;
    }
    particles_ = std::move(newParticles);
    normalize();
}

double MonteCarloLocalizer::wrap(double angle) const {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

}  // namespace robot
