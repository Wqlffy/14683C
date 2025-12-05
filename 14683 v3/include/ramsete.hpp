#pragma once

#include <functional>
#include <utility>
#include <vector>

#include "command_framework.hpp"
#include "mcl.hpp"
#include "motion_profile.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"

namespace robot {

struct ChassisSpeeds {
    double linear;
    double angular;
};

class RamseteController {
  public:
    RamseteController(double beta, double zeta) : beta_(beta), zeta_(zeta) {}
    ChassisSpeeds calculate(const Pose2D& current, const Pose2D& reference, double referenceVel,
                            double referenceOmega) const;

  private:
    double beta_;
    double zeta_;
};

class RamseteFollower {
  public:
    RamseteFollower(double trackWidth, double wheelDiameter, double maxRpm,
                    double beta = 2.0, double zeta = 0.7, double maxVoltage = 12000.0);

    void setTrajectory(std::vector<TrajectorySample> trajectory);
    ChassisSpeeds calculate(const Pose2D& current, double timeSec) const;
    std::pair<double, double> chassisToWheelSpeeds(const ChassisSpeeds& speeds) const;
    std::pair<double, double> wheelSpeedsToVolts(const std::pair<double, double>& wheelSpeeds) const;
    double totalTime() const;

  private:
    TrajectorySample sampleReference(double timeSec) const;

    std::vector<TrajectorySample> trajectory_;
    double trackWidth_;
    double wheelDiameter_;
    double maxRpm_;
    double maxVoltage_;
    RamseteController controller_;
};

class RamsetePathCommand : public Command {
  public:
    RamsetePathCommand(std::vector<TrajectorySample> trajectory, MonteCarloLocalizer& localizer,
                       std::function<Pose2D()> odomGetter,
                       std::function<void(double, double)> voltageOutput, double trackWidth,
                       double wheelDiameter, double maxRpm,
                       MotionNoise noise = {0.25, 0.25, 0.02}, double headingStd = 0.03);

    void initialize() override;
    void execute() override;
    bool isFinished() const override;
    void end(bool interrupted) override;

  private:
    RamseteFollower follower_;
    MonteCarloLocalizer& localizer_;
    std::function<Pose2D()> odomGetter_;
    std::function<void(double, double)> output_;
    MotionNoise noise_;
    double headingStd_;
    uint32_t startMs_{0};
    Pose2D lastOdom_{};
    bool seeded_{false};
};

}  // namespace robot
