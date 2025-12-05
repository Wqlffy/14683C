#include "ramsete.hpp"

#include <algorithm>
#include <cmath>

namespace robot {

namespace {
constexpr double PI = 3.14159265358979323846;

double wrap(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

double sinc(double x) {
    if (std::fabs(x) < 1e-6) return 1.0 - (x * x) / 6.0;
    return std::sin(x) / x;
}
}  // namespace

ChassisSpeeds RamseteController::calculate(const Pose2D& current, const Pose2D& reference,
                                           double referenceVel, double referenceOmega) const {
    const double cosRef = std::cos(reference.theta);
    const double sinRef = std::sin(reference.theta);
    const double dx = reference.x - current.x;
    const double dy = reference.y - current.y;

    const double errorX = cosRef * dx + sinRef * dy;
    const double errorY = -sinRef * dx + cosRef * dy;
    const double errorTheta = wrap(reference.theta - current.theta);

    const double k = 2.0 * zeta_ * std::sqrt(referenceOmega * referenceOmega +
                                             beta_ * referenceVel * referenceVel);

    const double linear = referenceVel * std::cos(errorTheta) + k * errorX;
    const double angular = referenceOmega + k * errorTheta + beta_ * referenceVel * sinc(errorTheta) * errorY;
    return {linear, angular};
}

RamseteFollower::RamseteFollower(double trackWidth, double wheelDiameter, double maxRpm,
                                 double beta, double zeta, double maxVoltage)
    : trackWidth_(trackWidth),
      wheelDiameter_(wheelDiameter),
      maxRpm_(maxRpm),
      maxVoltage_(maxVoltage),
      controller_(beta, zeta) {}

void RamseteFollower::setTrajectory(std::vector<TrajectorySample> trajectory) {
    trajectory_ = std::move(trajectory);
}

TrajectorySample RamseteFollower::sampleReference(double timeSec) const {
    if (trajectory_.empty()) return {0.0, {0.0, 0.0, 0.0}, 0.0, 0.0, 0.0};
    if (timeSec <= trajectory_.front().time) return trajectory_.front();
    if (timeSec >= trajectory_.back().time) return trajectory_.back();

    for (std::size_t i = 0; i + 1 < trajectory_.size(); ++i) {
        const auto& a = trajectory_[i];
        const auto& b = trajectory_[i + 1];
        if (timeSec >= a.time && timeSec <= b.time) {
            const double ratio = (timeSec - a.time) / (b.time - a.time);
            Pose2D pose;
            pose.x = a.pose.x + (b.pose.x - a.pose.x) * ratio;
            pose.y = a.pose.y + (b.pose.y - a.pose.y) * ratio;
            const double dTheta = wrap(b.pose.theta - a.pose.theta);
            pose.theta = wrap(a.pose.theta + dTheta * ratio);
            const double linVel = a.linearVelocity + (b.linearVelocity - a.linearVelocity) * ratio;
            const double angVel = a.angularVelocity + (b.angularVelocity - a.angularVelocity) * ratio;
            const double curvature = a.curvature + (b.curvature - a.curvature) * ratio;
            return {timeSec, pose, linVel, angVel, curvature};
        }
    }
    return trajectory_.back();
}

ChassisSpeeds RamseteFollower::calculate(const Pose2D& current, double timeSec) const {
    const auto reference = sampleReference(timeSec);
    return controller_.calculate(current, reference.pose, reference.linearVelocity,
                                 reference.angularVelocity);
}

std::pair<double, double> RamseteFollower::chassisToWheelSpeeds(const ChassisSpeeds& speeds) const {
    const double left = speeds.linear - speeds.angular * (trackWidth_ / 2.0);
    const double right = speeds.linear + speeds.angular * (trackWidth_ / 2.0);
    return {left, right};
}

std::pair<double, double> RamseteFollower::wheelSpeedsToVolts(
    const std::pair<double, double>& wheelSpeeds) const {
    const double maxLinearSpeed = (wheelDiameter_ * PI) * (maxRpm_ / 60.0);
    const double clampedMax = std::max(maxLinearSpeed, 1.0);
    const auto toVolt = [&](double speed) {
        double normalized = std::clamp(speed / clampedMax, -1.0, 1.0);
        return normalized * maxVoltage_;
    };
    return {toVolt(wheelSpeeds.first), toVolt(wheelSpeeds.second)};
}

double RamseteFollower::totalTime() const {
    if (trajectory_.empty()) return 0.0;
    return trajectory_.back().time;
}

RamsetePathCommand::RamsetePathCommand(std::vector<TrajectorySample> trajectory,
                                       MonteCarloLocalizer& localizer,
                                       std::function<Pose2D()> odomGetter,
                                       std::function<void(double, double)> voltageOutput,
                                       double trackWidth, double wheelDiameter, double maxRpm,
                                       MotionNoise noise, double headingStd)
    : follower_(trackWidth, wheelDiameter, maxRpm),
      localizer_(localizer),
      odomGetter_(std::move(odomGetter)),
      output_(std::move(voltageOutput)),
      noise_(noise),
      headingStd_(headingStd) {
    follower_.setTrajectory(std::move(trajectory));
}

void RamsetePathCommand::initialize() {
    startMs_ = pros::millis();
    seeded_ = false;
}

void RamsetePathCommand::execute() {
    const auto odom = odomGetter_();
    if (!seeded_) {
        localizer_.seedParticles(odom, 2.0, 2.0, 0.05);
        seeded_ = true;
        lastOdom_ = odom;
    }

    MotionDelta delta{odom.x - lastOdom_.x, odom.y - lastOdom_.y, wrap(odom.theta - lastOdom_.theta)};
    localizer_.predict(delta, noise_);
    localizer_.applyHeadingObservation({odom.theta, headingStd_});
    localizer_.resample();
    lastOdom_ = odom;

    const double elapsed = (pros::millis() - startMs_) / 1000.0;
    const auto estimate = localizer_.getEstimate();
    const auto speeds = follower_.calculate(estimate, elapsed);
    const auto wheelSpeeds = follower_.chassisToWheelSpeeds(speeds);
    const auto volts = follower_.wheelSpeedsToVolts(wheelSpeeds);
    output_(volts.first, volts.second);
}

bool RamsetePathCommand::isFinished() const {
    const double elapsed = (pros::millis() - startMs_) / 1000.0;
    return elapsed >= follower_.totalTime();
}

void RamsetePathCommand::end(bool interrupted) {
    (void)interrupted;
    output_(0.0, 0.0);
}

}  // namespace robot
