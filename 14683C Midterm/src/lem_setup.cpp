#include "lem_setup.hpp"

#include "config.hpp"
#include "nav_params.hpp"
#include "pros/misc.hpp"
#include "pros/rtos.hpp"

namespace {
pros::IMU imu_sensor(21);

lemlib::OdomSensors sensors{
    nullptr, nullptr, nullptr, nullptr,
    &imu_sensor
};

lemlib::Drivetrain drivetrain{
    &config::leftDrive,
    &config::rightDrive,
    static_cast<float>(nav_params::TRACK_WIDTH_MM / 25.4),  // convert mm to inches for lemlib
    static_cast<float>(nav_params::DRIVE_WHEEL_DIAMETER_MM / 25.4),
    nav_params::DRIVE_MAX_SPEED_MMPS,
    nav_params::DRIVE_MAX_ACCEL_MMPS2
};

lemlib::ControllerSettings linear_settings{
    static_cast<float>(nav_params::LATERAL_KP),
    static_cast<float>(nav_params::LATERAL_KI),
    static_cast<float>(nav_params::LATERAL_KD),
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};

lemlib::ControllerSettings angular_settings{
    static_cast<float>(nav_params::ANGULAR_KP),
    static_cast<float>(nav_params::ANGULAR_KI),
    static_cast<float>(nav_params::ANGULAR_KD),
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f,
    0.0f
};

lemlib::Chassis chassis_obj{
    drivetrain,
    linear_settings,
    angular_settings,
    sensors
};

lem_setup::PoseMM last_pose{0.0, 0.0, 0.0};
bool imu_ready = false;

}  // namespace

namespace lem_setup {

bool calibrateAndInit() {
  imu_ready = false;
  imu_sensor.reset();
  const std::uint32_t start = pros::c::millis();
  while (imu_sensor.is_calibrating()) {
    pros::delay(10);
    if (pros::c::millis() - start > 6000) {
      return false;
    }
  }
  chassis_obj.calibrate(false);
  imu_ready = true;
  chassis_obj.setPose(0.0f, 0.0f, 0.0f);
  return true;
}

void setPoseMM(double x, double y, double thetaDeg) {
  last_pose = {x, y, thetaDeg};
  chassis_obj.setPose(static_cast<float>(x / 25.4), static_cast<float>(y / 25.4), static_cast<float>(thetaDeg));
}

PoseMM getPoseMM() {
  const auto pose = chassis_obj.getPose();
  const double imu_heading = imu_ready ? imu_sensor.get_heading() : pose.theta;
  last_pose = {pose.x * 25.4, pose.y * 25.4, imu_heading};
  return last_pose;
}

lemlib::Chassis& chassis() {
  return chassis_obj;
}

pros::IMU& imu() {
  return imu_sensor;
}

bool imuReady() {
  return imu_ready;
}

}
