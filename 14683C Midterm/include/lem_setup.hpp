#pragma once

#include "nav_params.hpp"
#include "pros/imu.hpp"
#include "pros/motor_group.hpp"
#include "pros/rtos.hpp"
#include "pros/vision.hpp"
#include "lemlib/api.hpp"

namespace lem_setup {

struct PoseMM {
  double x;
  double y;
  double thetaDeg;
};

bool calibrateAndInit();
void setPoseMM(double x, double y, double thetaDeg);
PoseMM getPoseMM();
lemlib::Chassis& chassis();
pros::IMU& imu();
bool imuReady();

}

