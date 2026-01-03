#pragma once

#include <cstdint>

#include "liblvgl/lvgl.h"
#include "main.h"

namespace ui_motors {
enum class MotorMetric : uint8_t {
    Temp,
    Current,
    Power,
    Voltage,
    Velocity,
    Torque
};

struct MotorEntry {
    const char* name;
    pros::AbstractMotor* motor;
    std::uint8_t index;
};

lv_obj_t* build(lv_obj_t* parent);
void update();
}
