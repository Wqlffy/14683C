#pragma once

#include <cstddef>

#include "liblvgl/lvgl.h"
#include "main.h"

namespace ui_motors {
enum class MotorMode : uint8_t {
    Temp,
    Current,
    Power,
    Volts,
    Velocity,
    Torque
};

struct MotorEntry {
    const char* name;
    pros::Motor motor;
};

lv_obj_t* build(lv_obj_t* parent);
void update();
void set_mode(MotorMode mode);
// Call before build() to replace the default motor list.
void set_motor_entries(const MotorEntry* entries, size_t count);
}
