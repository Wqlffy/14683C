#pragma once

#include "../../auton_selector.hpp"
#include "liblvgl/lvgl.h"

namespace ui_autons {
lv_obj_t* build(lv_obj_t* parent);
void set_selected(AutonId id);
void set_detail(AutonId id);
void update();
}
