#pragma once

#include "liblvgl/lvgl.h"

namespace ui_sensors {
lv_obj_t* build(lv_obj_t* parent);
void update();
}
