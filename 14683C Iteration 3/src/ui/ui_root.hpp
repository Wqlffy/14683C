#pragma once

#include <cstdint>

namespace ui_root {
enum class Page : std::uint8_t { Autons, Motors, Sensors, Odom };

void init();
void update_fast();
Page active_page();
}
