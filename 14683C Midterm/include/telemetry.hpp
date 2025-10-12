#pragma once

#include "pros/rtos.hpp"

namespace telemetry {

void init();
void setEnabled(bool on);
void update();
void logEvent(const char* tag);

}

