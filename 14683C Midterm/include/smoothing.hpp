#pragma once

#include <cstdint>

namespace smoothing {

void setAutonSlewStepPct(double pct);
double applySlew(double targetPct, double prevPct);
void precisionModeOn();
void precisionModeOff();
void moveToProfile(double x_mm, double y_mm, double heading_deg, int timeout_ms, int max_speed_mmps,
                   int max_accel_mmps2, bool async);
void turnToProfile(double x_mm, double y_mm, int timeout_ms, int max_speed_mmps,
                   int max_accel_mmps2, bool async);
void clampPIDOutputNearTarget(bool enabled);
bool precisionMode();

}

