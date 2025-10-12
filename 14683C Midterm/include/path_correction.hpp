#pragma once

namespace path_correction {

void reset(double targetHeadingDeg);
void setTarget(double targetHeadingDeg);
void enable(bool on);
bool active();
double compute(double currentHeadingDeg, double forwardCmdPct);

}

