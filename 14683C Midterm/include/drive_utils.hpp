#pragma once

namespace drive_utils {

void reset();
void update(double leftTargetPct, double rightTargetPct);
double leftOutput();
double rightOutput();
void setDriveMode(bool braking);

}

