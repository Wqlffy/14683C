#pragma once

namespace mech_intake {

void intakeOn(int pct);
void intakeOff();
void intakeReverse(int pct, int ms);
void intakeOneBall(int pct, int ms);
void intakeTaskUpdate();
bool intakeActive();

}

