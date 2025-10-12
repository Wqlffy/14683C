#pragma once

namespace mech_scoring {

void scoreMidStart();
void scoreLongStart();
void purgeStart(int pct);
void jamClearStart(int pct, int ms);
bool scoringActive();
void update();

}

