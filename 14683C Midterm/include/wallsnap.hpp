#pragma once

namespace wallsnap {

void updateDistanceFilters();
bool wallSnapX();
bool wallSnapY();
bool wallSnapXY();
void zeroHeadingToNearestCardinalIfSnapped();

double filteredRight();
double filteredBack();

}

