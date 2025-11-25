#ifndef _PROS_MAIN_H_
#define _PROS_MAIN_H_

#define PROS_USE_SIMPLE_NAMES
#define PROS_USE_LITERALS

#include "api.h"
#include "lemlib/api.hpp"

// ------------------------------
// GLOBAL AUTON SELECTION
// ------------------------------
#ifdef __cplusplus
extern int selectedAuton;              // set by LVGL auton selector
extern lemlib::Chassis chassis;        // your chassis object from chassis.cpp
#endif

// ------------------------------
// COMPETITION TEMPLATE FUNCTIONS
// ------------------------------
#ifdef __cplusplus
extern "C" {
#endif

void autonomous(void);
void initialize(void);
void disabled(void);
void competition_initialize(void);
void opcontrol(void);

// LVGL auton selector builder
void build_auton_selector(void);

#ifdef __cplusplus
}
#endif

// ------------------------------
// C++ AUTON ROUTINES
// ------------------------------
#ifdef __cplusplus

// Normal match autons
void auton_red_left();
void auton_red_right();
void auton_blue_left();
void auton_blue_right();

// AWP autons
void auton_red_left_awp();
void auton_red_right_awp();
void auton_blue_left_awp();
void auton_blue_right_awp();

// Skills
void auton_skills();

#endif

#endif  // _PROS_MAIN_H_
