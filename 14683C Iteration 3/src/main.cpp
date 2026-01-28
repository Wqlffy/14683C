#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "lemlib/chassis/trackingWheel.hpp"
#include "auton_selector.hpp"
#include "liblvgl/lvgl.h"
#include "pros/adi.hpp"
#include "pros/misc.h"
#include "pros/screen.hpp"
#include "robot_config.hpp"
#include "pros/motors.h"
#include "ui/ui_root.hpp"

#include <algorithm>
#include <cstdint>
#include <cstdio>
#include <cmath>
#include <utility>

constexpr double JOYSTICK_SCALE = 127.0;
constexpr double DEADBAND = 0.05;
constexpr double THROTTLE_EXPO = 1.6;
constexpr double TURN_EXPO = 1.8;
constexpr double TURN_AT_FULL = 0.55;
constexpr double BASE_STEER = 0.15;
constexpr double TURN_IN_PLACE_GAIN = 0.75;
constexpr double TURN_IN_PLACE_THROTTLE = 0.10;

constexpr std::uint32_t kLvTickMs = 5;
constexpr std::uint32_t kUiUpdateMs = 100;
constexpr std::uint32_t kTouchLogMs = 250;

#define UI_TOUCH_DEBUG 1

static double clamp(double value, double minValue, double maxValue) {
    return std::max(minValue, std::min(maxValue, value));
}

static double apply_deadband_rescale(double value, double db) {
    const double mag = std::abs(value);
    if (mag <= db) {
        return 0.0;
    }
    const double scaled = (mag - db) / (1.0 - db);
    return std::copysign(scaled, value);
}

static double expo(double value, double e) {
    const double mag = std::pow(std::abs(value), e);
    return std::copysign(mag, value);
}

static std::pair<double, double> arcDrive(double throttle, double turn) {
    throttle = apply_deadband_rescale(throttle, DEADBAND);
    turn = apply_deadband_rescale(turn, DEADBAND);

    throttle = expo(throttle, THROTTLE_EXPO);
    turn = expo(turn, TURN_EXPO);

    double left = 0.0;
    double right = 0.0;

    if (std::abs(throttle) < TURN_IN_PLACE_THROTTLE) {
        const double angular = turn * TURN_IN_PLACE_GAIN;
        left = angular;
        right = -angular;
    } else {
        const double angular =
            turn * (TURN_AT_FULL * std::abs(throttle) + BASE_STEER);
        left = throttle + angular;
        right = throttle - angular;
    }

    const double maxMag = std::max(std::abs(left), std::abs(right));
    if (maxMag > 1.0) {
        left /= maxMag;
        right /= maxMag;
    }

    left = clamp(left, -1.0, 1.0);
    right = clamp(right, -1.0, 1.0);
    return {left, right};
}

extern "C" {
namespace pros {
namespace c {
void display_mutex_take(void) __attribute__((weak));
void display_mutex_give(void) __attribute__((weak));
}
}
}

static lv_indev_t* s_touch_indev = nullptr;

#if UI_TOUCH_DEBUG
static std::uint32_t s_touch_press_count = 0;
static std::uint32_t s_touch_release_count = 0;
static std::int16_t s_touch_last_x = -1;
static std::int16_t s_touch_last_y = -1;
static lv_indev_state_t s_touch_last_state = LV_INDEV_STATE_RELEASED;
static std::uint32_t s_touch_last_log = 0;
static std::uint32_t s_lvgl_last_log = 0;
#endif

static void touch_read_cb(lv_indev_t* indev, lv_indev_data_t* data) {
    (void)indev;
    const pros::screen_touch_status_s_t status =
        pros::screen::touch_status();
    const bool pressed = (status.touch_status == pros::E_TOUCH_PRESSED ||
                          status.touch_status == pros::E_TOUCH_HELD);

    data->state = pressed ? LV_INDEV_STATE_PRESSED
                          : LV_INDEV_STATE_RELEASED;
    if (status.x >= 0 && status.y >= 0) {
        data->point.x = status.x;
        data->point.y = status.y;
#if UI_TOUCH_DEBUG
        s_touch_last_x = status.x;
        s_touch_last_y = status.y;
#endif
    } else {
        data->point.x = (s_touch_last_x >= 0) ? s_touch_last_x : 0;
        data->point.y = (s_touch_last_y >= 0) ? s_touch_last_y : 0;
    }

#if UI_TOUCH_DEBUG
    if (data->state != s_touch_last_state) {
        if (data->state == LV_INDEV_STATE_PRESSED) {
            ++s_touch_press_count;
        } else {
            ++s_touch_release_count;
        }
    }
    s_touch_last_state = data->state;
    const std::uint32_t now = pros::millis();
    if (now - s_touch_last_log >= kTouchLogMs) {
        std::printf("[ui] touch state=%d press=%lu release=%lu x=%d y=%d\n",
                    static_cast<int>(data->state),
                    static_cast<unsigned long>(s_touch_press_count),
                    static_cast<unsigned long>(s_touch_release_count),
                    static_cast<int>(data->point.x),
                    static_cast<int>(data->point.y));
        s_touch_last_log = now;
    }
#endif
}

static void init_touch_indev() {
    if (s_touch_indev) {
        return;
    }
    s_touch_indev = lv_indev_create();
    lv_indev_set_type(s_touch_indev, LV_INDEV_TYPE_POINTER);
    lv_indev_set_read_cb(s_touch_indev, touch_read_cb);
    lv_indev_set_display(s_touch_indev, lv_display_get_default());
}

pros::Controller master(pros::E_CONTROLLER_MASTER);

lemlib::Drivetrain drivetrain(&leftMotors,
                              &rightMotors, 
                              11, 
                              lemlib::Omniwheel::NEW_325, 
                              450, 
                              2 
);

lemlib::ControllerSettings linearController(10, // proportional gain (kP)
                                            0, // integral gain (kI)
                                            3, // derivative gain (kD)
                                            3, // anti windup
                                            1, // small error range, in inches
                                            100, // small error range timeout, in milliseconds
                                            3, // large error range, in inches
                                            500, // large error range timeout, in milliseconds
                                            20 // maximum acceleration (slew)
);

lemlib::ControllerSettings angularController(2, // proportional gain (kP)
                                             0, // integral gain (kI)
                                             10, // derivative gain (kD)
                                             3, // anti windup
                                             1, // small error range, in degrees
                                             100, // small error range timeout, in milliseconds
                                             3, // large error range, in degrees
                                             500, // large error range timeout, in milliseconds
                                             0 // maximum acceleration (slew)
);

lemlib::OdomSensors sensors(nullptr,
                            nullptr,
                            nullptr,
                            nullptr, 
                            &imu 
);

lemlib::ExpoDriveCurve throttleCurve(3, // joystick deadband out of 127
                                     10, // minimum output where drivetrain will move out of 127
                                     1.019 // expo curve gain
);

lemlib::ExpoDriveCurve steerCurve(3, // joystick deadband out of 127
                                  10, // minimum output where drivetrain will move out of 127
                                  1.019 // expo curve gain
);

lemlib::Chassis chassis(drivetrain, linearController, angularController, sensors, &throttleCurve, &steerCurve);

void initialize() {
    leftMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);
    rightMotors.set_brake_mode_all(pros::E_MOTOR_BRAKE_COAST);

    matchloader.set_value(false);
    wing.set_value(false);
    middescore.set_value(false);
    midgoal.set_value(true);
    pros::delay(30);
    
    chassis.calibrate();
    load_auton_state();

    ui_root::init();
    init_touch_indev();
    static pros::Task ui_task([] {
        std::uint32_t last_ui = pros::millis();
        while (true) {
            lv_tick_inc(kLvTickMs);
            pros::c::display_mutex_take();
            lv_timer_handler();
            pros::c::display_mutex_give();

            const std::uint32_t now = pros::millis();
#if UI_TOUCH_DEBUG
            if (now - s_lvgl_last_log >= 1000) {
                std::printf("[ui] lvgl tick running press=%lu release=%lu x=%d y=%d\n",
                            static_cast<unsigned long>(s_touch_press_count),
                            static_cast<unsigned long>(s_touch_release_count),
                            static_cast<int>(s_touch_last_x),
                            static_cast<int>(s_touch_last_y));
                s_lvgl_last_log = now;
            }
#endif
            if (now - last_ui >= kUiUpdateMs) {
                ui_root::update_fast();
                last_ui = now;
            }
            pros::delay(kLvTickMs);
        }
    });
}
void disabled() {

}
void competition_initialize() {
    
}
void autonomous() {
    run_selected_auton();
}


void opcontrol() {
    chassis.cancelAllMotions();
    bool flagStateLoader = false;
    bool flagStateWing = false;
    bool flagStateDescore = false;

    bool lastA = false;
    bool lastB = false;
    bool lastDown = false;

    while (true) {
        double throttle = master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y) / JOYSTICK_SCALE;
        double turn = master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X) / JOYSTICK_SCALE;
        auto [left, right] = arcDrive(throttle, turn);
        leftMotors.move(static_cast<int>(left * JOYSTICK_SCALE));
        rightMotors.move(static_cast<int>(right * JOYSTICK_SCALE));

        int intake = 0;
        int outtake = 0;

        if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L2)) {
            intake = 127;
            outtake = 127;
            midgoal.set_value(true); 
        }
        else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_L1)) {
            intake = 127;
            outtake = -127;
            midgoal.set_value(false);
        }
        else {
            if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R1)) {
                intake = 127;
                outtake = 50; 
            }
            else if (master.get_digital(pros::E_CONTROLLER_DIGITAL_R2)) {
                intake = -127;
                outtake = -127;
            }
            midgoal.set_value(false);
        }

        intakeMotor.move(intake);
        outtakeMotor.move(outtake);

        bool currA = master.get_digital(pros::E_CONTROLLER_DIGITAL_A);

        if (currA && !lastA) {
            flagStateLoader = !flagStateLoader;
            if (flagStateLoader)
                matchloader.extend();
            else
                matchloader.retract();
        }

        lastA = currA;

        bool currB = master.get_digital(pros::E_CONTROLLER_DIGITAL_B);

        if (currB && !lastB) {
            flagStateWing = !flagStateWing;
            if (flagStateWing)
                wing.extend();
            else
                wing.retract();
        }

        lastB = currB;

        bool currDown = master.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);

        if (currDown && !lastDown) {
            flagStateDescore = !flagStateDescore;
            if (flagStateDescore)
                middescore.extend();
            else
                middescore.retract();
        }

        lastDown = currDown;

        pros::delay(10);
    }
}
