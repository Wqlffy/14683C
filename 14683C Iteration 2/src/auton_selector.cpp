#include "liblvgl/display/lv_display.h"
#include "liblvgl/font/lv_font.h"
#include "liblvgl/lv_conf_internal.h"
#include "liblvgl/widgets/button/lv_button.h"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include "pros/rtos.hpp"
#include <cstdio>
#include <cmath>
#include <cstdint>

extern lemlib::Chassis chassis;

extern const lv_image_dsc_t img_14683C;

extern pros::Controller controller;

// Global selected auton (default to 1, e.g. "Red Left")
int selectedAuton = 1;
int autonCursor = 1;

static lv_obj_t* label_title = nullptr;
static lv_obj_t* label_pose  = nullptr;
static bool poseTimerStarted = false;

static const char* get_auton_name(int id) {
    switch (id) {
        case 1: return "Red Left";
        case 2: return "Red Right";
        case 3: return "Blue Left";
        case 4: return "Blue Right";
        default: return "None";
    }
}
static void update_title() {
    if (!label_title) return;

    const char* name = get_auton_name(selectedAuton);

    static char buf[32];
    std::snprintf(buf, sizeof(buf), "Auton: %s", name);
    lv_label_set_text(label_title, buf);
}

static void pose_timer_cb(lv_timer_t* timer) {
    (void)timer;

    if (!label_pose) return;

    auto pose = chassis.getPose();
    double theta_deg = pose.theta * 180.0 / M_PI;

    static char buf[64];
    std::snprintf(
        buf,
        sizeof(buf),
        "Pose: x=%.1f  y=%.1f  θ=%.1f°",
        pose.x,
        pose.y,
        theta_deg
    );

    lv_label_set_text(label_pose, buf);
}

static void auton_btn_event_cb(lv_event_t* e) {
    int id = (int)(intptr_t)lv_event_get_user_data(e);

    selectedAuton = id;
    autonCursor = id;

    update_title();
    controller.rumble("-.");
}

void auton_controller_task(void* param) {
    (void)param;

    bool lastUp = false, lastDown = false, lastX = false;

    while (true) {
        const char* name = get_auton_name(autonCursor);
        char buf[20];
        std::snprintf(buf, sizeof(buf),
                      (autonCursor == selectedAuton) ? "> %s *" : "> %s",
                      name);
        controller.set_text(0, 0, buf);

        if (!pros::competition::is_disabled()) {
            pros::delay(50);
            continue;
        }

        bool up   = controller.get_digital(pros::E_CONTROLLER_DIGITAL_UP);
        bool down = controller.get_digital(pros::E_CONTROLLER_DIGITAL_DOWN);
        bool xBtn = controller.get_digital(pros::E_CONTROLLER_DIGITAL_X);

        if (up && !lastUp) {
            autonCursor = (autonCursor % 4) + 1;
            controller.rumble(".");
        }

        if (down && !lastDown) {
            autonCursor--;
            if (autonCursor < 1) autonCursor = 4;
            controller.rumble(".");
        }

        if (xBtn && !lastX) {
            selectedAuton = autonCursor;
            update_title();
            controller.rumble("-.");
        }

        lastUp = up;
        lastDown = down;
        lastX = xBtn;

        pros::delay(50);
    }
}



extern "C" void build_base_screen() {
    lv_obj_clean(lv_screen_active());

    lv_obj_set_style_bg_image_src(lv_screen_active(), &img_14683C, LV_PART_MAIN);
    lv_obj_set_style_bg_image_opa(lv_screen_active(), LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, LV_PART_MAIN);

    label_title = lv_label_create(lv_screen_active());
    lv_label_set_text(label_title, "Auton: None");
    lv_obj_set_style_text_color(label_title, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(label_title, LV_FONT_MONTSERRAT_22, LV_PART_MAIN);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 5);

    lv_obj_t* label_team = lv_label_create(lv_screen_active());
    lv_label_set_text(label_team, "Team 14683C");
    lv_obj_set_style_text_color(label_team, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(label_team, &lv_font_montserrat_18, LV_PART_MAIN);
    lv_obj_align(label_team, LV_ALIGN_TOP_MID, 0, 25);

    label_pose = lv_label_create(lv_screen_active());
    lv_label_set_text(label_pose, "Pose: x=0.0  y=0.0  θ=0.0°");
    lv_obj_set_style_text_color(label_pose, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_set_style_text_font(label_pose, &lv_font_montserrat_20, LV_PART_MAIN);
    lv_obj_align(label_pose, LV_ALIGN_BOTTOM_MID, 0, -5);

    if (!poseTimerStarted) {
        lv_timer_create(pose_timer_cb, 100, nullptr);
        poseTimerStarted = true;
    }

    update_title();
}


extern "C" void build_auton_selector() {
    auto make_auton_button = [](const char* text, int id,
                                lv_align_t align, int x_ofs, int y_ofs) {
        lv_obj_t* btn = lv_button_create(lv_screen_active());
        lv_obj_set_size(btn, 140, 60);
        lv_obj_align(btn, align, x_ofs, y_ofs);

        // Only handle real clicks
        lv_obj_add_event_cb(
            btn,
            auton_btn_event_cb,
            LV_EVENT_CLICKED,
            (void*)(intptr_t)id
        );

        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, text);
        lv_obj_center(lbl);

        // If you tap the label text, bubble the event up to the button
        lv_obj_add_flag(lbl, LV_OBJ_FLAG_EVENT_BUBBLE);
    };

    make_auton_button("Red Left", 1, LV_ALIGN_LEFT_MID,   10, -60);
    make_auton_button("Red Right", 2, LV_ALIGN_RIGHT_MID, -10, -60);
    make_auton_button("Blue Left", 3, LV_ALIGN_LEFT_MID,   10, 0);
    make_auton_button("Blue Right", 4, LV_ALIGN_RIGHT_MID, -10, 0);

    update_title();
}
