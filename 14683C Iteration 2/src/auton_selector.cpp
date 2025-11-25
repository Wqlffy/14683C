#include "liblvgl/display/lv_display.h"
#include "liblvgl/widgets/button/lv_button.h"
#include "liblvgl/widgets/image/lv_image.h"
#include "main.h"
#include "lemlib/api.hpp" // IWYU pragma: keep
#include <cstdio>
#include <cmath>
#include <cstdint>

extern lemlib::Chassis chassis;

extern const lv_image_dsc_t img_14683C;

// Global selected auton (default to 1, e.g. "Red Left")
int selectedAuton = 1;

static lv_obj_t* label_title = nullptr;
static lv_obj_t* label_pose  = nullptr;
static bool poseTimerStarted = false;


static void update_title() {
    if (!label_title) return;

    const char* name = "None";
    switch (selectedAuton) {
        case 1: name = "Red Left";           break;
        case 2: name = "Red Right";          break;
        case 3: name = "Blue Left";          break;
        case 4: name = "Blue Right";         break;
        case 5: name = "Red Left AWP";       break;
        case 6: name = "Red Right AWP";      break;
        case 7: name = "Blue Left AWP";      break;
        case 8: name = "Blue Right AWP";     break;
        case 9: name = "Skills";             break;
        default: break;
    }

    static char buf[32];
    std::snprintf(buf, sizeof(buf), "Auton: %s", name);
    lv_label_set_text(label_title, buf);
}

static void pose_timer_cb(lv_timer_t* timer) {
    (void)timer; // unused

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
    lv_event_code_t code = lv_event_get_code(e);
    if (code != LV_EVENT_CLICKED) return;

    int id = (int)(intptr_t)lv_event_get_user_data(e);
    selectedAuton = id;
    update_title();
}

extern "C" void build_base_screen() {
    // Clear the screen
    lv_obj_clean(lv_screen_active());

    lv_obj_t* bg = lv_image_create(lv_screen_active());
    lv_image_set_src(bg, &img_14683C);

    lv_obj_set_style_bg_image_src(lv_screen_active(), &img_14683C, LV_PART_MAIN);
    lv_obj_set_style_bg_image_opa(lv_screen_active(), LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, LV_PART_MAIN);


    label_title = lv_label_create(lv_screen_active());
    lv_label_set_text(label_title, "Auton: None");
    lv_obj_set_style_text_color(label_title, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label_title, LV_ALIGN_TOP_MID, 0, 5);

    lv_obj_t* label_team = lv_label_create(lv_screen_active());
    lv_label_set_text(label_team, "Team 14683C");
    lv_obj_set_style_text_color(label_team, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label_team, LV_ALIGN_TOP_MID, 0, 25);

    label_pose = lv_label_create(lv_screen_active());
    lv_label_set_text(label_pose, "Pose: x=0.0  y=0.0  θ=0.0°");
    lv_obj_set_style_text_color(label_pose, lv_color_hex(0xffffff), LV_PART_MAIN);
    lv_obj_align(label_pose, LV_ALIGN_BOTTOM_MID, 0, -5);

    if (!poseTimerStarted) {
        lv_timer_create(pose_timer_cb, 100, nullptr);
        poseTimerStarted = true;
    }

    selectedAuton = 0;
    update_title(); // will show "Auton: None"
}


extern "C" void build_auton_selector() {
    auto make_auton_button = [](const char* text, int id,
                                lv_align_t align, int x_ofs, int y_ofs) {
        lv_obj_t* btn = lv_button_create(lv_screen_active());
        lv_obj_set_size(btn, 140, 60);
        lv_obj_align(btn, align, x_ofs, y_ofs);

        lv_obj_add_event_cb(
            btn,
            auton_btn_event_cb,
            LV_EVENT_ALL,
            (void*)(intptr_t)id
        );

        lv_obj_t* lbl = lv_label_create(btn);
        lv_label_set_text(lbl, text);
        lv_obj_center(lbl);
    };

    make_auton_button("Red Left",        1, LV_ALIGN_LEFT_MID,   10, -60);
    make_auton_button("Red Right",       2, LV_ALIGN_RIGHT_MID, -10, -60);

    make_auton_button("Blue Left",       3, LV_ALIGN_LEFT_MID,   10, 0);
    make_auton_button("Blue Right",      4, LV_ALIGN_RIGHT_MID, -10, 0);

    make_auton_button("Red Left AWP",    5, LV_ALIGN_LEFT_MID,   10, 60);
    make_auton_button("Red Right AWP",   6, LV_ALIGN_RIGHT_MID, -10, 60);

    make_auton_button("Blue Left AWP",   7, LV_ALIGN_LEFT_MID,   10, 120);
    make_auton_button("Blue Right AWP",  8, LV_ALIGN_RIGHT_MID, -10, 120);

    make_auton_button("Skills",          9, LV_ALIGN_BOTTOM_MID, 0, -40);

    selectedAuton = 1;
    update_title();
}