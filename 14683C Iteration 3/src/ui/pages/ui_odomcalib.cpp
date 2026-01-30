#include "ui_odomcalib.hpp"

#include <cstdint>
#include <cstdio>
#include <cstring>

#include "lemlib/api.hpp"
#include "main.h"
#include "robot_config.hpp"
#include "../ui_theme.hpp"

extern lemlib::Chassis chassis;

namespace ui_odomcalib {
namespace {
constexpr lv_coord_t kHeaderPillHeight = 34;
constexpr lv_coord_t kPosePanelWidth = 140;
constexpr lv_coord_t kSidePanelWidth = 86;
constexpr lv_coord_t kSideButtonHeight = 36;
constexpr std::uint32_t kPoseUpdateMs = 100;

lv_obj_t* s_root = nullptr;
lv_obj_t* s_pose_x = nullptr;
lv_obj_t* s_pose_y = nullptr;
lv_obj_t* s_pose_theta = nullptr;
lv_obj_t* s_left_dist = nullptr;
lv_obj_t* s_right_dist = nullptr;

std::uint32_t s_last_pose_update = 0;
char s_last_x[24] = "";
char s_last_y[24] = "";
char s_last_theta[24] = "";
char s_last_left[24] = "";
char s_last_right[24] = "";
bool s_zero_pose_req = false;
bool s_zero_imu_req = false;

void set_transparent(lv_obj_t* obj) {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(obj, LV_DIR_NONE);
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
}

lv_obj_t* make_header_pill(lv_obj_t* parent, const char* text) {
    lv_obj_t* pill = lv_obj_create(parent);
    ui_theme::apply_panel(pill);
    lv_obj_set_style_radius(pill, ui_theme::kRadiusPill, LV_PART_MAIN);
    lv_obj_set_height(pill, kHeaderPillHeight);
    lv_obj_set_width(pill, LV_PCT(100));
    lv_obj_set_flex_flow(pill, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(pill, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_align(pill, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    lv_obj_t* icon = lv_obj_create(pill);
    lv_obj_set_size(icon, 18, 18);
    lv_obj_set_style_bg_color(icon, ui_theme::color_accent(), LV_PART_MAIN);
    lv_obj_set_style_border_width(icon, 0, LV_PART_MAIN);
    lv_obj_set_style_radius(icon, 4, LV_PART_MAIN);

    ui_theme::make_label(pill, text, ui_theme::color_text(),
                         ui_theme::font_body());
    return pill;
}

lv_obj_t* make_title(lv_obj_t* parent, const char* text) {
    return ui_theme::make_label(parent, text, ui_theme::color_text_dim(),
                                ui_theme::font_small());
}

void set_label_cached(lv_obj_t* label, const char* text, char* cache,
                      size_t cache_len) {
    if (!label || !text) {
        return;
    }
    if (std::strncmp(cache, text, cache_len) == 0) {
        return;
    }
    std::snprintf(cache, cache_len, "%s", text);
    lv_label_set_text(label, cache);
}

void on_zero_pose(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    s_zero_pose_req = true;
}

void on_zero_imu(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    s_zero_imu_req = true;
}
}

lv_obj_t* build(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_root, ui_theme::kPad, LV_PART_MAIN);
    set_transparent(s_root);

    make_header_pill(s_root, "ODOM");

    lv_obj_t* content = lv_obj_create(s_root);
    set_transparent(content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(content, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_grow(content, 1);
    lv_obj_set_width(content, LV_PCT(100));

    lv_obj_t* pose = lv_obj_create(content);
    ui_theme::apply_panel(pose);
    lv_obj_set_size(pose, kPosePanelWidth, LV_SIZE_CONTENT);
    lv_obj_set_flex_flow(pose, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(pose, ui_theme::kPadSm, LV_PART_MAIN);

    make_title(pose, "POSE");
    s_pose_x = ui_theme::make_label(pose, "X: 0.0",
                                    ui_theme::color_text(),
                                    ui_theme::font_body());
    s_pose_y = ui_theme::make_label(pose, "Y: 0.0",
                                    ui_theme::color_text(),
                                    ui_theme::font_body());
    s_pose_theta = ui_theme::make_label(pose, "θ: 0.0°",
                                        ui_theme::color_text(),
                                        ui_theme::font_body());

    lv_obj_t* dist = lv_obj_create(content);
    ui_theme::apply_panel(dist);
    lv_obj_set_flex_flow(dist, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(dist, 1);
    lv_obj_set_style_pad_row(dist, ui_theme::kPadSm, LV_PART_MAIN);

    make_title(dist, "DISTANCE");
    s_left_dist = ui_theme::make_label(dist, "Left: 0mm",
                                       ui_theme::color_text(),
                                       ui_theme::font_body());
    s_right_dist = ui_theme::make_label(dist, "Right: 0mm",
                                        ui_theme::color_text(),
                                        ui_theme::font_body());

    lv_obj_t* side = lv_obj_create(content);
    ui_theme::apply_panel(side);
    lv_obj_set_width(side, kSidePanelWidth);
    lv_obj_set_flex_flow(side, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(side, ui_theme::kPadSm, LV_PART_MAIN);

    lv_obj_t* zero_btn = lv_button_create(side);
    ui_theme::apply_button(zero_btn);
    lv_obj_set_width(zero_btn, LV_PCT(100));
    lv_obj_set_height(zero_btn, kSideButtonHeight);
    lv_obj_add_event_cb(zero_btn, on_zero_pose, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* zero_label =
        ui_theme::make_label(zero_btn, "ZERO", ui_theme::color_text(),
                             ui_theme::font_body());
    lv_obj_center(zero_label);

    lv_obj_t* imu_btn = lv_button_create(side);
    ui_theme::apply_button(imu_btn);
    lv_obj_set_width(imu_btn, LV_PCT(100));
    lv_obj_set_height(imu_btn, kSideButtonHeight);
    lv_obj_add_event_cb(imu_btn, on_zero_imu, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* imu_label =
        ui_theme::make_label(imu_btn, "IMU 0", ui_theme::color_text(),
                             ui_theme::font_body());
    lv_obj_center(imu_label);

    update();
    return s_root;
}

void update() {
    if (!s_root) {
        return;
    }
    const std::uint32_t now = pros::millis();
    if (now - s_last_pose_update < kPoseUpdateMs) {
        return;
    }
    s_last_pose_update = now;

    if (s_zero_pose_req) {
        s_zero_pose_req = false;
        leftMotors.move(0);
        rightMotors.move(0);
        chassis.setPose(0, 0, 0);
        imu.tare_heading();
    }
    if (s_zero_imu_req) {
        s_zero_imu_req = false;
        imu.tare_heading();
    }

    const auto pose = chassis.getPose();
    char buf[24];
    std::snprintf(buf, sizeof(buf), "X: %.1f", pose.x);
    set_label_cached(s_pose_x, buf, s_last_x, sizeof(s_last_x));
    std::snprintf(buf, sizeof(buf), "Y: %.1f", pose.y);
    set_label_cached(s_pose_y, buf, s_last_y, sizeof(s_last_y));
    std::snprintf(buf, sizeof(buf), "θ: %.1f°", pose.theta);
    set_label_cached(s_pose_theta, buf, s_last_theta, sizeof(s_last_theta));

    const int left_mm = leftDist.get();
    const int right_mm = rightDist.get();
    std::snprintf(buf, sizeof(buf), "Left: %dmm", left_mm);
    set_label_cached(s_left_dist, buf, s_last_left, sizeof(s_last_left));
    std::snprintf(buf, sizeof(buf), "Right: %dmm", right_mm);
    set_label_cached(s_right_dist, buf, s_last_right, sizeof(s_last_right));
}
}
