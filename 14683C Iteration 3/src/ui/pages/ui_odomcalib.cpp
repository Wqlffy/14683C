#include "ui_odomcalib.hpp"

#include <cstdint>

#include "lemlib/api.hpp"
#include "main.h"
#include "robot_config.hpp"
#include "../ui_theme.hpp"

extern lemlib::Chassis chassis;

namespace ui_odomcalib {
namespace {
constexpr lv_coord_t kHeaderPillHeight = 34;
constexpr lv_coord_t kButtonHeight = 48;
constexpr lv_coord_t kBottomPanelHeight = 90;

lv_obj_t* s_root = nullptr;
lv_obj_t* s_pose_x = nullptr;
lv_obj_t* s_pose_y = nullptr;
lv_obj_t* s_pose_z = nullptr;
lv_obj_t* s_pose_theta = nullptr;
lv_obj_t* s_left_dist = nullptr;
lv_obj_t* s_right_dist = nullptr;
lv_obj_t* s_status = nullptr;

int s_left_offset = 0;
int s_right_offset = 0;
bool s_recalib_busy = false;
bool s_done_pending = false;
std::uint32_t s_done_time = 0;

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

void set_status(const char* text) {
    if (s_status) {
        lv_label_set_text(s_status, text);
    }
}

void on_recalib(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    if (s_recalib_busy || s_done_pending) {
        return;  // Debounce recalibration while running or cooling down.
    }

    s_recalib_busy = true;
    set_status("RUNNING...");

    chassis.setPose(0, 0, 0);
    imu.tare_heading();

    s_left_offset = leftDist.get();
    s_right_offset = rightDist.get();

    set_status("DONE");
    s_done_time = pros::millis();
    s_done_pending = true;
    s_recalib_busy = false;
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
    lv_obj_set_flex_flow(pose, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(pose, 1);
    lv_obj_set_style_pad_row(pose, ui_theme::kPadSm, LV_PART_MAIN);

    make_title(pose, "POSE");
    s_pose_x = ui_theme::make_label(pose, "X: 0.00",
                                    ui_theme::color_text(),
                                    ui_theme::font_body());
    s_pose_y = ui_theme::make_label(pose, "Y: 0.00",
                                    ui_theme::color_text(),
                                    ui_theme::font_body());
    s_pose_z = ui_theme::make_label(pose, "Z: N/A",
                                    ui_theme::color_text_dim(),
                                    ui_theme::font_body());
    s_pose_theta = ui_theme::make_label(pose, "Theta: 0.00",
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

    lv_obj_t* bottom = lv_obj_create(s_root);
    ui_theme::apply_panel(bottom);
    lv_obj_set_height(bottom, kBottomPanelHeight);
    lv_obj_set_width(bottom, LV_PCT(100));
    lv_obj_set_flex_flow(bottom, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(bottom, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_style_pad_all(bottom, ui_theme::kPadSm, LV_PART_MAIN);

    lv_obj_t* btn = lv_button_create(bottom);
    ui_theme::apply_button(btn);
    lv_obj_set_width(btn, LV_PCT(100));
    lv_obj_set_height(btn, kButtonHeight);
    lv_obj_add_event_cb(btn, on_recalib, LV_EVENT_CLICKED, nullptr);
    lv_obj_t* btn_label =
        ui_theme::make_label(btn, "RECALIBRATE", ui_theme::color_text(),
                             ui_theme::font_body());
    lv_obj_center(btn_label);

    s_status = ui_theme::make_label(bottom, "READY", ui_theme::color_text_dim(),
                                    ui_theme::font_body());

    update();
    return s_root;
}

void update() {
    if (!s_root) {
        return;
    }

    if (s_done_pending &&
        (pros::millis() - s_done_time) >= 1000) {
        set_status("READY");
        s_done_pending = false;
    }

    const auto pose = chassis.getPose();
    lv_label_set_text_fmt(s_pose_x, "X: %.2f", pose.x);
    lv_label_set_text_fmt(s_pose_y, "Y: %.2f", pose.y);
    lv_label_set_text_fmt(s_pose_theta, "Theta: %.2f", pose.theta);

    const int left_mm = leftDist.get() - s_left_offset;
    const int right_mm = rightDist.get() - s_right_offset;
    lv_label_set_text_fmt(s_left_dist, "Left: %dmm", left_mm);
    lv_label_set_text_fmt(s_right_dist, "Right: %dmm", right_mm);
}
}
