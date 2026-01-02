#include "ui_sensors.hpp"

#include "main.h"
#include "../ui_theme.hpp"

namespace ui_sensors {
namespace {
lv_obj_t* s_root = nullptr;

lv_obj_t* s_enc_l_angle = nullptr;
lv_obj_t* s_enc_l_vel = nullptr;
lv_obj_t* s_enc_r_angle = nullptr;
lv_obj_t* s_enc_r_vel = nullptr;
lv_obj_t* s_limit_status = nullptr;

lv_obj_t* s_opt_hue = nullptr;
lv_obj_t* s_opt_sat = nullptr;
lv_obj_t* s_opt_bri = nullptr;

lv_obj_t* s_inertial_heading = nullptr;
lv_obj_t* s_inertial_rotation = nullptr;
lv_obj_t* s_inertial_arc = nullptr;

lv_obj_t* s_batt_volt = nullptr;
lv_obj_t* s_batt_temp = nullptr;
lv_obj_t* s_batt_curr = nullptr;
lv_obj_t* s_batt_cap = nullptr;

lv_obj_t* make_header_pill(lv_obj_t* parent, const char* text) {
    lv_obj_t* pill = lv_obj_create(parent);
    ui_theme::apply_panel(pill);
    lv_obj_set_style_radius(pill, ui_theme::kRadiusPill, LV_PART_MAIN);
    lv_obj_set_height(pill, 34);
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
}

lv_obj_t* build(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_root, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_root, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(s_root, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);

    make_header_pill(s_root, "SENSORS");

    lv_obj_t* row1 = lv_obj_create(s_root);
    lv_obj_set_style_bg_opa(row1, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row1, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row1, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(row1, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row1, LV_PCT(100));
    lv_obj_set_flex_grow(row1, 1);

    lv_obj_t* enc_l = lv_obj_create(row1);
    ui_theme::apply_panel(enc_l);
    lv_obj_set_flex_flow(enc_l, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(enc_l, 1);
    make_title(enc_l, "ENCODER L");
    s_enc_l_angle = ui_theme::make_label(enc_l, "Angle: 0.0",
                                         ui_theme::color_text(),
                                         ui_theme::font_body());
    s_enc_l_vel = ui_theme::make_label(enc_l, "Vel: 0.0",
                                       ui_theme::color_text_dim(),
                                       ui_theme::font_body());

    lv_obj_t* enc_r = lv_obj_create(row1);
    ui_theme::apply_panel(enc_r);
    lv_obj_set_flex_flow(enc_r, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(enc_r, 1);
    make_title(enc_r, "ENCODER R");
    s_enc_r_angle = ui_theme::make_label(enc_r, "Angle: 0.0",
                                         ui_theme::color_text(),
                                         ui_theme::font_body());
    s_enc_r_vel = ui_theme::make_label(enc_r, "Vel: 0.0",
                                       ui_theme::color_text_dim(),
                                       ui_theme::font_body());

    lv_obj_t* limit = lv_obj_create(row1);
    ui_theme::apply_panel(limit);
    lv_obj_set_width(limit, 140);
    lv_obj_set_flex_flow(limit, LV_FLEX_FLOW_COLUMN);
    make_title(limit, "LIMIT");
    s_limit_status =
        ui_theme::make_label(limit, "OPEN", ui_theme::color_text(),
                             ui_theme::font_title());

    lv_obj_t* row2 = lv_obj_create(s_root);
    lv_obj_set_style_bg_opa(row2, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row2, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(row2, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(row2, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row2, LV_PCT(100));
    lv_obj_set_flex_grow(row2, 2);

    lv_obj_t* optical = lv_obj_create(row2);
    ui_theme::apply_panel(optical);
    lv_obj_set_width(optical, 190);
    lv_obj_set_flex_flow(optical, LV_FLEX_FLOW_COLUMN);
    make_title(optical, "OPTICAL");
    s_opt_hue = ui_theme::make_label(optical, "Hue: 0",
                                     ui_theme::color_text(),
                                     ui_theme::font_body());
    s_opt_sat = ui_theme::make_label(optical, "Sat: 0",
                                     ui_theme::color_text_dim(),
                                     ui_theme::font_body());
    s_opt_bri = ui_theme::make_label(optical, "Bri: 0",
                                     ui_theme::color_text_dim(),
                                     ui_theme::font_body());

    lv_obj_t* inertial = lv_obj_create(row2);
    ui_theme::apply_panel(inertial);
    lv_obj_set_flex_grow(inertial, 1);
    lv_obj_set_flex_flow(inertial, LV_FLEX_FLOW_COLUMN);
    make_title(inertial, "INERTIAL");

    lv_obj_t* inertial_top = lv_obj_create(inertial);
    lv_obj_set_style_bg_opa(inertial_top, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(inertial_top, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(inertial_top, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(inertial_top, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_flow(inertial_top, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(inertial_top, LV_PCT(100));

    s_inertial_heading =
        ui_theme::make_label(inertial_top, "Heading: 0.0",
                             ui_theme::color_text(), ui_theme::font_body());
    s_inertial_rotation =
        ui_theme::make_label(inertial_top, "Rot: 0.0",
                             ui_theme::color_text_dim(),
                             ui_theme::font_body());

    s_inertial_arc = lv_arc_create(inertial);
    lv_obj_set_size(s_inertial_arc, 90, 90);
    lv_arc_set_range(s_inertial_arc, 0, 360);
    lv_arc_set_value(s_inertial_arc, 0);
    lv_obj_set_style_arc_width(s_inertial_arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_inertial_arc, 6, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_inertial_arc, ui_theme::color_panel_alt(),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_inertial_arc, ui_theme::color_accent(),
                               LV_PART_INDICATOR);
    lv_obj_remove_style(s_inertial_arc, nullptr, LV_PART_KNOB);

    lv_obj_t* row3 = lv_obj_create(s_root);
    lv_obj_set_style_bg_opa(row3, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(row3, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(row3, 0, LV_PART_MAIN);
    lv_obj_set_width(row3, LV_PCT(100));

    lv_obj_t* battery = lv_obj_create(row3);
    ui_theme::apply_panel(battery);
    lv_obj_set_flex_flow(battery, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(battery, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_width(battery, LV_PCT(100));
    lv_obj_set_height(battery, 50);

    s_batt_volt =
        ui_theme::make_label(battery, "Batt: 0.0V", ui_theme::color_text(),
                             ui_theme::font_body());
    s_batt_temp =
        ui_theme::make_label(battery, "Temp: 0.0C", ui_theme::color_text_dim(),
                             ui_theme::font_body());
    s_batt_curr =
        ui_theme::make_label(battery, "Curr: 0.0A", ui_theme::color_text_dim(),
                             ui_theme::font_body());
    s_batt_cap =
        ui_theme::make_label(battery, "Cap: 0", ui_theme::color_text_dim(),
                             ui_theme::font_body());

    update();
    return s_root;
}

void update() {
    if (!s_root) {
        return;
    }

    // TODO: Replace placeholders with your real sensor reads.
    const double enc_l_angle = 0.0;
    const double enc_l_vel = 0.0;
    const double enc_r_angle = 0.0;
    const double enc_r_vel = 0.0;

    lv_label_set_text_fmt(s_enc_l_angle, "Angle: %.1f", enc_l_angle);
    lv_label_set_text_fmt(s_enc_l_vel, "Vel: %.1f", enc_l_vel);
    lv_label_set_text_fmt(s_enc_r_angle, "Angle: %.1f", enc_r_angle);
    lv_label_set_text_fmt(s_enc_r_vel, "Vel: %.1f", enc_r_vel);

    const bool limit_closed = false;
    lv_label_set_text(s_limit_status, limit_closed ? "CLOSED" : "OPEN");
    lv_obj_set_style_text_color(
        s_limit_status,
        limit_closed ? ui_theme::color_accent() : ui_theme::color_text(),
        LV_PART_MAIN);

    const int opt_hue = 0;
    const int opt_sat = 0;
    const int opt_bri = 0;
    lv_label_set_text_fmt(s_opt_hue, "Hue: %d", opt_hue);
    lv_label_set_text_fmt(s_opt_sat, "Sat: %d", opt_sat);
    lv_label_set_text_fmt(s_opt_bri, "Bri: %d", opt_bri);

    const double heading = 0.0;
    const double rotation = 0.0;
    lv_label_set_text_fmt(s_inertial_heading, "Heading: %.1f", heading);
    lv_label_set_text_fmt(s_inertial_rotation, "Rot: %.1f", rotation);
    lv_arc_set_value(s_inertial_arc, static_cast<int>(heading));

    const double batt_volt = pros::battery::get_voltage() / 1000.0;
    const double batt_temp = pros::battery::get_temperature();
    const double batt_curr = pros::battery::get_current() / 1000.0;
    const double batt_cap = pros::battery::get_capacity();

    lv_label_set_text_fmt(s_batt_volt, "Batt: %.1fV", batt_volt);
    lv_label_set_text_fmt(s_batt_temp, "Temp: %.1fC", batt_temp);
    lv_label_set_text_fmt(s_batt_curr, "Curr: %.1fA", batt_curr);
    lv_label_set_text_fmt(s_batt_cap, "Cap: %.0f", batt_cap);
}
}
