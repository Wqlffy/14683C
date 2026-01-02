#include "ui_motors.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>

#include "../ui_theme.hpp"

namespace ui_motors {
namespace {
constexpr size_t kModeCount = 6;
constexpr size_t kMaxRows = 12;

const char* kModeLabels[kModeCount] = {
    "TEMP", "CURRENT", "POWER", "VOLTS", "VELOCITY", "TORQUE"};
const char* kModeHeaders[kModeCount] = {
    "TEMP C", "CURR A", "PWR W", "VOLTS V", "VEL RPM", "TORQ NM"};

struct MotorRow {
    lv_obj_t* name;
    lv_obj_t* col1;
    lv_obj_t* col2;
    lv_obj_t* col3;
};

MotorMode s_mode = MotorMode::Temp;
lv_obj_t* s_root = nullptr;
lv_obj_t* s_mode_buttons[kModeCount] = {};
MotorRow s_rows[kMaxRows] = {};
size_t s_row_count = 0;
lv_obj_t* s_header_mode = nullptr;

// Update these ports/names to match your real drivetrain and mechanisms.
MotorEntry s_default_motors[] = {
    {"Left Front", pros::Motor(1)},
    {"Left Mid", pros::Motor(2)},
    {"Left Rear", pros::Motor(3)},
    {"Right Front", pros::Motor(4)},
    {"Right Mid", pros::Motor(5)},
    {"Right Rear", pros::Motor(6)},
};

const MotorEntry* s_entries = s_default_motors;
size_t s_entry_count = sizeof(s_default_motors) / sizeof(s_default_motors[0]);

double get_mode_value(const MotorEntry& entry, MotorMode mode) {
    switch (mode) {
        case MotorMode::Temp:
            return entry.motor.get_temperature();
        case MotorMode::Current:
            return entry.motor.get_current_draw() / 1000.0;
        case MotorMode::Power:
            return entry.motor.get_power();
        case MotorMode::Volts:
            return entry.motor.get_voltage() / 1000.0;
        case MotorMode::Velocity:
            return entry.motor.get_actual_velocity();
        case MotorMode::Torque:
            return entry.motor.get_torque();
    }
    return 0.0;
}

void update_header() {
    if (!s_header_mode) {
        return;
    }
    const size_t idx = static_cast<size_t>(s_mode);
    lv_label_set_text(s_header_mode, kModeHeaders[idx]);
}

void update_mode_buttons() {
    for (size_t i = 0; i < kModeCount; ++i) {
        if (s_mode_buttons[i]) {
            ui_theme::set_button_selected(s_mode_buttons[i],
                                          i == static_cast<size_t>(s_mode));
        }
    }
}

void on_mode_event(lv_event_t* e) {
    const intptr_t raw = reinterpret_cast<intptr_t>(lv_event_get_user_data(e));
    const auto mode = static_cast<MotorMode>(raw);
    set_mode(mode);
}

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
}  // namespace

lv_obj_t* build(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_remove_flag(s_root, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(s_root, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(s_root, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(s_root, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(s_root, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);

    make_header_pill(s_root, "MOTORS");

    lv_obj_t* content = lv_obj_create(s_root);
    lv_obj_remove_flag(content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(content, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(content, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(content, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(content, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_grow(content, 1);
    lv_obj_set_width(content, LV_PCT(100));

    lv_obj_t* mode_panel = lv_obj_create(content);
    ui_theme::apply_panel(mode_panel);
    lv_obj_set_width(mode_panel, 170);
    lv_obj_set_flex_flow(mode_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(mode_panel, ui_theme::kPadSm, LV_PART_MAIN);

    for (size_t i = 0; i < kModeCount; ++i) {
        lv_obj_t* btn = lv_button_create(mode_panel);
        ui_theme::apply_button(btn);
        lv_obj_set_width(btn, LV_PCT(100));
        lv_obj_set_height(btn, 40);
        lv_obj_add_event_cb(btn, on_mode_event, LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(static_cast<intptr_t>(i)));
        lv_obj_t* label =
            ui_theme::make_label(btn, kModeLabels[i], ui_theme::color_text(),
                                 ui_theme::font_body());
        lv_obj_center(label);
        s_mode_buttons[i] = btn;
    }

    lv_obj_t* table_panel = lv_obj_create(content);
    ui_theme::apply_panel(table_panel);
    lv_obj_set_flex_flow(table_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(table_panel, 1);
    lv_obj_set_width(table_panel, LV_PCT(100));
    lv_obj_set_style_pad_row(table_panel, ui_theme::kPadSm, LV_PART_MAIN);

    lv_obj_t* header = lv_obj_create(table_panel);
    lv_obj_remove_flag(header, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(header, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(header, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(header, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_column(header, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(header, LV_PCT(100));

    lv_obj_t* header_name =
        ui_theme::make_label(header, "MOTOR", ui_theme::color_text_dim(),
                             ui_theme::font_small());
    lv_obj_set_width(header_name, 140);

    s_header_mode =
        ui_theme::make_label(header, "TEMP C", ui_theme::color_text_dim(),
                             ui_theme::font_small());
    lv_obj_set_width(s_header_mode, 70);
    lv_obj_set_style_text_align(s_header_mode, LV_TEXT_ALIGN_RIGHT,
                                LV_PART_MAIN);

    lv_obj_t* header_target =
        ui_theme::make_label(header, "TGT", ui_theme::color_text_dim(),
                             ui_theme::font_small());
    lv_obj_set_width(header_target, 70);
    lv_obj_set_style_text_align(header_target, LV_TEXT_ALIGN_RIGHT,
                                LV_PART_MAIN);

    lv_obj_t* header_err =
        ui_theme::make_label(header, "ERR", ui_theme::color_text_dim(),
                             ui_theme::font_small());
    lv_obj_set_width(header_err, 70);
    lv_obj_set_style_text_align(header_err, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    lv_obj_t* rows = lv_obj_create(table_panel);
    lv_obj_set_flex_flow(rows, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_bg_opa(rows, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(rows, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(rows, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(rows, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_width(rows, LV_PCT(100));
    lv_obj_set_flex_grow(rows, 1);

    s_row_count = std::min(s_entry_count, kMaxRows);
    for (size_t i = 0; i < s_row_count; ++i) {
        lv_obj_t* row = lv_obj_create(rows);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, ui_theme::color_panel_alt(),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_opa(row, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, ui_theme::kBorder, LV_PART_MAIN);
        lv_obj_set_style_border_color(row, ui_theme::color_border(),
                                      LV_PART_MAIN);
        lv_obj_set_style_radius(row, 8, LV_PART_MAIN);
        lv_obj_set_style_pad_all(row, ui_theme::kPadSm, LV_PART_MAIN);
        lv_obj_set_style_pad_column(row, ui_theme::kPadSm, LV_PART_MAIN);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_height(row, 32);
        lv_obj_set_width(row, LV_PCT(100));

        s_rows[i].name =
            ui_theme::make_label(row, s_entries[i].name,
                                 ui_theme::color_text(), ui_theme::font_body());
        lv_label_set_long_mode(s_rows[i].name, LV_LABEL_LONG_DOT);
        lv_obj_set_width(s_rows[i].name, 140);

        s_rows[i].col1 =
            ui_theme::make_label(row, "--", ui_theme::color_text(),
                                 ui_theme::font_body());
        lv_obj_set_width(s_rows[i].col1, 70);
        lv_obj_set_style_text_align(s_rows[i].col1, LV_TEXT_ALIGN_RIGHT,
                                    LV_PART_MAIN);

        s_rows[i].col2 =
            ui_theme::make_label(row, "--", ui_theme::color_text_dim(),
                                 ui_theme::font_body());
        lv_obj_set_width(s_rows[i].col2, 70);
        lv_obj_set_style_text_align(s_rows[i].col2, LV_TEXT_ALIGN_RIGHT,
                                    LV_PART_MAIN);

        s_rows[i].col3 =
            ui_theme::make_label(row, "--", ui_theme::color_text_dim(),
                                 ui_theme::font_body());
        lv_obj_set_width(s_rows[i].col3, 70);
        lv_obj_set_style_text_align(s_rows[i].col3, LV_TEXT_ALIGN_RIGHT,
                                    LV_PART_MAIN);
    }

    set_mode(s_mode);
    update();
    return s_root;
}

void update() {
    if (!s_root) {
        return;
    }
    for (size_t i = 0; i < s_row_count; ++i) {
        const MotorEntry& entry = s_entries[i];
        const double value = get_mode_value(entry, s_mode);
        const double target = entry.motor.get_target_velocity();
        const double actual = entry.motor.get_actual_velocity();
        const double error = target - actual;

        lv_label_set_text_fmt(s_rows[i].col1, "%.1f", value);
        lv_label_set_text_fmt(s_rows[i].col2, "%.0f", target);
        lv_label_set_text_fmt(s_rows[i].col3, "%.0f", error);
    }
}

void set_mode(MotorMode mode) {
    s_mode = mode;
    update_header();
    update_mode_buttons();
    update();
}

void set_motor_entries(const MotorEntry* entries, size_t count) {
    s_entries = entries ? entries : s_default_motors;
    s_entry_count = entries ? count
                            : sizeof(s_default_motors) / sizeof(s_default_motors[0]);
}
}
