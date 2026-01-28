#include "ui_motors.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "../ui_theme.hpp"
#include "liblvgl/core/lv_obj_tree.h"
#include "main.h"
#include "robot_config.hpp"

namespace ui_motors {
namespace {
constexpr size_t kMetricCount = 3;
constexpr size_t kMaxRows = 12;

constexpr lv_coord_t kButtonHeight = 40;
constexpr lv_coord_t kHeaderPillHeight = 34;
constexpr lv_coord_t kRowHeight = 32;
constexpr lv_coord_t kTabBarHeight = 44;

constexpr lv_coord_t kNameWidth = 140;
constexpr lv_coord_t kValueWidth = 90;
constexpr lv_coord_t kUnitWidth = 60;

constexpr uint32_t kWarnColor = 0xe0b84b;
constexpr uint32_t kDangerColor = 0xe05858;
constexpr std::uint32_t kUpdateMs = 100;

const char* kMetricLabels[kMetricCount] = {"TEMP", "CURR", "PWR"};
const char* kMetricUnits[kMetricCount] = {"C", "A", "W"};

struct MotorRow {
    lv_obj_t* name;
    lv_obj_t* value;
    lv_obj_t* unit;
};

struct MetricValue {
    double value;
    const char* unit;
    const char* format;
    lv_color_t color;
};

MotorMetric s_metric = MotorMetric::Temp;
lv_obj_t* s_root = nullptr;
lv_obj_t* s_metric_buttons[kMetricCount] = {};
lv_obj_t* s_header_value = nullptr;
lv_obj_t* s_rows_container = nullptr;
MotorRow s_rows[kMaxRows] = {};
size_t s_row_count = 0;
std::uint32_t s_last_update = 0;
char s_last_header[8] = "";
char s_last_values[kMaxRows][16] = {};
char s_last_units[kMaxRows][8] = {};

// Telemetry reads from hardware defined in robot_config.cpp.
MotorEntry kMotorList[] = {
    {"LF", &leftFront},
    {"LM", &leftMid},
    {"LB", &leftBack},
    {"RF", &rightFront},
    {"RM", &rightMid},
    {"RB", &rightBack},
    {"INTAKE", &intakeMotor},
    {"OUTTAKE", &outtakeMotor},
};
constexpr size_t kMotorCount = sizeof(kMotorList) / sizeof(kMotorList[0]);

const MotorEntry* s_motors = kMotorList;
size_t s_motor_count = kMotorCount;

void set_transparent(lv_obj_t* obj) {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(obj, LV_DIR_NONE);  // Prevent parent containers from intercepting scroll.
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
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

MetricValue read_metric(const MotorEntry& entry, MotorMetric metric) {
    MetricValue out{};
    out.color = ui_theme::color_text();
    out.unit = "--";
    out.format = "%.1f";

    switch (metric) {
        case MotorMetric::Temp: {
            out.value = entry.motor->get_temperature();
            out.unit = kMetricUnits[static_cast<size_t>(metric)];
            out.format = "%.1f";
            if (out.value >= 60.0) {
                out.color = lv_color_hex(kDangerColor);
            } else if (out.value >= 50.0) {
                out.color = lv_color_hex(kWarnColor);
            }
            break;
        }
        case MotorMetric::Current: {
            const double current_a = entry.motor->get_current_draw() / 1000.0;
            out.value = current_a;
            out.unit = kMetricUnits[static_cast<size_t>(metric)];
            out.format = "%.2f";
            break;
        }
        case MotorMetric::Power: {
            out.value = entry.motor->get_power();
            out.unit = kMetricUnits[static_cast<size_t>(metric)];
            out.format = "%.2f";
            break;
        }
    }
    return out;
}

void update_buttons() {
    for (size_t i = 0; i < kMetricCount; ++i) {
        if (s_metric_buttons[i]) {
            ui_theme::set_button_selected(
                s_metric_buttons[i],
                i == static_cast<size_t>(s_metric));
        }
    }
}

void update_header() {
    if (!s_header_value) {
        return;
    }
    set_label_cached(s_header_value,
                     kMetricLabels[static_cast<size_t>(s_metric)],
                     s_last_header, sizeof(s_last_header));
}

void set_metric(MotorMetric metric) {
    s_metric = metric;
    update_buttons();
    update_header();
    update();
}

void on_metric_event(lv_event_t* e) {
    if (lv_event_get_code(e) != LV_EVENT_CLICKED) {
        return;
    }
    const intptr_t raw = reinterpret_cast<intptr_t>(lv_event_get_user_data(e));
    const auto metric = static_cast<MotorMetric>(raw);
    set_metric(metric);
}
}

lv_obj_t* build(lv_obj_t* parent) {
    // Root container
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    set_transparent(s_root);
    lv_obj_set_style_pad_row(s_root, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);

    // Header pill
    make_header_pill(s_root, "MOTORS");

    lv_obj_t* content = lv_obj_create(s_root);
    set_transparent(content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(content, 1);
    lv_obj_set_width(content, LV_PCT(100));
    lv_obj_set_height(content, LV_PCT(100));

    lv_obj_t* tab_bar = lv_obj_create(content);
    ui_theme::apply_panel(tab_bar);
    lv_obj_set_height(tab_bar, kTabBarHeight);
    lv_obj_set_width(tab_bar, LV_PCT(100));
    lv_obj_set_flex_flow(tab_bar, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(tab_bar, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_align(tab_bar, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_remove_flag(tab_bar, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(tab_bar, LV_OBJ_FLAG_CLICKABLE);

    for (size_t i = 0; i < kMetricCount; ++i) {
        lv_obj_t* btn = lv_button_create(tab_bar);
        ui_theme::apply_button(btn);
        lv_obj_set_flex_grow(btn, 1);
        lv_obj_set_height(btn, kButtonHeight);
        lv_obj_add_flag(btn, LV_OBJ_FLAG_CLICKABLE);
        lv_obj_add_event_cb(btn, on_metric_event, LV_EVENT_CLICKED,
                            reinterpret_cast<void*>(static_cast<intptr_t>(i)));

        lv_obj_t* label =
            ui_theme::make_label(btn, kMetricLabels[i],
                                 ui_theme::color_text(), ui_theme::font_body());
        lv_obj_center(label);
        s_metric_buttons[i] = btn;
    }

    lv_obj_move_to_index(tab_bar,-1);

    lv_obj_t* table_panel = lv_obj_create(content);
    ui_theme::apply_panel(table_panel);
    lv_obj_remove_flag(table_panel, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_remove_flag(table_panel, LV_OBJ_FLAG_CLICKABLE);
    lv_obj_set_scroll_dir(table_panel, LV_DIR_NONE);
    lv_obj_set_flex_flow(table_panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(table_panel, 1);
    lv_obj_set_width(table_panel, LV_PCT(100));
    lv_obj_set_height(table_panel, 0);
    lv_obj_set_style_pad_row(table_panel, ui_theme::kPadSm, LV_PART_MAIN);

    lv_obj_t* header = lv_obj_create(table_panel);
    set_transparent(header);
    lv_obj_set_style_pad_column(header, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_flow(header, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(header, LV_PCT(100));

    lv_obj_t* header_name =
        ui_theme::make_label(header, "MOTOR", ui_theme::color_text_dim(),
                             ui_theme::font_small());
    lv_obj_set_width(header_name, kNameWidth);

    s_header_value =
        ui_theme::make_label(header, "TEMP", ui_theme::color_text_dim(),
                             ui_theme::font_small());
    lv_obj_set_width(s_header_value, kValueWidth);
    lv_obj_set_style_text_align(s_header_value, LV_TEXT_ALIGN_RIGHT,
                                LV_PART_MAIN);

    lv_obj_t* header_unit =
        ui_theme::make_label(header, "UNIT", ui_theme::color_text_dim(),
                             ui_theme::font_small());
    lv_obj_set_width(header_unit, kUnitWidth);
    lv_obj_set_style_text_align(header_unit, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);

    s_rows_container = lv_obj_create(table_panel);
    set_transparent(s_rows_container);
    lv_obj_add_flag(s_rows_container, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_rows_container, LV_OBJ_FLAG_SCROLL_MOMENTUM);
    lv_obj_add_flag(s_rows_container, LV_OBJ_FLAG_SCROLL_ELASTIC);
    lv_obj_add_flag(s_rows_container, LV_OBJ_FLAG_SCROLL_CHAIN);
    lv_obj_set_scroll_dir(s_rows_container, LV_DIR_VER);
    lv_obj_set_scrollbar_mode(s_rows_container, LV_SCROLLBAR_MODE_AUTO);
    lv_obj_set_style_pad_row(s_rows_container, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_flow(s_rows_container, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_width(s_rows_container, LV_PCT(100));
    lv_obj_set_height(s_rows_container, 0);
    lv_obj_set_flex_grow(s_rows_container, 1);

    s_row_count = std::min(s_motor_count, kMaxRows);
    static int created_rows = 0;
    for (size_t i = 0; i < s_row_count; ++i) {
        lv_obj_t* row = lv_obj_create(s_rows_container);
        lv_obj_remove_flag(row, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_bg_color(row, ui_theme::color_panel_alt(),
                                  LV_PART_MAIN);
        lv_obj_set_style_bg_opa(row, LV_OPA_COVER, LV_PART_MAIN);
        lv_obj_set_style_border_width(row, ui_theme::kBorder, LV_PART_MAIN);
        lv_obj_set_style_border_color(row, ui_theme::color_border(),
                                      LV_PART_MAIN);
        lv_obj_set_style_radius(row, ui_theme::kRadius, LV_PART_MAIN);
        lv_obj_set_style_pad_all(row, ui_theme::kPadSm, LV_PART_MAIN);
        lv_obj_set_style_pad_column(row, ui_theme::kPadSm, LV_PART_MAIN);
        lv_obj_set_flex_flow(row, LV_FLEX_FLOW_ROW);
        lv_obj_set_height(row, kRowHeight);
        lv_obj_set_width(row, LV_PCT(100));

        s_rows[i].name =
            ui_theme::make_label(row, s_motors[i].name,
                                 ui_theme::color_text(), ui_theme::font_body());
        lv_label_set_long_mode(s_rows[i].name, LV_LABEL_LONG_DOT);
        lv_obj_set_width(s_rows[i].name, kNameWidth);

        s_rows[i].value =
            ui_theme::make_label(row, "--", ui_theme::color_text(),
                                 ui_theme::font_body());
        lv_obj_set_width(s_rows[i].value, kValueWidth);
        lv_obj_set_style_text_align(s_rows[i].value, LV_TEXT_ALIGN_RIGHT,
                                    LV_PART_MAIN);

        s_rows[i].unit =
            ui_theme::make_label(row, "--", ui_theme::color_text_dim(),
                                 ui_theme::font_body());
        lv_obj_set_width(s_rows[i].unit, kUnitWidth);
        lv_obj_set_style_text_align(s_rows[i].unit, LV_TEXT_ALIGN_RIGHT,
                                    LV_PART_MAIN);
        lv_label_set_text(s_rows[i].unit, "C");
        ++created_rows;
        pros::lcd::print(1, "rows=%d", created_rows);
    }

    set_metric(s_metric);
    update();
    return s_root;
}

void update() {
    if (!s_root) {
        return;
    }
    const std::uint32_t now = pros::millis();
    if (now - s_last_update < kUpdateMs) {
        return;
    }
    s_last_update = now;

    set_label_cached(s_header_value, kMetricLabels[static_cast<size_t>(s_metric)],
                     s_last_header, sizeof(s_last_header));

    char buf[16];
    for (size_t i = 0; i < s_row_count; ++i) {
        const MotorEntry& entry = s_motors[i];
        const MetricValue metric = read_metric(entry, s_metric);
        std::snprintf(buf, sizeof(buf), metric.format, metric.value);
        set_label_cached(s_rows[i].value, buf, s_last_values[i],
                         sizeof(s_last_values[i]));
        set_label_cached(s_rows[i].unit, metric.unit, s_last_units[i],
                         sizeof(s_last_units[i]));
        lv_obj_set_style_text_color(s_rows[i].value, metric.color, LV_PART_MAIN);
    }
}
}
