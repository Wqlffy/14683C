#include "ui_sensors.hpp"

#include <algorithm>
#include <cstddef>

#include "main.h"
#include "robot_config.hpp"
#include "sensor_health.hpp"
#include "../ui_theme.hpp"

namespace ui_sensors {
namespace {
constexpr lv_coord_t kHeaderPillHeight = 34;
constexpr lv_coord_t kRightColumnWidth = 190;
constexpr lv_coord_t kRowHeightSm = 90;
constexpr lv_coord_t kRowHeightMd = 110;
constexpr lv_coord_t kRowHeightLg = 170;

lv_obj_t* s_root = nullptr;

lv_obj_t* s_left_dist = nullptr;
lv_obj_t* s_left_vel = nullptr;
lv_obj_t* s_right_dist = nullptr;
lv_obj_t* s_right_vel = nullptr;
lv_obj_t* s_left_status = nullptr;
lv_obj_t* s_right_status = nullptr;

lv_obj_t* s_left_conf = nullptr;
lv_obj_t* s_left_size = nullptr;
lv_obj_t* s_right_conf = nullptr;
lv_obj_t* s_right_size = nullptr;

lv_obj_t* s_in_heading = nullptr;
lv_obj_t* s_in_rotation = nullptr;
lv_obj_t* s_in_pitch = nullptr;
lv_obj_t* s_in_roll = nullptr;
lv_obj_t* s_in_arc = nullptr;
lv_obj_t* s_imu_status = nullptr;

lv_obj_t* s_batt_volt = nullptr;
lv_obj_t* s_batt_curr = nullptr;
lv_obj_t* s_batt_temp = nullptr;
lv_obj_t* s_batt_cap = nullptr;
lv_obj_t* s_left_drive_status = nullptr;
lv_obj_t* s_right_drive_status = nullptr;
lv_obj_t* s_intake_status = nullptr;
lv_obj_t* s_outtake_status = nullptr;

constexpr uint32_t kOkColor = 0x4fd681;
constexpr uint32_t kWarnColor = 0xe0b84b;
constexpr uint32_t kFaultColor = 0xe05858;

void set_transparent(lv_obj_t* obj) {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_scroll_dir(obj, LV_DIR_NONE);  // Prevent parent containers from stealing scroll.
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

lv_color_t health_color(HealthState state) {
    switch (state) {
        case HealthState::OK:
            return lv_color_hex(kOkColor);
        case HealthState::WARNING:
            return lv_color_hex(kWarnColor);
        case HealthState::FAULT:
        default:
            return lv_color_hex(kFaultColor);
    }
}

const char* imu_status_text(ImuStatus status) {
    switch (status) {
        case ImuStatus::CALIBRATING:
            return "CALIBRATING";
        case ImuStatus::OK:
            return "OK";
        case ImuStatus::FAULT:
        default:
            return "FAULT";
    }
}

const char* distance_status_text(DistanceStatus status) {
    switch (status) {
        case DistanceStatus::OK:
            return "OK";
        case DistanceStatus::NO_TARGET:
            return "NO TARGET";
        case DistanceStatus::STUCK:
            return "STUCK";
        case DistanceStatus::FAULT:
        default:
            return "FAULT";
    }
}

const char* motor_status_text(MotorStatus status) {
    return status == MotorStatus::STALL ? "STALL DETECTED" : "NORMAL";
}
}

lv_obj_t* build(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    set_transparent(s_root);
    lv_obj_set_style_pad_row(s_root, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);

    make_header_pill(s_root, "SENSORS");

    lv_obj_t* content = lv_obj_create(s_root);
    set_transparent(content);
    lv_obj_set_style_pad_column(content, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_grow(content, 1);
    lv_obj_set_width(content, LV_PCT(100));

    // Left column: main grid
    lv_obj_t* grid = lv_obj_create(content);
    set_transparent(grid);
    lv_obj_add_flag(grid, LV_OBJ_FLAG_SCROLLABLE);  // Allow sensor grid to scroll when content exceeds view.
    lv_obj_add_flag(grid, LV_OBJ_FLAG_SCROLL_MOMENTUM |
                             LV_OBJ_FLAG_SCROLL_ELASTIC |
                             LV_OBJ_FLAG_SCROLL_CHAIN);  // Enable drag scrolling on the grid.
    lv_obj_set_scroll_dir(grid, LV_DIR_VER);  // Vertical scrolling for sensor grid.
    lv_obj_set_scrollbar_mode(grid, LV_SCROLLBAR_MODE_AUTO);  // Auto scrollbars for sensor grid.
    lv_obj_set_flex_flow(grid, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(grid, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_grow(grid, 1);
    lv_obj_set_height(grid, LV_PCT(100));  // Fixed height keeps grid from auto-expanding and blocking scroll.
    lv_obj_set_width(grid, LV_PCT(100));

    // Top row: distances
    lv_obj_t* row_top = lv_obj_create(grid);
    set_transparent(row_top);
    lv_obj_set_style_pad_column(row_top, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(row_top, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row_top, LV_PCT(100));
    lv_obj_set_height(row_top, kRowHeightSm);

    lv_obj_t* left_dist = lv_obj_create(row_top);
    ui_theme::apply_panel(left_dist);
    lv_obj_set_flex_flow(left_dist, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(left_dist, 1);
    make_title(left_dist, "LEFT DIST");
    s_left_dist = ui_theme::make_label(left_dist, "Dist: 0mm",
                                       ui_theme::color_text(),
                                       ui_theme::font_body());
    s_left_vel = ui_theme::make_label(left_dist, "Vel: 0.0m/s",
                                      ui_theme::color_text(),
                                      ui_theme::font_body());
    s_left_status = ui_theme::make_label(left_dist, "Status: OK",
                                         ui_theme::color_text_dim(),
                                         ui_theme::font_body());

    lv_obj_t* right_dist = lv_obj_create(row_top);
    ui_theme::apply_panel(right_dist);
    lv_obj_set_flex_flow(right_dist, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(right_dist, 1);
    make_title(right_dist, "RIGHT DIST");
    s_right_dist = ui_theme::make_label(right_dist, "Dist: 0mm",
                                        ui_theme::color_text(),
                                        ui_theme::font_body());
    s_right_vel = ui_theme::make_label(right_dist, "Vel: 0.0m/s",
                                       ui_theme::color_text(),
                                       ui_theme::font_body());
    s_right_status = ui_theme::make_label(right_dist, "Status: OK",
                                          ui_theme::color_text_dim(),
                                          ui_theme::font_body());

    // Middle row: confidence + size
    lv_obj_t* row_mid = lv_obj_create(grid);
    set_transparent(row_mid);
    lv_obj_set_style_pad_column(row_mid, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_flow(row_mid, LV_FLEX_FLOW_ROW);
    lv_obj_set_width(row_mid, LV_PCT(100));
    lv_obj_set_height(row_mid, kRowHeightMd);

    lv_obj_t* left_meta = lv_obj_create(row_mid);
    ui_theme::apply_panel(left_meta);
    lv_obj_set_flex_flow(left_meta, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(left_meta, 1);
    make_title(left_meta, "LEFT META");
    s_left_conf = ui_theme::make_label(left_meta, "Conf: 0",
                                       ui_theme::color_text_dim(),
                                       ui_theme::font_body());
    s_left_size = ui_theme::make_label(left_meta, "Size: 0",
                                       ui_theme::color_text(),
                                       ui_theme::font_body());

    lv_obj_t* right_meta = lv_obj_create(row_mid);
    ui_theme::apply_panel(right_meta);
    lv_obj_set_flex_flow(right_meta, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(right_meta, 1);
    make_title(right_meta, "RIGHT META");
    s_right_conf = ui_theme::make_label(right_meta, "Conf: 0",
                                        ui_theme::color_text_dim(),
                                        ui_theme::font_body());
    s_right_size = ui_theme::make_label(right_meta, "Size: 0",
                                        ui_theme::color_text(),
                                        ui_theme::font_body());

    // Bottom row: inertial
    lv_obj_t* row_bot = lv_obj_create(grid);
    set_transparent(row_bot);
    lv_obj_set_width(row_bot, LV_PCT(100));
    lv_obj_set_height(row_bot, kRowHeightLg);

    lv_obj_t* inertial = lv_obj_create(row_bot);
    ui_theme::apply_panel(inertial);
    lv_obj_set_flex_flow(inertial, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(inertial, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_width(inertial, LV_PCT(100));
    lv_obj_set_height(inertial, LV_PCT(100));

    lv_obj_t* inertial_left = lv_obj_create(inertial);
    set_transparent(inertial_left);
    lv_obj_set_flex_flow(inertial_left, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(inertial_left, 1);
    make_title(inertial_left, "INERTIAL");
    s_imu_status = ui_theme::make_label(inertial_left, "Status: OK",
                                        ui_theme::color_text_dim(),
                                        ui_theme::font_body());
    s_in_heading = ui_theme::make_label(inertial_left, "Heading: 0.0",
                                        ui_theme::color_text(),
                                        ui_theme::font_body());
    s_in_rotation = ui_theme::make_label(inertial_left, "Rotation: 0.0",
                                         ui_theme::color_text(),
                                         ui_theme::font_body());
    s_in_pitch = ui_theme::make_label(inertial_left, "Pitch: 0.0",
                                      ui_theme::color_text_dim(),
                                      ui_theme::font_body());
    s_in_roll = ui_theme::make_label(inertial_left, "Roll: 0.0",
                                     ui_theme::color_text_dim(),
                                     ui_theme::font_body());

    lv_obj_t* inertial_right = lv_obj_create(inertial);
    set_transparent(inertial_right);
    lv_obj_set_flex_flow(inertial_right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_align(inertial_right, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    lv_obj_set_width(inertial_right, 110);

    s_in_arc = lv_arc_create(inertial_right);
    lv_obj_set_size(s_in_arc, 90, 90);
    lv_arc_set_range(s_in_arc, 0, 360);
    lv_arc_set_value(s_in_arc, 0);
    lv_obj_set_style_arc_width(s_in_arc, 6, LV_PART_MAIN);
    lv_obj_set_style_arc_width(s_in_arc, 6, LV_PART_INDICATOR);
    lv_obj_set_style_arc_color(s_in_arc, ui_theme::color_panel_alt(),
                               LV_PART_MAIN);
    lv_obj_set_style_arc_color(s_in_arc, ui_theme::color_accent(),
                               LV_PART_INDICATOR);
    lv_obj_remove_style(s_in_arc, nullptr, LV_PART_KNOB);

    // Right column: battery card
    lv_obj_t* right_col = lv_obj_create(content);
    set_transparent(right_col);
    lv_obj_set_width(right_col, kRightColumnWidth);
    lv_obj_set_flex_flow(right_col, LV_FLEX_FLOW_COLUMN);

    lv_obj_t* battery = lv_obj_create(right_col);
    ui_theme::apply_panel(battery);
    lv_obj_set_flex_flow(battery, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_width(battery, LV_PCT(100));
    make_title(battery, "BATTERY");
    s_batt_volt = ui_theme::make_label(battery, "Volt: 0.0V",
                                       ui_theme::color_text(),
                                       ui_theme::font_body());
    s_batt_curr = ui_theme::make_label(battery, "Curr: 0.0A",
                                       ui_theme::color_text_dim(),
                                       ui_theme::font_body());
    s_batt_temp = ui_theme::make_label(battery, "Temp: 0.0C",
                                       ui_theme::color_text_dim(),
                                       ui_theme::font_body());
    s_batt_cap = ui_theme::make_label(battery, "Cap: 0%",
                                      ui_theme::color_text_dim(),
                                      ui_theme::font_body());

    make_title(battery, "MOTOR STATUS");
    s_left_drive_status =
        ui_theme::make_label(battery, "Left: NORMAL",
                             ui_theme::color_text_dim(),
                             ui_theme::font_body());
    s_right_drive_status =
        ui_theme::make_label(battery, "Right: NORMAL",
                             ui_theme::color_text_dim(),
                             ui_theme::font_body());
    s_intake_status =
        ui_theme::make_label(battery, "Intake: NORMAL",
                             ui_theme::color_text_dim(),
                             ui_theme::font_body());
    s_outtake_status =
        ui_theme::make_label(battery, "Outtake: NORMAL",
                             ui_theme::color_text_dim(),
                             ui_theme::font_body());

    update();
    return s_root;
}

void update() {
    if (!s_root) {
        return;
    }

    sensor_health_update();
    const SensorHealth& health = get_sensor_health();
    const SensorDetail& detail = get_sensor_detail();

    const int left_mm = leftDist.get();
    const int left_conf = leftDist.get_confidence();
    const int left_size = leftDist.get_object_size();
    const double left_vel = leftDist.get_object_velocity();

    const int right_mm = rightDist.get();
    const int right_conf = rightDist.get_confidence();
    const int right_size = rightDist.get_object_size();
    const double right_vel = rightDist.get_object_velocity();

    lv_label_set_text_fmt(s_left_dist, "Dist: %dmm", left_mm);
    lv_label_set_text_fmt(s_left_vel, "Vel: %.2fm/s", left_vel);
    lv_label_set_text_fmt(s_right_dist, "Dist: %dmm", right_mm);
    lv_label_set_text_fmt(s_right_vel, "Vel: %.2fm/s", right_vel);

    lv_label_set_text_fmt(s_left_conf, "Conf: %d", left_conf);
    lv_label_set_text_fmt(s_left_size, "Size: %d", left_size);
    lv_label_set_text_fmt(s_right_conf, "Conf: %d", right_conf);
    lv_label_set_text_fmt(s_right_size, "Size: %d", right_size);

    const double heading = imu.get_heading();
    const double rotation = imu.get_rotation();
    const double pitch = imu.get_pitch();
    const double roll = imu.get_roll();

    lv_label_set_text_fmt(s_in_heading, "Heading: %.1f", heading);
    lv_label_set_text_fmt(s_in_rotation, "Rotation: %.1f", rotation);
    lv_label_set_text_fmt(s_in_pitch, "Pitch: %.1f", pitch);
    lv_label_set_text_fmt(s_in_roll, "Roll: %.1f", roll);
    lv_arc_set_value(s_in_arc, static_cast<int>(heading));

    lv_label_set_text_fmt(s_left_status, "Status: %s",
                          distance_status_text(detail.left_dist_status));
    lv_obj_set_style_text_color(
        s_left_status, health_color(health.leftDist), LV_PART_MAIN);

    lv_label_set_text_fmt(s_right_status, "Status: %s",
                          distance_status_text(detail.right_dist_status));
    lv_obj_set_style_text_color(
        s_right_status, health_color(health.rightDist), LV_PART_MAIN);

    lv_label_set_text_fmt(s_imu_status, "Status: %s",
                          imu_status_text(detail.imu_status));
    lv_obj_set_style_text_color(
        s_imu_status, health_color(health.imu), LV_PART_MAIN);

    lv_label_set_text_fmt(s_left_drive_status, "Left: %s",
                          motor_status_text(detail.left_drive_status));
    lv_obj_set_style_text_color(
        s_left_drive_status, health_color(health.leftDrive), LV_PART_MAIN);

    lv_label_set_text_fmt(s_right_drive_status, "Right: %s",
                          motor_status_text(detail.right_drive_status));
    lv_obj_set_style_text_color(
        s_right_drive_status, health_color(health.rightDrive), LV_PART_MAIN);

    lv_label_set_text_fmt(s_intake_status, "Intake: %s",
                          motor_status_text(detail.intake_status));
    lv_obj_set_style_text_color(
        s_intake_status, health_color(health.intakeMotor), LV_PART_MAIN);

    lv_label_set_text_fmt(s_outtake_status, "Outtake: %s",
                          motor_status_text(detail.outtake_status));
    lv_obj_set_style_text_color(
        s_outtake_status, health_color(health.outtakeMotor), LV_PART_MAIN);

    const double batt_volt = pros::battery::get_voltage() / 1000.0;
    const double batt_curr = pros::battery::get_current() / 1000.0;
    const double batt_temp = pros::battery::get_temperature();
    const double batt_cap = pros::battery::get_capacity();

    lv_label_set_text_fmt(s_batt_volt, "Volt: %.1fV", batt_volt);
    lv_label_set_text_fmt(s_batt_curr, "Curr: %.1fA", batt_curr);
    lv_label_set_text_fmt(s_batt_temp, "Temp: %.1fC", batt_temp);
    lv_label_set_text_fmt(s_batt_cap, "Cap: %.0f%%", batt_cap);
}
}
