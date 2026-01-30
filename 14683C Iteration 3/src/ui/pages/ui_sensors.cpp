#include "ui_sensors.hpp"

#include <cstddef>
#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>

#include "main.h"
#include "robot_config.hpp"
#include "auton_recovery.hpp"
#include "../ui_theme.hpp"

namespace ui_sensors {
namespace {
constexpr lv_coord_t kHeaderPillHeight = 34;
constexpr std::uint32_t kUpdateMs = 100;

lv_obj_t* s_root = nullptr;
lv_obj_t* s_imu_label = nullptr;
lv_obj_t* s_left_label = nullptr;
lv_obj_t* s_right_label = nullptr;

std::uint32_t s_last_update = 0;
char s_last_imu[32] = "";
char s_last_left[24] = "";
char s_last_right[24] = "";

bool distance_valid(int mm) {
    return AutonRecovery::distValidMm(mm) && mm != 0 && mm != 9999;
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

#define SENSORS_DEBUG 0
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
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(content, 1);
    lv_obj_set_width(content, LV_PCT(100));

    lv_obj_t* panel = lv_obj_create(content);
    ui_theme::apply_panel(panel);
    lv_obj_set_flex_flow(panel, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(panel, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_width(panel, LV_PCT(100));

    make_title(panel, "IMU");
    s_imu_label = ui_theme::make_label(panel, "IMU: calibrating",
                                       ui_theme::color_text(),
                                       ui_theme::font_body());

    make_title(panel, "DISTANCE");
    s_left_label = ui_theme::make_label(panel, "Dist L: ---",
                                        ui_theme::color_text(),
                                        ui_theme::font_body());
    s_right_label = ui_theme::make_label(panel, "Dist R: ---",
                                         ui_theme::color_text(),
                                         ui_theme::font_body());

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

    const bool imu_cal = imu.is_calibrating();
    if (imu_cal) {
        set_label_cached(s_imu_label, "IMU: calibrating", s_last_imu,
                         sizeof(s_last_imu));
    } else {
        const double heading = imu.get_heading();
        if (std::isfinite(heading)) {
            char buf[32];
            std::snprintf(buf, sizeof(buf), "IMU: %.1f°", heading);
            set_label_cached(s_imu_label, buf, s_last_imu, sizeof(s_last_imu));
        } else {
            set_label_cached(s_imu_label, "IMU: ---", s_last_imu,
                             sizeof(s_last_imu));
        }
    }

    const int left_mm = leftDist.get();
    const int right_mm = rightDist.get();
    const bool left_valid = distance_valid(left_mm);
    const bool right_valid = distance_valid(right_mm);

    if (left_valid) {
        char buf[24];
        std::snprintf(buf, sizeof(buf), "Dist L: %d mm", left_mm);
        set_label_cached(s_left_label, buf, s_last_left, sizeof(s_last_left));
    } else {
        set_label_cached(s_left_label, "Dist L: ---", s_last_left,
                         sizeof(s_last_left));
    }
    if (right_valid) {
        char buf[24];
        std::snprintf(buf, sizeof(buf), "Dist R: %d mm", right_mm);
        set_label_cached(s_right_label, buf, s_last_right, sizeof(s_last_right));
    } else {
        set_label_cached(s_right_label, "Dist R: ---", s_last_right,
                         sizeof(s_last_right));
    }

#if SENSORS_DEBUG
    static std::uint32_t last_dbg = 0;
    if (now - last_dbg >= 1000) {
        last_dbg = now;
        std::printf("SENSORS: imu_cal=%d heading=%.2f L=%d v=%d R=%d v=%d\n",
                    static_cast<int>(imu_cal),
                    imu.get_heading(),
                    left_mm,
                    static_cast<int>(left_valid),
                    right_mm,
                    static_cast<int>(right_valid));
    }
#endif
}
}
