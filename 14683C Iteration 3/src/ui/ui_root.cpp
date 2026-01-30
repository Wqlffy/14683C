#include "ui_root.hpp"

#include <cstddef>
#include <cstdint>
#include <cstdio>

#include "liblvgl/lvgl.h"
#include "main.h"
#include "pages/ui_autons.hpp"
#include "pages/ui_motors.hpp"
#include "pages/ui_sensors.hpp"
#include "pages/ui_odomcalib.hpp"
#include "ui_theme.hpp"

extern "C" {
namespace pros {
namespace c {
void display_mutex_take(void) __attribute__((weak));
void display_mutex_give(void) __attribute__((weak));
}
}
}

extern "C" {
namespace pros {
namespace c {
void display_mutex_take(void) { lv_lock(); }
void display_mutex_give(void) { lv_unlock(); }
}
}
}

namespace ui_root {
namespace {
constexpr size_t kPageCount = 4;
lv_obj_t* s_pages[kPageCount] = {};
lv_obj_t* s_nav_buttons[kPageCount] = {};
Page s_active = Page::Autons;
bool s_inited = false;

void show_page(Page id) {
    s_active = id;
    for (size_t i = 0; i < kPageCount; ++i) {
        const bool active = (i == static_cast<size_t>(id));
        if (s_pages[i]) {
            if (active) {
                lv_obj_remove_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
            } else {
                lv_obj_add_flag(s_pages[i], LV_OBJ_FLAG_HIDDEN);
            }
        }
        if (s_nav_buttons[i]) {
            ui_theme::set_button_selected(s_nav_buttons[i], active);
        }
    }
}

void on_nav_event(lv_event_t* e) {
    const intptr_t raw = reinterpret_cast<intptr_t>(lv_event_get_user_data(e));
    const auto id = static_cast<Page>(raw);
    show_page(id);
}
}

void init() {
    if (s_inited) {
        return;
    }
    pros::c::display_mutex_take();  // Fix: LVGL was updated without the PROS display mutex.
    lv_obj_t* scr = lv_screen_active();
    ui_theme::apply_screen(scr);
    lv_obj_set_flex_flow(scr, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(scr, ui_theme::kPad, LV_PART_MAIN);

    lv_obj_t* nav = lv_obj_create(scr);
    ui_theme::apply_panel(nav);
    lv_obj_set_height(nav, 46);
    lv_obj_set_width(nav, LV_PCT(100));
    lv_obj_set_flex_flow(nav, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(nav, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_flex_align(nav, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER);

    const char* nav_labels[kPageCount] = {"AUTONS", "MOTORS", "SENSORS", "ODOM"};
    for (size_t i = 0; i < kPageCount; ++i) {
        lv_obj_t* btn = lv_button_create(nav);
        ui_theme::apply_button(btn);
        lv_obj_set_flex_grow(btn, 1);
        lv_obj_set_height(btn, 36);
        lv_obj_add_event_cb(
            btn, on_nav_event, LV_EVENT_CLICKED,
            reinterpret_cast<void*>(static_cast<intptr_t>(i)));
        lv_obj_t* label =
            ui_theme::make_label(btn, nav_labels[i], ui_theme::color_text(),
                                 ui_theme::font_body());
        lv_obj_center(label);
        s_nav_buttons[i] = btn;
    }

    lv_obj_t* content = lv_obj_create(scr);
    lv_obj_remove_flag(content, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(content, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(content, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(content, 0, LV_PART_MAIN);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(content, 1);
    lv_obj_set_width(content, LV_PCT(100));

    s_pages[0] = ui_autons::build(content);
    s_pages[1] = ui_motors::build(content);
    s_pages[2] = ui_sensors::build(content);
    s_pages[3] = ui_odomcalib::build(content);

    show_page(Page::Autons);
    s_inited = true;
    pros::c::display_mutex_give();  // Release display mutex after LVGL init work.
}

void update_fast() {
    if (!s_inited) {
        return;
    }
    const std::uint32_t now = pros::millis();
    pros::c::display_mutex_take();  // Guard all LVGL operations in UI task.
    lv_timer_handler();
    switch (s_active) {
        case Page::Autons:
            ui_autons::update();
            break;
        case Page::Motors:
            ui_motors::update();
            break;
        case Page::Sensors:
            ui_sensors::update();
            break;
        case Page::Odom:
            ui_odomcalib::update();
            break;
    }
    pros::c::display_mutex_give();  // Allow LVGL's own task to resume safely.
}

Page active_page() {
    return s_active;
}
}
