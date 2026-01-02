#include "ui_autons.hpp"

#include <cstddef>

#include "main.h"
#include "../ui_theme.hpp"

namespace ui_autons {
namespace {
lv_obj_t* s_root = nullptr;
lv_obj_t* s_timer_label = nullptr;
lv_obj_t* s_detail_title = nullptr;
lv_obj_t* s_detail_desc = nullptr;
lv_obj_t* s_detail_img = nullptr;
lv_obj_t* s_buttons[AUTON_COUNT] = {};

void set_transparent(lv_obj_t* obj) {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
}

void on_auton_event(lv_event_t* e) {
    const auto* info =
        static_cast<const AutonInfo*>(lv_event_get_user_data(e));
    if (!info) {
        return;
    }

    const lv_event_code_t code = lv_event_get_code(e);
    if (code == LV_EVENT_CLICKED) {
        set_detail(info->id);
    } else if (code == LV_EVENT_LONG_PRESSED) {
        g_selected_auton = info->id;
        set_selected(info->id);
        set_detail(info->id);
    }
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
    lv_obj_t* arrow =
        ui_theme::make_label(pill, "˅", ui_theme::color_text_dim(),
                             ui_theme::font_body());
    lv_obj_set_flex_grow(arrow, 1);
    lv_obj_set_style_text_align(arrow, LV_TEXT_ALIGN_RIGHT, LV_PART_MAIN);
    return pill;
}
}

lv_obj_t* build(lv_obj_t* parent) {
    s_root = lv_obj_create(parent);
    lv_obj_set_size(s_root, LV_PCT(100), LV_PCT(100));
    lv_obj_set_flex_flow(s_root, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(s_root, ui_theme::kPad, LV_PART_MAIN);
    set_transparent(s_root);

    lv_obj_t* top = lv_obj_create(s_root);
    ui_theme::apply_panel(top);
    lv_obj_set_height(top, ui_theme::kTopBarHeight);
    lv_obj_set_width(top, LV_PCT(100));
    lv_obj_set_flex_flow(top, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top, LV_FLEX_ALIGN_SPACE_BETWEEN,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

    lv_obj_t* top_left = lv_obj_create(top);
    set_transparent(top_left);
    lv_obj_set_flex_flow(top_left, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(top_left, ui_theme::kPadSm, LV_PART_MAIN);
    ui_theme::make_label(top_left, "Joyboy", ui_theme::color_text(),
                         ui_theme::font_body());

    lv_obj_t* top_center = lv_obj_create(top);
    set_transparent(top_center);
    lv_obj_set_flex_grow(top_center, 1);
    lv_obj_set_flex_flow(top_center, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(top_center, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    s_timer_label =
        ui_theme::make_label(top_center, "0:17", ui_theme::color_text(),
                             ui_theme::font_title());

    lv_obj_t* top_right = lv_obj_create(top);
    set_transparent(top_right);
    lv_obj_set_flex_flow(top_right, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(top_right, ui_theme::kPadSm, LV_PART_MAIN);
    ui_theme::make_label(top_right, "BAT", ui_theme::color_text_dim(),
                         ui_theme::font_small());
    ui_theme::make_label(top_right, "LINK", ui_theme::color_text_dim(),
                         ui_theme::font_small());

    lv_obj_t* content = lv_obj_create(s_root);
    set_transparent(content);
    lv_obj_set_flex_flow(content, LV_FLEX_FLOW_ROW);
    lv_obj_set_style_pad_column(content, ui_theme::kPad, LV_PART_MAIN);
    lv_obj_set_flex_grow(content, 1);
    lv_obj_set_width(content, LV_PCT(100));

    lv_obj_t* left = lv_obj_create(content);
    ui_theme::apply_panel(left);
    lv_obj_set_width(left, ui_theme::kLeftColumnWidth);
    lv_obj_set_flex_flow(left, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_style_pad_row(left, ui_theme::kPadSm, LV_PART_MAIN);

    make_header_pill(left, "AUTONS");

    lv_obj_t* list = lv_obj_create(left);
    lv_obj_set_flex_flow(list, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(list, 1);
    lv_obj_set_width(list, LV_PCT(100));
    lv_obj_set_scroll_dir(list, LV_DIR_VER);
    lv_obj_set_style_bg_opa(list, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_border_width(list, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(list, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_row(list, ui_theme::kPadSm, LV_PART_MAIN);
    lv_obj_set_style_width(list, 4, LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_color(list, ui_theme::color_border(),
                              LV_PART_SCROLLBAR);
    lv_obj_set_style_bg_opa(list, LV_OPA_50, LV_PART_SCROLLBAR);
    lv_obj_set_scrollbar_mode(list, LV_SCROLLBAR_MODE_ACTIVE);

    for (size_t i = 0; i < AUTON_COUNT; ++i) {
        const AutonInfo* info = &AUTONS[i];
        lv_obj_t* btn = lv_button_create(list);
        ui_theme::apply_button(btn);
        lv_obj_set_width(btn, LV_PCT(100));
        lv_obj_set_height(btn, 44);
        lv_obj_add_event_cb(btn, on_auton_event, LV_EVENT_CLICKED,
                            const_cast<AutonInfo*>(info));
        lv_obj_add_event_cb(btn, on_auton_event, LV_EVENT_LONG_PRESSED,
                            const_cast<AutonInfo*>(info));

        lv_obj_t* label =
            ui_theme::make_label(btn, info->name, ui_theme::color_text(),
                                 ui_theme::font_body());
        lv_obj_center(label);
        s_buttons[i] = btn;
    }

    lv_obj_t* right = lv_obj_create(content);
    set_transparent(right);
    lv_obj_set_flex_flow(right, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(right, 1);
    lv_obj_set_width(right, LV_PCT(100));
    lv_obj_set_style_pad_row(right, ui_theme::kPadSm, LV_PART_MAIN);

    lv_obj_t* banner = lv_obj_create(right);
    ui_theme::apply_panel(banner);
    lv_obj_set_height(banner, 60);
    lv_obj_set_width(banner, LV_PCT(100));
    lv_obj_set_style_bg_color(banner, ui_theme::color_panel_alt(),
                              LV_PART_MAIN);
    lv_obj_set_flex_flow(banner, LV_FLEX_FLOW_ROW);
    lv_obj_set_flex_align(banner, LV_FLEX_ALIGN_CENTER,
                          LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);
    ui_theme::make_label(banner, "ARSENAL", ui_theme::color_text(),
                         ui_theme::font_title());

    lv_obj_t* detail = lv_obj_create(right);
    ui_theme::apply_panel(detail);
    lv_obj_set_flex_flow(detail, LV_FLEX_FLOW_COLUMN);
    lv_obj_set_flex_grow(detail, 1);
    lv_obj_set_width(detail, LV_PCT(100));
    lv_obj_set_style_pad_row(detail, ui_theme::kPadSm, LV_PART_MAIN);

    s_detail_title =
        ui_theme::make_label(detail, "AUTON", ui_theme::color_text(),
                             ui_theme::font_title());
    s_detail_desc = ui_theme::make_label(detail, "--",
                                         ui_theme::color_text_dim(),
                                         ui_theme::font_body());
    lv_label_set_long_mode(s_detail_desc, LV_LABEL_LONG_WRAP);
    lv_obj_set_width(s_detail_desc, LV_PCT(100));

    s_detail_img = lv_image_create(detail);
    lv_obj_set_size(s_detail_img, 180, 120);
    lv_obj_add_flag(s_detail_img, LV_OBJ_FLAG_HIDDEN);

    set_selected(g_selected_auton);
    set_detail(g_selected_auton);
    return s_root;
}

void set_selected(AutonId id) {
    for (size_t i = 0; i < AUTON_COUNT; ++i) {
        const bool selected = AUTONS[i].id == id;
        if (s_buttons[i]) {
            ui_theme::set_button_selected(s_buttons[i], selected);
        }
    }
}

void set_detail(AutonId id) {
    const AutonInfo* info = get_auton_info(id);
    if (!info || !s_detail_title || !s_detail_desc || !s_detail_img) {
        return;
    }
    lv_label_set_text(s_detail_title, info->name);
    lv_label_set_text(s_detail_desc, info->desc);

    if (info->img_src) {
        lv_image_set_src(s_detail_img, info->img_src);
        lv_obj_remove_flag(s_detail_img, LV_OBJ_FLAG_HIDDEN);
    } else {
        lv_obj_add_flag(s_detail_img, LV_OBJ_FLAG_HIDDEN);
    }
}

void update() {
    if (!s_timer_label) {
        return;
    }
    const int total_sec = pros::millis() / 1000;
    const int minutes = total_sec / 60;
    const int seconds = total_sec % 60;
    lv_label_set_text_fmt(s_timer_label, "%d:%02d", minutes, seconds);
}
}
