#pragma once

#include "liblvgl/lvgl.h"

namespace ui_theme {
constexpr lv_coord_t kRadius = 12;
constexpr lv_coord_t kRadiusPill = 999;
constexpr lv_coord_t kBorder = 1;
constexpr lv_coord_t kPad = 10;
constexpr lv_coord_t kPadSm = 6;
constexpr lv_coord_t kTopBarHeight = 40;
constexpr lv_coord_t kLeftColumnWidth = 220;

constexpr uint32_t COLOR_BG = 0x0b1220;
constexpr uint32_t COLOR_PANEL = 0x111b2d;
constexpr uint32_t COLOR_PANEL_ALT = 0x141f33;
constexpr uint32_t COLOR_BORDER = 0x2b6aa8;
constexpr uint32_t COLOR_TEXT = 0xf3f6ff;
constexpr uint32_t COLOR_TEXT_DIM = 0x9fb1c9;
constexpr uint32_t COLOR_BTN = 0x15233a;
constexpr uint32_t COLOR_BTN_PRESSED = 0x1b2d4a;
constexpr uint32_t COLOR_BTN_SEL = 0x244a78;
constexpr uint32_t COLOR_ACCENT = 0x2f7cd4;

inline lv_color_t color_bg() { return lv_color_hex(COLOR_BG); }
inline lv_color_t color_panel() { return lv_color_hex(COLOR_PANEL); }
inline lv_color_t color_panel_alt() { return lv_color_hex(COLOR_PANEL_ALT); }
inline lv_color_t color_border() { return lv_color_hex(COLOR_BORDER); }
inline lv_color_t color_text() { return lv_color_hex(COLOR_TEXT); }
inline lv_color_t color_text_dim() { return lv_color_hex(COLOR_TEXT_DIM); }
inline lv_color_t color_btn() { return lv_color_hex(COLOR_BTN); }
inline lv_color_t color_btn_pressed() { return lv_color_hex(COLOR_BTN_PRESSED); }
inline lv_color_t color_btn_selected() { return lv_color_hex(COLOR_BTN_SEL); }
inline lv_color_t color_accent() { return lv_color_hex(COLOR_ACCENT); }

inline const lv_font_t* font_title() {
#if LV_FONT_MONTSERRAT_20
    return &lv_font_montserrat_20;
#elif LV_FONT_MONTSERRAT_18
    return &lv_font_montserrat_18;
#else
    return LV_FONT_DEFAULT;
#endif
}

inline const lv_font_t* font_body() {
    return LV_FONT_DEFAULT;
}

inline const lv_font_t* font_small() {
#if LV_FONT_MONTSERRAT_12
    return &lv_font_montserrat_12;
#else
    return LV_FONT_DEFAULT;
#endif
}

inline void apply_screen(lv_obj_t* obj) {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(obj, color_bg(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, kPad, LV_PART_MAIN);
    lv_obj_set_style_pad_row(obj, kPad, LV_PART_MAIN);
    lv_obj_set_style_pad_column(obj, kPad, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
}

inline void apply_panel(lv_obj_t* obj) {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(obj, color_panel(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    lv_obj_set_style_border_width(obj, kBorder, LV_PART_MAIN);
    lv_obj_set_style_border_color(obj, color_border(), LV_PART_MAIN);
    lv_obj_set_style_radius(obj, kRadius, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, kPad, LV_PART_MAIN);
    lv_obj_set_style_pad_row(obj, kPadSm, LV_PART_MAIN);
    lv_obj_set_style_pad_column(obj, kPadSm, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
}

inline void apply_button(lv_obj_t* obj) {
    lv_obj_remove_flag(obj, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_bg_color(obj, color_btn(), LV_PART_MAIN);
    lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, LV_PART_MAIN);
    const auto pressed_sel = static_cast<lv_style_selector_t>(LV_PART_MAIN) |
                             static_cast<lv_style_selector_t>(LV_STATE_PRESSED);
    const auto checked_sel = static_cast<lv_style_selector_t>(LV_PART_MAIN) |
                             static_cast<lv_style_selector_t>(LV_STATE_CHECKED);
    const auto checked_pressed_sel =
        static_cast<lv_style_selector_t>(LV_PART_MAIN) |
        static_cast<lv_style_selector_t>(LV_STATE_CHECKED | LV_STATE_PRESSED);
    lv_obj_set_style_bg_color(obj, color_btn_pressed(), pressed_sel);
    lv_obj_set_style_bg_color(obj, color_btn_selected(), checked_sel);
    lv_obj_set_style_bg_color(obj, color_btn_selected(), checked_pressed_sel);
    lv_obj_set_style_border_width(obj, kBorder, LV_PART_MAIN);
    lv_obj_set_style_border_color(obj, color_border(), LV_PART_MAIN);
    lv_obj_set_style_radius(obj, kRadius, LV_PART_MAIN);
    lv_obj_set_style_pad_all(obj, 0, LV_PART_MAIN);
    lv_obj_set_style_shadow_width(obj, 0, LV_PART_MAIN);
}

inline void set_button_selected(lv_obj_t* obj, bool selected) {
    if (selected) {
        lv_obj_add_state(obj, LV_STATE_CHECKED);
    } else {
        lv_obj_remove_state(obj, LV_STATE_CHECKED);
    }
}

inline lv_obj_t* make_label(lv_obj_t* parent, const char* text,
                            lv_color_t color = color_text(),
                            const lv_font_t* font = LV_FONT_DEFAULT) {
    lv_obj_t* label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_obj_set_style_text_color(label, color, LV_PART_MAIN);
    lv_obj_set_style_text_font(label, font, LV_PART_MAIN);
    return label;
}
}
