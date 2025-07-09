#pragma once

#ifndef _DISPLAY_H_
#define _DISPLAY_H_

#include <esp_lcd_panel_ops.h>

#include <string>

#include "lvgl.h"

class Display {
 public:
  enum class Role : uint8_t {
    kSystem,
    kAssistant,
    kUser,
  };
  Display(esp_lcd_panel_io_handle_t panel_io,
          esp_lcd_panel_handle_t panel,
          int width,
          int height,
          int offset_x,
          int offset_y,
          bool mirror_x,
          bool mirror_y,
          bool swap_xy);
  ~Display();
  void Start();
  void SetChatMessage(const Role role, const std::string& content);
  void ShowStatus(const char* status);
  void SetEmotion(const std::string& emotion);

 private:
  struct ThemeColors {
    lv_color_t background;
    lv_color_t text;
    lv_color_t chat_background;
    lv_color_t user_bubble;
    lv_color_t assistant_bubble;
    lv_color_t system_bubble;
    lv_color_t system_text;
    lv_color_t border;
    lv_color_t low_battery;
  };
  uint32_t width_ = 0;
  uint32_t height_ = 0;
  lv_display_t* display_ = nullptr;
  lv_obj_t* container_ = nullptr;
  lv_obj_t* status_bar_ = nullptr;
  lv_obj_t* content_ = nullptr;
  lv_obj_t* content_left_ = nullptr;
  lv_obj_t* content_right_ = nullptr;
  lv_obj_t* emotion_label_ = nullptr;
  lv_obj_t* chat_message_label_ = nullptr;
  lv_obj_t* network_label_ = nullptr;
  lv_obj_t* notification_label_ = nullptr;
  lv_obj_t* status_label_ = nullptr;
  lv_obj_t* mute_label_ = nullptr;
  ThemeColors current_theme_;
};

#endif