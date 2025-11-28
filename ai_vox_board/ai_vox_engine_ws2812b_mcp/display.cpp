#include <esp_log.h>

#include <algorithm>
#include <map>
#include <vector>

#include "esp_lvgl_port.h"
#include "font_awesome_symbols.h"
#include "font_emoji.h"
#include "display.h"

LV_FONT_DECLARE(font_puhui_16_4);
LV_FONT_DECLARE(font_awesome_30_4);
LV_FONT_DECLARE(font_awesome_16_4);

// Color definitions for light theme
#define LIGHT_BACKGROUND_COLOR lv_color_white()             // White background
#define LIGHT_TEXT_COLOR lv_color_black()                   // Black text
#define LIGHT_CHAT_BACKGROUND_COLOR lv_color_hex(0xE0E0E0)  // Light gray background
#define LIGHT_USER_BUBBLE_COLOR lv_color_hex(0x95EC69)      // WeChat green
#define LIGHT_ASSISTANT_BUBBLE_COLOR lv_color_white()       // White
#define LIGHT_SYSTEM_BUBBLE_COLOR lv_color_hex(0xE0E0E0)    // Light gray
#define LIGHT_SYSTEM_TEXT_COLOR lv_color_hex(0x666666)      // Dark gray text
#define LIGHT_BORDER_COLOR lv_color_hex(0xE0E0E0)           // Light gray border
#define LIGHT_LOW_BATTERY_COLOR lv_color_black()            // Black for light mode

Display::Display(esp_lcd_panel_io_handle_t panel_io,
                 esp_lcd_panel_handle_t panel,
                 int width,
                 int height,
                 int offset_x,
                 int offset_y,
                 bool mirror_x,
                 bool mirror_y,
                 bool swap_xy)
    : width_(width),
      height_(height),
      current_theme_{
          .background = LIGHT_BACKGROUND_COLOR,
          .text = LIGHT_TEXT_COLOR,
          .chat_background = LIGHT_CHAT_BACKGROUND_COLOR,
          .user_bubble = LIGHT_USER_BUBBLE_COLOR,
          .assistant_bubble = LIGHT_ASSISTANT_BUBBLE_COLOR,
          .system_bubble = LIGHT_SYSTEM_BUBBLE_COLOR,
          .system_text = LIGHT_SYSTEM_TEXT_COLOR,
          .border = LIGHT_BORDER_COLOR,
          .low_battery = LIGHT_LOW_BATTERY_COLOR,
      } {
  // draw white
  std::vector<uint16_t> buffer(width_, 0xFFFF);
  for (int y = 0; y < height_; y++) {
    esp_lcd_panel_draw_bitmap(panel, 0, y, width_, y + 1, buffer.data());
  }

  ESP_ERROR_CHECK(esp_lcd_panel_disp_on_off(panel, true));
  lv_init();

  lvgl_port_cfg_t port_cfg = ESP_LVGL_PORT_INIT_CONFIG();
  port_cfg.task_priority = tskIDLE_PRIORITY;
  port_cfg.timer_period_ms = 50;
  lvgl_port_init(&port_cfg);

  const lvgl_port_display_cfg_t display_cfg = {
      .io_handle = panel_io,
      .panel_handle = panel,
      .control_handle = nullptr,
      .buffer_size = static_cast<uint32_t>(width * 10),
      .double_buffer = false,
      .trans_size = 0,
      .hres = static_cast<uint32_t>(width),
      .vres = static_cast<uint32_t>(height),
      .monochrome = false,
      .rotation =
          {
              .swap_xy = swap_xy,
              .mirror_x = mirror_x,
              .mirror_y = mirror_y,
          },
      .color_format = LV_COLOR_FORMAT_RGB565,
      .flags =
          {
              .buff_dma = 1,
              .buff_spiram = 0,
              .sw_rotate = 0,
              .swap_bytes = 1,
              .full_refresh = 0,
              .direct_mode = 0,
          },
  };

  display_ = lvgl_port_add_disp(&display_cfg);

  assert(display_ != nullptr);
  if (display_ == nullptr) {
    abort();
    return;
  }

  if (offset_x != 0 || offset_y != 0) {
    lv_display_set_offset(display_, offset_x, offset_y);
  }
}

Display::~Display() {
  // TODO:
}

void Display::Start() {
  lvgl_port_lock(0);

  auto screen = lv_screen_active();
  lv_obj_set_style_text_font(screen, &font_puhui_16_4, 0);
  lv_obj_set_style_text_color(screen, current_theme_.text, 0);
  lv_obj_set_style_bg_color(screen, current_theme_.background, 0);

  /* Container */
  container_ = lv_obj_create(screen);
  lv_obj_set_size(container_, LV_HOR_RES, LV_VER_RES);
  lv_obj_set_flex_flow(container_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_style_pad_all(container_, 0, 0);
  lv_obj_set_style_border_width(container_, 0, 0);
  lv_obj_set_style_pad_row(container_, 0, 0);
  lv_obj_set_style_bg_color(container_, current_theme_.background, 0);
  lv_obj_set_style_border_color(container_, current_theme_.border, 0);

  /* Status bar */
  status_bar_ = lv_obj_create(container_);
  lv_obj_set_size(status_bar_, LV_HOR_RES, LV_SIZE_CONTENT);
  lv_obj_set_style_radius(status_bar_, 0, 0);
  lv_obj_set_style_bg_color(status_bar_, current_theme_.background, 0);
  lv_obj_set_style_text_color(status_bar_, current_theme_.text, 0);

  /* Content - Chat area */
  content_ = lv_obj_create(container_);
  lv_obj_set_style_radius(content_, 0, 0);
  lv_obj_set_width(content_, LV_HOR_RES);
  lv_obj_set_flex_grow(content_, 1);
  lv_obj_set_style_pad_all(content_, 10, 0);
  lv_obj_set_style_bg_color(content_, current_theme_.chat_background, 0);  // Background for chat area
  lv_obj_set_style_border_color(content_, current_theme_.border, 0);       // Border color for chat area

  // Enable scrolling for chat content
  lv_obj_set_scrollbar_mode(content_, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_scroll_dir(content_, LV_DIR_VER);

  // Create a flex container for chat messages
  lv_obj_set_flex_flow(content_, LV_FLEX_FLOW_COLUMN);
  lv_obj_set_flex_align(content_, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START, LV_FLEX_ALIGN_START);
  lv_obj_set_style_pad_row(content_, 10, 0);  // Space between messages

  // We'll create chat messages dynamically in SetChatMessage
  chat_message_label_ = nullptr;

  /* Status bar */
  lv_obj_set_flex_flow(status_bar_, LV_FLEX_FLOW_ROW);
  lv_obj_set_style_pad_all(status_bar_, 0, 0);
  lv_obj_set_style_border_width(status_bar_, 0, 0);
  lv_obj_set_style_pad_column(status_bar_, 0, 0);
  lv_obj_set_style_pad_left(status_bar_, 10, 0);
  lv_obj_set_style_pad_right(status_bar_, 10, 0);
  lv_obj_set_style_pad_top(status_bar_, 2, 0);
  lv_obj_set_style_pad_bottom(status_bar_, 2, 0);
  lv_obj_set_scrollbar_mode(status_bar_, LV_SCROLLBAR_MODE_OFF);
  // 设置状态栏的内容垂直居中
  lv_obj_set_flex_align(status_bar_, LV_FLEX_ALIGN_SPACE_BETWEEN, LV_FLEX_ALIGN_CENTER, LV_FLEX_ALIGN_CENTER);

  // 创建emotion_label_在状态栏最左侧
  emotion_label_ = lv_label_create(status_bar_);
  lv_obj_set_style_text_font(emotion_label_, &font_awesome_30_4, 0);
  lv_obj_set_style_text_color(emotion_label_, current_theme_.text, 0);
  lv_label_set_text(emotion_label_, FONT_AWESOME_AI_CHIP);
  lv_obj_set_style_margin_right(emotion_label_, 5, 0);  // 添加右边距，与后面的元素分隔

  notification_label_ = lv_label_create(status_bar_);
  lv_obj_set_flex_grow(notification_label_, 1);
  lv_obj_set_style_text_align(notification_label_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(notification_label_, current_theme_.text, 0);
  lv_label_set_text(notification_label_, "");
  lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);

  status_label_ = lv_label_create(status_bar_);
  lv_obj_set_flex_grow(status_label_, 1);
  lv_label_set_long_mode(status_label_, LV_LABEL_LONG_SCROLL_CIRCULAR);
  lv_obj_set_style_text_align(status_label_, LV_TEXT_ALIGN_CENTER, 0);
  lv_obj_set_style_text_color(status_label_, current_theme_.text, 0);
  lv_label_set_text(status_label_, "初始化");

  mute_label_ = lv_label_create(status_bar_);
  lv_label_set_text(mute_label_, "");
  lv_obj_set_style_text_font(mute_label_, &font_awesome_16_4, 0);
  lv_obj_set_style_text_color(mute_label_, current_theme_.text, 0);

  network_label_ = lv_label_create(status_bar_);
  lv_label_set_text(network_label_, "");
  lv_obj_set_style_text_font(network_label_, &font_awesome_16_4, 0);
  lv_obj_set_style_text_color(network_label_, current_theme_.text, 0);
  lv_obj_set_style_margin_left(network_label_, 5, 0);  // 添加左边距，与前面的元素分隔

#if 0
  battery_label_ = lv_label_create(status_bar_);
  lv_label_set_text(battery_label_, "");
  lv_obj_set_style_text_font(battery_label_, &font_awesome_16_4, 0);
  lv_obj_set_style_text_color(battery_label_, current_theme_.text, 0);
  lv_obj_set_style_margin_left(battery_label_, 5, 0);  // 添加左边距，与前面的元素分隔

  low_battery_popup_ = lv_obj_create(screen);
  lv_obj_set_scrollbar_mode(low_battery_popup_, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_size(low_battery_popup_, LV_HOR_RES * 0.9, &font_puhui_16_4->line_height * 2);
  lv_obj_align(low_battery_popup_, LV_ALIGN_BOTTOM_MID, 0, 0);
  lv_obj_set_style_bg_color(low_battery_popup_, current_theme_.low_battery, 0);
  lv_obj_set_style_radius(low_battery_popup_, 10, 0);
  lv_obj_t* low_battery_label = lv_label_create(low_battery_popup_);
  lv_label_set_text(low_battery_label, Lang::Strings::BATTERY_NEED_CHARGE);
  lv_obj_set_style_text_color(low_battery_label, lv_color_white(), 0);
  lv_obj_center(low_battery_label);
  lv_obj_add_flag(low_battery_popup_, LV_OBJ_FLAG_HIDDEN);
#endif
  lvgl_port_unlock();
}

#define MAX_MESSAGES (5)
void Display::SetChatMessage(const Role role, const std::string& content) {
  if (content.empty()) {
    return;
  }

  lvgl_port_lock(0);

  // 检查消息数量是否超过限制
  uint32_t child_count = lv_obj_get_child_cnt(content_);
  if (child_count >= MAX_MESSAGES) {
    // 删除最早的消息（第一个子对象）
    lv_obj_t* first_child = lv_obj_get_child(content_, 0);
    lv_obj_t* last_child = lv_obj_get_child(content_, child_count - 1);
    if (first_child != nullptr) {
      lv_obj_del(first_child);
    }
    // Scroll to the last message immediately
    if (last_child != nullptr) {
      lv_obj_scroll_to_view_recursive(last_child, LV_ANIM_OFF);
    }
  }

  // Create a message bubble
  lv_obj_t* msg_bubble = lv_obj_create(content_);
  lv_obj_set_style_radius(msg_bubble, 8, 0);
  lv_obj_set_scrollbar_mode(msg_bubble, LV_SCROLLBAR_MODE_OFF);
  lv_obj_set_style_border_width(msg_bubble, 1, 0);
  lv_obj_set_style_border_color(msg_bubble, current_theme_.border, 0);
  lv_obj_set_style_pad_all(msg_bubble, 8, 0);

  // Create the message text
  lv_obj_t* msg_text = lv_label_create(msg_bubble);
  lv_label_set_text(msg_text, content.c_str());

  // 计算文本实际宽度
  lv_coord_t text_width = lv_txt_get_width(content.c_str(), content.size(), &font_puhui_16_4, 0);

  // 计算气泡宽度
  lv_coord_t max_width = LV_HOR_RES * 85 / 100 - 16;  // 屏幕宽度的85%
  lv_coord_t min_width = 20;
  lv_coord_t bubble_width;

  // 确保文本宽度不小于最小宽度
  if (text_width < min_width) {
    text_width = min_width;
  }

  // 如果文本宽度小于最大宽度，使用文本宽度
  if (text_width < max_width) {
    bubble_width = text_width;
  } else {
    bubble_width = max_width;
  }

  // 设置消息文本的宽度
  lv_obj_set_width(msg_text, bubble_width);  // 减去padding
  lv_label_set_long_mode(msg_text, LV_LABEL_LONG_WRAP);
  lv_obj_set_style_text_font(msg_text, &font_puhui_16_4, 0);

  // 设置气泡宽度
  lv_obj_set_width(msg_bubble, bubble_width);
  lv_obj_set_height(msg_bubble, LV_SIZE_CONTENT);

  // Set alignment and style based on message role
  if (role == Role::kUser) {
    // User messages are right-aligned with green background
    lv_obj_set_style_bg_color(msg_bubble, current_theme_.user_bubble, 0);
    // Set text color for contrast
    lv_obj_set_style_text_color(msg_text, current_theme_.text, 0);

    // 设置自定义属性标记气泡类型
    lv_obj_set_user_data(msg_bubble, (void*)"user");

    // Set appropriate width for content
    lv_obj_set_width(msg_bubble, LV_SIZE_CONTENT);
    lv_obj_set_height(msg_bubble, LV_SIZE_CONTENT);

    // Don't grow
    lv_obj_set_style_flex_grow(msg_bubble, 0, 0);
  } else if (role == Role::kAssistant) {
    // Assistant messages are left-aligned with white background
    lv_obj_set_style_bg_color(msg_bubble, current_theme_.assistant_bubble, 0);
    // Set text color for contrast
    lv_obj_set_style_text_color(msg_text, current_theme_.text, 0);

    // 设置自定义属性标记气泡类型
    lv_obj_set_user_data(msg_bubble, (void*)"assistant");

    // Set appropriate width for content
    lv_obj_set_width(msg_bubble, LV_SIZE_CONTENT);
    lv_obj_set_height(msg_bubble, LV_SIZE_CONTENT);

    // Don't grow
    lv_obj_set_style_flex_grow(msg_bubble, 0, 0);
  } else if (role == Role::kSystem) {
    // System messages are center-aligned with light gray background
    lv_obj_set_style_bg_color(msg_bubble, current_theme_.system_bubble, 0);
    // Set text color for contrast
    lv_obj_set_style_text_color(msg_text, current_theme_.system_text, 0);

    // 设置自定义属性标记气泡类型
    lv_obj_set_user_data(msg_bubble, (void*)"system");

    // Set appropriate width for content
    lv_obj_set_width(msg_bubble, LV_SIZE_CONTENT);
    lv_obj_set_height(msg_bubble, LV_SIZE_CONTENT);

    // Don't grow
    lv_obj_set_style_flex_grow(msg_bubble, 0, 0);
  }

  // Create a full-width container for user messages to ensure right alignment
  if (role == Role::kUser) {
    // Create a full-width container
    lv_obj_t* container = lv_obj_create(content_);
    lv_obj_set_width(container, LV_HOR_RES);
    lv_obj_set_height(container, LV_SIZE_CONTENT);

    // Make container transparent and borderless
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_style_pad_all(container, 0, 0);

    // Move the message bubble into this container
    lv_obj_set_parent(msg_bubble, container);

    // Right align the bubble in the container
    lv_obj_align(msg_bubble, LV_ALIGN_RIGHT_MID, -25, 0);

    // Auto-scroll to this container
    lv_obj_scroll_to_view_recursive(container, LV_ANIM_ON);
  } else if (role == Role::kSystem) {
    // 为系统消息创建全宽容器以确保居中对齐
    lv_obj_t* container = lv_obj_create(content_);
    lv_obj_set_width(container, LV_HOR_RES);
    lv_obj_set_height(container, LV_SIZE_CONTENT);

    // 使容器透明且无边框
    lv_obj_set_style_bg_opa(container, LV_OPA_TRANSP, 0);
    lv_obj_set_style_border_width(container, 0, 0);
    lv_obj_set_style_pad_all(container, 0, 0);

    // 将消息气泡移入此容器
    lv_obj_set_parent(msg_bubble, container);

    // 将气泡居中对齐在容器中
    lv_obj_align(msg_bubble, LV_ALIGN_CENTER, 0, 0);

    // 自动滚动底部
    lv_obj_scroll_to_view_recursive(container, LV_ANIM_ON);
  } else {
    // For assistant messages
    // Left align assistant messages
    lv_obj_align(msg_bubble, LV_ALIGN_LEFT_MID, 0, 0);

    // Auto-scroll to the message bubble
    lv_obj_scroll_to_view_recursive(msg_bubble, LV_ANIM_ON);
  }

  // Store reference to the latest message label
  chat_message_label_ = msg_text;
  lvgl_port_unlock();
}

void Display::ShowStatus(const char* status) {
  lvgl_port_lock(0);
  lv_label_set_text(status_label_, status);
  lv_obj_clear_flag(status_label_, LV_OBJ_FLAG_HIDDEN);
  lv_obj_add_flag(notification_label_, LV_OBJ_FLAG_HIDDEN);
  lvgl_port_unlock();
}

void Display::SetEmotion(const std::string& emotion) {
  std::map<std::string, const char*> emotion_map = {
      {"neutral", "😶"}, {"happy", "🙂"},       {"laughing", "😆"},  {"funny", "😂"},     {"sad", "😔"},      {"angry", "😠"},   {"crying", "😭"},
      {"loving", "😍"},  {"embarrassed", "😳"}, {"surprised", "😯"}, {"shocked", "😱"},   {"thinking", "🤔"}, {"winking", "😉"}, {"cool", "😎"},
      {"relaxed", "😌"}, {"delicious", "🤤"},   {"kissy", "😘"},     {"confident", "😏"}, {"sleepy", "😴"},   {"silly", "😜"},   {"confused", "🙄"},
  };

  auto it = emotion_map.find(emotion);

  lvgl_port_lock(0);
  if (emotion_label_ == nullptr) {
    lvgl_port_unlock();
    return;
  }

  lv_obj_set_style_text_font(emotion_label_, font_emoji_32_init(), 0);
  if (it != emotion_map.end()) {
    lv_label_set_text(emotion_label_, it->second);
  } else {
    lv_label_set_text(emotion_label_, "😶");
  }
  lvgl_port_unlock();
}
