#include <Arduino.h>
#include <WiFi.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#include "ai_vox_engine.h"
#include "audio_device/audio_output_device_i2s_std.h"
#include "audio_input_device_sph0645.h"
#include "components/cjson_util/cjson_util.h"
#include "components/espressif/button/button_gpio.h"
#include "components/espressif/button/iot_button.h"
#include "components/espressif/esp_audio_codec/esp_audio_simple_dec.h"
#include "components/espressif/esp_audio_codec/esp_mp3_dec.h"
#include "components/wifi_configurator/wifi_configurator.h"
#include "display.h"
#include "network_config_mode_mp3.h"
#include "network_connected_mp3.h"
#include "notification_0_mp3.h"

#ifndef ARDUINO_ESP32S3_DEV
#error "This example only supports ESP32S3-Dev board."
#endif

#ifndef CONFIG_SPIRAM_MODE_OCT
#error "This example requires PSRAM to OPI PSRAM. Please enable it in Arduino IDE."
#endif

/**
 * 如果开机自动连接WiFi而不需要配置WiFi, 请注释掉下面的宏, 然后请修改下面的WIFI_SSID和WIFI_PASSWORD
 */
// #define WIFI_SSID "your_wifi_ssid"
// #define WIFI_PASSWORD "your_wifi_password"

namespace {
/**
 *  SC_TYPE_ESPTOUCH            协议: ESPTouch
 *  SC_TYPE_AIRKISS,            协议: AirKiss
 *  SC_TYPE_ESPTOUCH_AIRKISS,   协议: ESPTouch 和 AirKiss
 *  SC_TYPE_ESPTOUCH_V2,        协议: ESPTouch v2
 */
constexpr smartconfig_type_t kSmartConfigType = SC_TYPE_ESPTOUCH_AIRKISS;  // ESPTouch and AirKiss

const std::vector<gpio_num_t> kServoPins = {GPIO_NUM_46, GPIO_NUM_47};

constexpr gpio_num_t kMicPinSck = GPIO_NUM_5;
constexpr gpio_num_t kMicPinWs = GPIO_NUM_2;
constexpr gpio_num_t kMicPinSd = GPIO_NUM_4;

constexpr gpio_num_t kSpeakerPinSck = GPIO_NUM_13;
constexpr gpio_num_t kSpeakerPinWs = GPIO_NUM_14;
constexpr gpio_num_t kSpeakerPinSd = GPIO_NUM_1;

constexpr gpio_num_t kButtonBoot = GPIO_NUM_0;

constexpr gpio_num_t kDisplayBacklightPin = GPIO_NUM_11;
constexpr gpio_num_t kDisplayMosiPin = GPIO_NUM_17;
constexpr gpio_num_t kDisplayClkPin = GPIO_NUM_16;
constexpr gpio_num_t kDisplayDcPin = GPIO_NUM_12;
constexpr gpio_num_t kDisplayRstPin = GPIO_NUM_21;
constexpr gpio_num_t kDisplayCsPin = GPIO_NUM_15;

constexpr auto kDisplaySpiMode = 0;
constexpr uint32_t kDisplayWidth = 240;
constexpr uint32_t kDisplayHeight = 240;
constexpr bool kDisplayMirrorX = false;
constexpr bool kDisplayMirrorY = false;
constexpr bool kDisplayInvertColor = true;
constexpr bool kDisplaySwapXY = false;
constexpr auto kDisplayRgbElementOrder = LCD_RGB_ELEMENT_ORDER_RGB;

auto g_audio_output_device = std::make_shared<ai_vox::AudioOutputDeviceI2sStd>(kSpeakerPinSck, kSpeakerPinWs, kSpeakerPinSd);
button_handle_t g_button_boot_handle = nullptr;

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();

constexpr uint8_t kServoFrequency = 50;
constexpr uint8_t kServoResolution = 12;
constexpr uint32_t kMinPulse = 500;
constexpr uint32_t kMaxPulse = 2500;
constexpr uint16_t kMaxServoAngle = 180;

constexpr uint32_t kMaxPwmDuty = pow(2, kServoResolution) - 1;
constexpr uint32_t kPwmFactor = 1000 * 1000 / kServoFrequency;

constexpr uint8_t kDisplayBacklightChannel = 7;
constexpr uint32_t kDisplayBacklightFrequency = 1000;
constexpr uint8_t kDisplayBacklightResolution = 8;

std::vector<uint16_t> g_servo_angles(kServoPins.size(), 90);

uint8_t g_display_brightness = 255;

void InitDisplay() {
  printf("init display\n");

  if (!ledcAttachChannel(kDisplayBacklightPin, kDisplayBacklightFrequency, kDisplayBacklightResolution, kDisplayBacklightChannel)) {
    printf("Error: Failed to attach LCD backlight LEDC channel.\n");
    return;
  }

  if (!ledcWriteChannel(kDisplayBacklightChannel, g_display_brightness)) {
    printf("Error: Failed to set LCD backlight brightness.\n");
    return;
  }

  spi_bus_config_t buscfg{
      .mosi_io_num = kDisplayMosiPin,
      .miso_io_num = GPIO_NUM_NC,
      .sclk_io_num = kDisplayClkPin,
      .quadwp_io_num = GPIO_NUM_NC,
      .quadhd_io_num = GPIO_NUM_NC,
      .data4_io_num = GPIO_NUM_NC,
      .data5_io_num = GPIO_NUM_NC,
      .data6_io_num = GPIO_NUM_NC,
      .data7_io_num = GPIO_NUM_NC,
      .data_io_default_level = false,
      .max_transfer_sz = kDisplayWidth * kDisplayHeight * sizeof(uint16_t),
      .flags = 0,
      .isr_cpu_id = ESP_INTR_CPU_AFFINITY_AUTO,
      .intr_flags = 0,
  };
  ESP_ERROR_CHECK(spi_bus_initialize(SPI3_HOST, &buscfg, SPI_DMA_CH_AUTO));

  esp_lcd_panel_io_handle_t panel_io = nullptr;
  esp_lcd_panel_handle_t panel = nullptr;

  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.cs_gpio_num = kDisplayCsPin;
  io_config.dc_gpio_num = kDisplayDcPin;
  io_config.spi_mode = kDisplaySpiMode;
  io_config.pclk_hz = 40 * 1000 * 1000;
  io_config.trans_queue_depth = 10;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = kDisplayRstPin;
  panel_config.rgb_ele_order = kDisplayRgbElementOrder;
  panel_config.bits_per_pixel = 16;
  ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel));

  esp_lcd_panel_reset(panel);

  esp_lcd_panel_init(panel);
  esp_lcd_panel_invert_color(panel, kDisplayInvertColor);
  esp_lcd_panel_swap_xy(panel, kDisplaySwapXY);
  esp_lcd_panel_mirror(panel, kDisplayMirrorX, kDisplayMirrorY);

  g_display = std::make_unique<Display>(panel_io, panel, kDisplayWidth, kDisplayHeight, 0, 0, kDisplayMirrorX, kDisplayMirrorY, kDisplaySwapXY);
  g_display->Start();
}

#ifdef PRINT_HEAP_INFO_INTERVAL
void PrintMemInfo() {
  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_SPIRAM);
    printf("SPIRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }

  if (heap_caps_get_total_size(MALLOC_CAP_INTERNAL) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_INTERNAL);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_INTERNAL);
    printf("IRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }

  if (heap_caps_get_total_size(MALLOC_CAP_DEFAULT) > 0) {
    const auto total_size = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
    const auto free_size = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
    const auto min_free_size = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
    printf("DRAM total size: %zu B (%zu KB), free size: %zu B (%zu KB), minimum free size: %zu B (%zu KB)\n",
           total_size,
           total_size >> 10,
           free_size,
           free_size >> 10,
           min_free_size,
           min_free_size >> 10);
  }
}
#endif

void PlayMp3(const uint8_t *data, size_t size) {
  auto ret = esp_mp3_dec_register();
  if (ret != ESP_AUDIO_ERR_OK) {
    printf("Failed to register mp3 decoder: %d\n", ret);
    abort();
  }

  esp_audio_simple_dec_handle_t decoder = nullptr;
  esp_audio_simple_dec_cfg_t audio_dec_cfg{
      .dec_type = ESP_AUDIO_SIMPLE_DEC_TYPE_MP3,
      .dec_cfg = nullptr,
      .cfg_size = 0,
  };
  ret = esp_audio_simple_dec_open(&audio_dec_cfg, &decoder);
  if (ret != ESP_AUDIO_ERR_OK) {
    printf("Failed to open mp3 decoder: %d\n", ret);
    abort();
  }
  g_audio_output_device->OpenOutput(16000);

  esp_audio_simple_dec_raw_t raw = {
      .buffer = const_cast<uint8_t *>(data),
      .len = size,
      .eos = true,
      .consumed = 0,
      .frame_recover = ESP_AUDIO_SIMPLE_DEC_RECOVERY_NONE,
  };

  uint8_t *frame_data = (uint8_t *)malloc(4096);
  esp_audio_simple_dec_out_t out_frame = {
      .buffer = frame_data,
      .len = 4096,
      .needed_size = 0,
      .decoded_size = 0,
  };

  while (raw.len > 0) {
    const auto ret = esp_audio_simple_dec_process(decoder, &raw, &out_frame);
    if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
      out_frame.buffer = reinterpret_cast<uint8_t *>(realloc(out_frame.buffer, out_frame.needed_size));
      if (out_frame.buffer == nullptr) {
        break;
      }
      out_frame.len = out_frame.needed_size;
      continue;
    }

    if (ret != ESP_AUDIO_ERR_OK) {
      break;
    }

    g_audio_output_device->Write(reinterpret_cast<int16_t *>(out_frame.buffer), out_frame.decoded_size >> 1);
    raw.len -= raw.consumed;
    raw.buffer += raw.consumed;
  }

  free(frame_data);

  g_audio_output_device->CloseOutput();
  esp_audio_simple_dec_close(decoder);
  esp_audio_dec_unregister(ESP_AUDIO_TYPE_MP3);
}

void ConfigureWifi() {
  printf("configure wifi\n");
  auto wifi_configurator = std::make_unique<WifiConfigurator>(WiFi, kSmartConfigType);

  ESP_ERROR_CHECK(iot_button_register_cb(
      g_button_boot_handle,
      BUTTON_PRESS_DOWN,
      nullptr,
      [](void *, void *data) {
        printf("boot button pressed\n");
        static_cast<WifiConfigurator *>(data)->StartSmartConfig();
      },
      wifi_configurator.get()));

  g_display->ShowStatus("网络配置中");
  PlayMp3(kNotification0mp3, sizeof(kNotification0mp3));

#if defined(WIFI_SSID) && defined(WIFI_PASSWORD)
  printf("wifi config start with wifi: %s, %s\n", WIFI_SSID, WIFI_PASSWORD);
  wifi_configurator->Start(WIFI_SSID, WIFI_PASSWORD);
#else
  printf("wifi config start\n");
  wifi_configurator->Start();
#endif

  while (true) {
    const auto state = wifi_configurator->WaitStateChanged();
    if (state == WifiConfigurator::State::kConnecting) {
      printf("wifi connecting\n");
      g_display->ShowStatus("网络连接中");
    } else if (state == WifiConfigurator::State::kSmartConfiguring) {
      printf("wifi smart configuring\n");
      g_display->ShowStatus("配网模式");
      PlayMp3(kNetworkConfigModeMp3, sizeof(kNetworkConfigModeMp3));
    } else if (state == WifiConfigurator::State::kFinished) {
      break;
    }
  }

  iot_button_unregister_cb(g_button_boot_handle, BUTTON_PRESS_DOWN, nullptr);

  printf("wifi connected\n");
  printf("- mac address: %s\n", WiFi.macAddress().c_str());
  printf("- bssid:       %s\n", WiFi.BSSIDstr().c_str());
  printf("- ssid:        %s\n", WiFi.SSID().c_str());
  printf("- ip:          %s\n", WiFi.localIP().toString().c_str());
  printf("- gateway:     %s\n", WiFi.gatewayIP().toString().c_str());
  printf("- subnet mask: %s\n", WiFi.subnetMask().toString().c_str());

  g_display->ShowStatus("网络已连接");
  PlayMp3(kNetworkConnectedMp3, sizeof(kNetworkConnectedMp3));
}

void InitMcpTools() {
  auto &engine = ai_vox::Engine::GetInstance();
  engine.AddMcpTool("self.audio_speaker.set_volume",         // tool name
                    "Set the volume of the audio speaker.",  // tool description
                    {
                        {
                            "volume",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 0,
                                .max = 100,
                            },
                        },
                        // add more parameter schema as needed
                    }  // parameter schema
  );

  engine.AddMcpTool("self.audio_speaker.get_volume",         // tool name
                    "Get the volume of the audio speaker.",  // tool description
                    {
                        // empty
                    }  // parameter schema
  );

  engine.AddMcpTool("self.servo.set_one_servo",            // tool name
                    "Set the angle of a specific servo.",  // tool description
                    {
                        {
                            "index",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 1,
                                .max = static_cast<int64_t>(kServoPins.size()),
                            },
                        },
                        {
                            "angle",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 0,
                                .max = static_cast<int64_t>(kMaxServoAngle),
                            },
                        },
                        // add more parameter schema as needed
                    }  // parameter schema
  );

  engine.AddMcpTool("self.servo.set_range_servos",                    // tool name
                    "Set the angle of the continuous range servos.",  // tool description
                    {
                        {
                            "start_index",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 1,
                                .max = static_cast<int64_t>(kServoPins.size()),
                            },
                        },
                        {
                            "end_index",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 1,
                                .max = static_cast<int64_t>(kServoPins.size()),
                            },
                        },
                        {
                            "angle",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 0,
                                .max = static_cast<int64_t>(kMaxServoAngle),
                            },
                        },
                        // add more parameter schema as needed
                    }  // parameter schema
  );

  engine.AddMcpTool("self.servo.get_index_servo_angle",            // tool name
                    "Get the current angle of a specific servo.",  // tool description
                    {
                        {
                            "index",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 1,
                                .max = static_cast<int64_t>(kServoPins.size()),
                            },
                        },
                        // add more parameter schema as needed
                    }  // parameter schema
  );

  engine.AddMcpTool("self.servo.get_range_servo_angles",                   // tool name
                    "Get the current angles of continuous range servos.",  // tool description
                    {
                        {
                            "start_index",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 1,
                                .max = static_cast<int64_t>(kServoPins.size()),
                            },
                        },
                        {
                            "end_index",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 1,
                                .max = static_cast<int64_t>(kServoPins.size()),
                            },
                        },
                    }  // parameter schema
  );

  engine.AddMcpTool("self.display.set_brightness",         // tool name
                    "Set the brightness of the display.",  // tool description
                    {
                        {
                            "brightness",
                            ai_vox::ParamSchema<int64_t>{
                                .default_value = std::nullopt,
                                .min = 0,
                                .max = 255,
                            },
                        },
                        // add more parameter schema as needed
                    }  // parameter schema
  );

  engine.AddMcpTool("self.display.get_brightness",                 // tool name
                    "Get the current brightness of the display.",  // tool description
                    {
                        // empty
                    }  // parameter schema
  );
}

uint32_t CalculateDuty(const uint16_t angle) {
  // Map the angle to the pulse width（500-2500μs）
  const uint32_t pulse_width = map(angle, 0, kMaxServoAngle, kMinPulse, kMaxPulse);
  return static_cast<uint32_t>((pulse_width * kMaxPwmDuty) / kPwmFactor);
}

void InitServos() {
  printf("init servos\n");

  for (uint8_t i = 0; i < kServoPins.size(); i++) {
    if (!ledcAttach(kServoPins[i], kServoFrequency, kServoResolution)) {
      printf("Error: Failed to attach servo %" PRIu8 " on pin %" PRIu8 " .\n", i, kServoPins[i]);
      continue;
    }

    if (!ledcWrite(kServoPins[i], CalculateDuty(g_servo_angles[i]))) {
      printf("Error: Failed to set initial duty for servo %" PRIu8 " .\n", i);
      continue;
    }
  }
}

bool SetServoAngle(const uint8_t servo_index, const uint16_t angle) {
  if (servo_index < 1 || servo_index > kServoPins.size()) {
    printf("Error: Invalid servo index: %" PRIu8 ", valid range: 1-%" PRIu8 " .\n", servo_index, kServoPins.size());
    return false;
  }

  if (angle > kMaxServoAngle) {
    printf("Error: Invalid servo angle: %" PRIu16 " for index: %" PRIu8 " .\n", angle, servo_index);
    return false;
  }

  if (!ledcWrite(kServoPins[servo_index - 1], CalculateDuty(angle))) {
    printf("Error: Failed to write duty for servo %" PRIu8 " .\n", servo_index);
    return false;
  }
  g_servo_angles[servo_index - 1] = angle;

  return true;
}

}  // namespace

void setup() {
  Serial.begin(115200);

  printf("init button\n");
  const button_config_t btn_cfg = {
      .long_press_time = 1000,
      .short_press_time = 50,
  };

  const button_gpio_config_t gpio_cfg = {
      .gpio_num = kButtonBoot,
      .active_level = 0,
      .enable_power_save = false,
      .disable_pull = false,
  };

  ESP_ERROR_CHECK(iot_button_new_gpio_device(&btn_cfg, &gpio_cfg, &g_button_boot_handle));

  InitDisplay();

  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0) {
    g_display->SetChatMessage(Display::Role::kSystem, "No SPIRAM available, please check your board.");
    while (true) {
      printf("No SPIRAM available, please check your board.\n");
      delay(1000);
    }
  }

  g_display->ShowStatus("初始化");
  ConfigureWifi();
  InitMcpTools();

  InitServos();

  auto audio_input_device = std::make_shared<AudioInputDeviceSph0645>(kMicPinSck, kMicPinWs, kMicPinSd);
  auto &ai_vox_engine = ai_vox::Engine::GetInstance();
  ai_vox_engine.SetObserver(g_observer);
  ai_vox_engine.SetOtaUrl("https://api.tenclass.net/xiaozhi/ota/");
  ai_vox_engine.ConfigWebsocket("wss://api.tenclass.net/xiaozhi/v1/",
                                {
                                    {"Authorization", "Bearer test-token"},
                                });
  printf("engine starting\n");
  g_display->ShowStatus("AI引擎启动中");

  ai_vox_engine.Start(audio_input_device, g_audio_output_device);

  printf("engine started\n");

  ESP_ERROR_CHECK(iot_button_register_cb(
      g_button_boot_handle,
      BUTTON_PRESS_DOWN,
      nullptr,
      [](void *button_handle, void *usr_data) {
        printf("boot button pressed\n");
        ai_vox::Engine::GetInstance().Advance();
      },
      nullptr));

  g_display->ShowStatus("AI引擎已启动");
}

void loop() {
#ifdef PRINT_HEAP_INFO_INTERVAL
  static uint32_t s_print_heap_info_time = 0;
  if (s_print_heap_info_time == 0 || millis() - s_print_heap_info_time >= PRINT_HEAP_INFO_INTERVAL) {
    s_print_heap_info_time = millis();
    PrintMemInfo();
  }
#endif

  auto &engine = ai_vox::Engine::GetInstance();

  const auto events = g_observer->PopEvents();

  for (auto &event : events) {
    if (auto text_received_event = std::get_if<ai_vox::TextReceivedEvent>(&event)) {
      printf("on text received: %s\n", text_received_event->content.c_str());
    } else if (auto activation_event = std::get_if<ai_vox::ActivationEvent>(&event)) {
      printf("activation code: %s, message: %s\n", activation_event->code.c_str(), activation_event->message.c_str());
      g_display->ShowStatus("激活设备");
      g_display->SetChatMessage(Display::Role::kSystem, activation_event->message);
    } else if (auto state_changed_event = std::get_if<ai_vox::StateChangedEvent>(&event)) {
      switch (state_changed_event->new_state) {
        case ai_vox::ChatState::kIdle: {
          printf("Idle\n");
          break;
        }
        case ai_vox::ChatState::kInitted: {
          printf("Initted\n");
          g_display->ShowStatus("初始化完成");
          break;
        }
        case ai_vox::ChatState::kLoading: {
          printf("Loading...\n");
          g_display->ShowStatus("加载协议中");
          break;
        }
        case ai_vox::ChatState::kLoadingFailed: {
          printf("Loading failed, please retry\n");
          g_display->ShowStatus("加载协议失败，请重试");
          break;
        }
        case ai_vox::ChatState::kStandby: {
          printf("Standby\n");
          g_display->ShowStatus("待命");
          break;
        }
        case ai_vox::ChatState::kConnecting: {
          printf("Connecting...\n");
          g_display->ShowStatus("连接中...");
          break;
        }
        case ai_vox::ChatState::kListening: {
          printf("Listening...\n");
          g_display->ShowStatus("聆听中");
          break;
        }
        case ai_vox::ChatState::kSpeaking: {
          printf("Speaking...\n");
          g_display->ShowStatus("说话中");
          break;
        }
        default: {
          break;
        }
      }
    } else if (auto emotion_event = std::get_if<ai_vox::EmotionEvent>(&event)) {
      printf("emotion: %s\n", emotion_event->emotion.c_str());
      g_display->SetEmotion(emotion_event->emotion);
    } else if (auto chat_message_event = std::get_if<ai_vox::ChatMessageEvent>(&event)) {
      switch (chat_message_event->role) {
        case ai_vox::ChatRole::kAssistant: {
          printf("role: assistant, content: %s\n", chat_message_event->content.c_str());
          g_display->SetChatMessage(Display::Role::kAssistant, chat_message_event->content);
          break;
        }
        case ai_vox::ChatRole::kUser: {
          printf("role: user, content: %s\n", chat_message_event->content.c_str());
          g_display->SetChatMessage(Display::Role::kUser, chat_message_event->content);
          break;
        }
      }
    } else if (auto mcp_tool_call_event = std::get_if<ai_vox::McpToolCallEvent>(&event)) {
      printf("on mcp tool call: %s\n", mcp_tool_call_event->ToString().c_str());

      if ("self.audio_speaker.set_volume" == mcp_tool_call_event->name) {
        const auto volume_ptr = mcp_tool_call_event->param<int64_t>("volume");
        if (volume_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: volume");
          continue;
        }
        if (*volume_ptr < 0 || *volume_ptr > 100) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Invalid volume value, volume must be between 0 and 100");
          continue;
        }

        printf("on mcp tool call: self.audio_speaker.set_volume, volume: %" PRId64 "\n", *volume_ptr);
        g_audio_output_device->set_volume(static_cast<uint16_t>(*volume_ptr));
        engine.SendMcpCallResponse(mcp_tool_call_event->id, true);

      } else if ("self.audio_speaker.get_volume" == mcp_tool_call_event->name) {
        const auto volume = g_audio_output_device->volume();
        printf("on mcp tool call: self.audio_speaker.get_volume, volume: %" PRIu16 "\n", volume);
        engine.SendMcpCallResponse(mcp_tool_call_event->id, static_cast<int64_t>(volume));

      } else if ("self.servo.set_one_servo" == mcp_tool_call_event->name) {
        const auto index_ptr = mcp_tool_call_event->param<int64_t>("index");
        if (index_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: index");
          continue;
        }
        if (*index_ptr < 1 || *index_ptr > kServoPins.size()) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Invalid index value, index must be between 1 and " + std::to_string(kServoPins.size()));
          continue;
        }

        const auto angle_ptr = mcp_tool_call_event->param<int64_t>("angle");
        if (angle_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: angle");
          continue;
        }
        if (*angle_ptr < 0 || *angle_ptr > kMaxServoAngle) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Invalid angle value, angle must be between: 0 and " + std::to_string(kMaxServoAngle));
          continue;
        }

        printf("on mcp tool call: self.servo.set_one_servo, index: %" PRId64 ", angle: %" PRId64 "\n", *index_ptr, *angle_ptr);
        if (SetServoAngle(static_cast<uint8_t>(*index_ptr), static_cast<uint16_t>(*angle_ptr))) {
          engine.SendMcpCallResponse(mcp_tool_call_event->id, true);
        } else {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Failed to set servo angle for index: " + std::to_string(*index_ptr));
        }

      } else if ("self.servo.set_range_servos" == mcp_tool_call_event->name) {
        const auto start_index_ptr = mcp_tool_call_event->param<int64_t>("start_index");
        if (start_index_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: start_index");
          continue;
        }
        if (*start_index_ptr < 1 || *start_index_ptr > kServoPins.size()) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "Invalid start_index value, start_index must be between 1 and " + std::to_string(kServoPins.size()));
          continue;
        }

        const auto end_index_ptr = mcp_tool_call_event->param<int64_t>("end_index");
        if (end_index_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: end_index");
          continue;
        }
        if (*end_index_ptr < 1 || *end_index_ptr > kServoPins.size()) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "Invalid end_index value, end_index must be between 1 and " + std::to_string(kServoPins.size()));
          continue;
        }

        if (*start_index_ptr > *end_index_ptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Invalid arguments: start_index must be less than or equal to end_index");
          continue;
        }

        const auto angle_ptr = mcp_tool_call_event->param<int64_t>("angle");
        if (angle_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: angle");
          continue;
        }
        if (*angle_ptr < 0 || *angle_ptr > kMaxServoAngle) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Invalid angle value, angle must be between 0 and " + std::to_string(kMaxServoAngle));
          continue;
        }

        printf("on mcp tool call: self.servo.set_range_servos, start_index: %" PRId64 ", end_index: %" PRId64 ", angle: %" PRId64 "\n",
               *start_index_ptr,
               *end_index_ptr,
               *angle_ptr);

        uint8_t index = 0;
        for (index = *start_index_ptr; index <= *end_index_ptr; index++) {
          if (!SetServoAngle(index, static_cast<uint16_t>(*angle_ptr))) {
            engine.SendMcpCallError(mcp_tool_call_event->id, "Failed to set servo angle for index: " + std::to_string(index));
            break;
          }
        }
        if (index > *end_index_ptr) {
          engine.SendMcpCallResponse(mcp_tool_call_event->id, true);
        }

      } else if ("self.servo.get_index_servo_angle" == mcp_tool_call_event->name) {
        const auto index_ptr = mcp_tool_call_event->param<int64_t>("index");
        if (index_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: index");
          continue;
        }
        if (*index_ptr < 1 || *index_ptr > kServoPins.size()) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Invalid index value, index must be between 1 and " + std::to_string(kServoPins.size()));
          continue;
        }

        printf(
            "on mcp tool call: self.servo.get_index_servo_angle, index: %" PRId64 " angle: %" PRIu8 "\n", *index_ptr, g_servo_angles[*index_ptr - 1]);
        engine.SendMcpCallResponse(mcp_tool_call_event->id, static_cast<int64_t>(g_servo_angles[*index_ptr - 1]));

      } else if ("self.servo.get_range_servo_angles" == mcp_tool_call_event->name) {
        const auto start_index_ptr = mcp_tool_call_event->param<int64_t>("start_index");
        if (start_index_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: start_index");
          continue;
        }
        if (*start_index_ptr < 1 || *start_index_ptr > kServoPins.size()) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "Invalid start_index value, start_index must be between 1 and " + std::to_string(kServoPins.size()));
          continue;
        }

        const auto end_index_ptr = mcp_tool_call_event->param<int64_t>("end_index");
        if (end_index_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: end_index");
          continue;
        }
        if (*end_index_ptr < 1 || *end_index_ptr > kServoPins.size()) {
          engine.SendMcpCallError(mcp_tool_call_event->id,
                                  "Invalid end_index value, end_index must be between 1 and " + std::to_string(kServoPins.size()));
          continue;
        }

        if (*start_index_ptr > *end_index_ptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Invalid arguments: start_index must be less than or equal to end_index");
          continue;
        }

        auto servo_angles = cjson_util::ArrayMakeUnique();
        for (int64_t i = *start_index_ptr; i <= *end_index_ptr; i++) {
          auto servo_angle = cjson_util::MakeUnique();

          cJSON_AddNumberToObject(servo_angle.get(), "index", i);
          cJSON_AddNumberToObject(servo_angle.get(), "angle", g_servo_angles[i - 1]);
          cJSON_AddItemToArray(servo_angles.get(), servo_angle.release());
        }

        const std::string servo_angles_json = cjson_util::ToString(servo_angles, false);
        printf("on mcp tool call: self.servo.get_range_servo_angles, angles: %s\n", servo_angles_json.c_str());
        engine.SendMcpCallResponse(mcp_tool_call_event->id, std::move(servo_angles_json));

      } else if ("self.display.set_brightness" == mcp_tool_call_event->name) {
        const auto brightness_ptr = mcp_tool_call_event->param<int64_t>("brightness");
        if (brightness_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing valid argument: brightness");
          continue;
        }
        if (*brightness_ptr < 0 || *brightness_ptr > 255) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Invalid brightness value, must be between 0 and 255");
          continue;
        }

        printf("on mcp tool call: self.display.set_brightness, brightness: %" PRId64 "\n", *brightness_ptr);
        if (ledcWriteChannel(kDisplayBacklightChannel, *brightness_ptr)) {
          g_display_brightness = static_cast<uint8_t>(*brightness_ptr);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, true);
        } else {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Failed to set display brightness");
        }

      } else if ("self.display.get_brightness" == mcp_tool_call_event->name) {
        printf("on mcp tool call: self.display.get_brightness, brightness: %" PRIu8 "\n", g_display_brightness);
        engine.SendMcpCallResponse(mcp_tool_call_event->id, static_cast<int64_t>(g_display_brightness));
      }
    }
  }
}
