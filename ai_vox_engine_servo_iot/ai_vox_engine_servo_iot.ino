#include <WiFi.h>
#include <driver/ledc.h>
#include <driver/spi_common.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#include "ai_vox_engine.h"
#include "ai_vox_observer.h"
#include "audio_input_device_sph0645.h"
#include "display.h"
#include "i2s_std_audio_output_device.h"

#ifndef ARDUINO_ESP32S3_DEV
#error "This example only supports ESP32S3-Dev board."
#endif

#ifndef CONFIG_SPIRAM_MODE_OCT
#error "This example requires PSRAM to OPI PSRAM. Please enable it in Arduino IDE."
#endif

#ifndef WIFI_SSID
#define WIFI_SSID "ssid"
#endif

#ifndef WIFI_PASSWORD
#define WIFI_PASSWORD "password"
#endif

namespace {
// Servo pin array
constexpr gpio_num_t kServoPins[] = {GPIO_NUM_46, GPIO_NUM_47};

constexpr gpio_num_t kMicPinBclk = GPIO_NUM_5;
constexpr gpio_num_t kMicPinWs = GPIO_NUM_2;
constexpr gpio_num_t kMicPinDin = GPIO_NUM_4;

constexpr gpio_num_t kSpeakerPinBclk = GPIO_NUM_13;
constexpr gpio_num_t kSpeakerPinWs = GPIO_NUM_14;
constexpr gpio_num_t kSpeakerPinDout = GPIO_NUM_1;

constexpr gpio_num_t kTriggerPin = GPIO_NUM_0;

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

constexpr uint32_t kServoFrequency = 50;
constexpr uint32_t kServoResolution = 12;
constexpr uint32_t kMinPulse = 500;
constexpr uint32_t kMaxPulse = 2500;

std::shared_ptr<ai_vox::iot::Entity> g_servo_iot_entity;
auto g_audio_output_device = std::make_shared<ai_vox::I2sStdAudioOutputDevice>(kSpeakerPinBclk, kSpeakerPinWs, kSpeakerPinDout);

std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();

void InitDisplay() {
  pinMode(kDisplayBacklightPin, OUTPUT);
  digitalWrite(kDisplayBacklightPin, HIGH);

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
  // 液晶屏控制IO初始化
  ESP_LOGD(TAG, "Install panel IO");
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.cs_gpio_num = kDisplayCsPin;
  io_config.dc_gpio_num = kDisplayDcPin;
  io_config.spi_mode = kDisplaySpiMode;
  io_config.pclk_hz = 40 * 1000 * 1000;
  io_config.trans_queue_depth = 10;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

  // 初始化液晶屏驱动芯片
  ESP_LOGD(TAG, "Install LCD driver");
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

void InitIot() {
  auto& ai_vox_engine = ai_vox::Engine::GetInstance();

  // g_servo_iot_controller
  // 1. Define the properties of the g_servo_iot_controller motor entity
  std::vector<ai_vox::iot::Property> servo_iot_properties;
  for (uint32_t i = 1; i <= sizeof(kServoPins) / sizeof(kServoPins[0]); i++) {
    std::string property_name = std::to_string(i) + "号舵机";
    std::string property_describe = std::to_string(i) + "号舵机的当前角度(0-180之间的整数)";
    servo_iot_properties.push_back({
        std::move(property_name),        // property name
        std::move(property_describe),    // property description
        ai_vox::iot::ValueType::kNumber  // property type
    });
  }
  // 2.Define the functions for the g_servo_iot_controller motor entity
  std::vector<ai_vox::iot::Function> servo_iot_functions({
      {"SetOneServo",
       "设置单个舵机角度",
       {{"angle_value", "舵机角度(0-180之间的整数)", ai_vox::iot::ValueType::kNumber, true},
        {"index", "舵机编号", ai_vox::iot::ValueType::kNumber, true}}},
      {"SetAllServos", "设置所有舵机角度", {{"angle_value", "舵机角度(0-180之间的整数)", ai_vox::iot::ValueType::kNumber, true}}}

      // add more functions as needed
  });
  // 3.Create the g_servo_iot_controller motor entity
  g_servo_iot_entity = std::make_shared<ai_vox::iot::Entity>("Servo",                          // name
                                                             "舵机",                           // description
                                                             std::move(servo_iot_properties),  // properties
                                                             std::move(servo_iot_functions)    // functions
  );

  // 4.Initialize the g_servo_iot_controller motor entity with default values
  for (uint32_t i = 1; i <= sizeof(kServoPins) / sizeof(kServoPins[0]); i++) {
    std::string property_name = std::to_string(i) + "号舵机";
    g_servo_iot_entity->UpdateState(std::move(property_name), 90);
  }

  // 5.Register the g_servo_iot_controller motor entity with the AI Vox engine
  ai_vox_engine.RegisterIotEntity(g_servo_iot_entity);
}

uint32_t CalculateDuty(const int64_t angle) {
  // Map the angle to the pulse width（500-2500μs）
  const uint32_t pulse_width = map(angle, 0, 180, kMinPulse, kMaxPulse);
  return (pulse_width * (1 << kServoResolution)) / (1000000 / kServoFrequency);
}

void InitServos() {
  const uint32_t duty = CalculateDuty(90);
  for (uint32_t i = 0; i < sizeof(kServoPins) / sizeof(kServoPins[0]); i++) {
    analogWriteResolution(kServoPins[i], kServoResolution);
    analogWriteFrequency(kServoPins[i], kServoFrequency);

    pinMode(kServoPins[i], OUTPUT);
    analogWrite(kServoPins[i], duty);  // Initialize to 90 degree position
  }
}

void SetServoAngle(const int64_t servo_index, const int64_t angle) {
  if (servo_index > sizeof(kServoPins) / sizeof(kServoPins[0])) {
    printf("Error: Invalid servo index: %d\n", servo_index);
    return;
  }
  if (angle < 0 || angle > 180) {
    printf("Error: Invalid servo angle: %lld for index: %lld\n", angle, servo_index);
    return;
  }

  const uint32_t duty = CalculateDuty(angle);
  analogWrite(kServoPins[servo_index], duty);
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
}  // namespace

void setup() {
  Serial.begin(115200);

  InitDisplay();

  if (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) == 0) {
    g_display->SetChatMessage(Display::Role::kSystem, "No SPIRAM available, please check your board.");
    while (true) {
      printf("No SPIRAM available, please check your board.\n");
      delay(1000);
    }
  }

  g_display->ShowStatus("Wifi connecting...");
  WiFi.useStaticBuffers(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    printf("Connecting to WiFi, ssid: %s, password: %s\n", WIFI_SSID, WIFI_PASSWORD);
  }
  printf("WiFi connected, IP address: %s\n", WiFi.localIP().toString().c_str());
  g_display->ShowStatus("Wifi connected");

  InitIot();
  InitServos();

  auto audio_input_device = std::make_shared<AudioInputDeviceSph0645>(kMicPinBclk, kMicPinWs, kMicPinDin);
  auto& ai_vox_engine = ai_vox::Engine::GetInstance();
  ai_vox_engine.SetObserver(g_observer);
  ai_vox_engine.SetTrigger(kTriggerPin);
  ai_vox_engine.SetOtaUrl("https://api.tenclass.net/xiaozhi/ota/");
  ai_vox_engine.ConfigWebsocket("wss://api.tenclass.net/xiaozhi/v1/",
                                {
                                    {"Authorization", "Bearer test-token"},
                                });
  ai_vox_engine.Start(audio_input_device, g_audio_output_device);
  g_display->ShowStatus("AI Vox Engine starting...");
}

void loop() {
#ifdef PRINT_HEAP_INFO_INTERVAL
  static uint32_t s_print_heap_info_time = 0;
  if (s_print_heap_info_time == 0 || millis() - s_print_heap_info_time >= PRINT_HEAP_INFO_INTERVAL) {
    s_print_heap_info_time = millis();
    PrintMemInfo();
  }
#endif

  const auto events = g_observer->PopEvents();
  for (auto& event : events) {
    if (auto activation_event = std::get_if<ai_vox::Observer::ActivationEvent>(&event)) {
      printf("activation code: %s, message: %s\n", activation_event->code.c_str(), activation_event->message.c_str());
      g_display->ShowStatus("激活设备");
      g_display->SetChatMessage(Display::Role::kSystem, activation_event->message);
    } else if (auto state_changed_event = std::get_if<ai_vox::Observer::StateChangedEvent>(&event)) {
      switch (state_changed_event->new_state) {
        case ai_vox::ChatState::kIdle: {
          printf("Idle\n");
          break;
        }
        case ai_vox::ChatState::kIniting: {
          printf("Initing...\n");
          g_display->ShowStatus("初始化");
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
    } else if (auto emotion_event = std::get_if<ai_vox::Observer::EmotionEvent>(&event)) {
      printf("emotion: %s\n", emotion_event->emotion.c_str());
      g_display->SetEmotion(emotion_event->emotion);
    } else if (auto chat_message_event = std::get_if<ai_vox::Observer::ChatMessageEvent>(&event)) {
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
    } else if (auto iot_message_event = std::get_if<ai_vox::Observer::IotMessageEvent>(&event)) {
      printf("IOT message: %s, function: %s\n", iot_message_event->name.c_str(), iot_message_event->function.c_str());
      for (const auto& [key, value] : iot_message_event->parameters) {
        if (std::get_if<bool>(&value)) {
          printf("key: %s, value: %s\n", key.c_str(), std::get<bool>(value) ? "true" : "false");
        } else if (std::get_if<std::string>(&value)) {
          printf("key: %s, value: %s\n", key.c_str(), std::get<std::string>(value).c_str());
        } else if (std::get_if<int64_t>(&value)) {
          printf("key: %s, value: %lld\n", key.c_str(), std::get<int64_t>(value));
        }
      }

      if (iot_message_event->name == "Servo") {
        if (iot_message_event->function == "SetAllServos") {  // Simultaneously set the angle of servo A and servo B
          int64_t angle_value = 0;

          if (const auto it = iot_message_event->parameters.find("angle_value"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              angle_value = std::get<int64_t>(it->second);

              if (angle_value < 0 || angle_value > 180) {
                printf("Error: servo angle is out of range (0-180), got: %lld\n", angle_value);
                continue;
              }
            } else {
              printf("Error: servo angle acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'angle_value' not obtained, please check.\n");
            continue;
          }

          printf("Set all servos to angle: %lld\n", angle_value);
          for (uint32_t i = 1; i <= sizeof(kServoPins) / sizeof(kServoPins[0]); i++) {
            SetServoAngle(i - 1, angle_value);
            std::string property_name = std::to_string(i) + "号舵机";
            g_servo_iot_entity->UpdateState(std::move(property_name), angle_value);
          }
        } else if (iot_message_event->function == "SetOneServo") {  // Set the angle of servo A
          int64_t angle_value = 0;
          int64_t index = 0;

          if (const auto it = iot_message_event->parameters.find("angle_value"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              angle_value = std::get<int64_t>(it->second);

              if (angle_value < 0 || angle_value > 180) {
                printf("Error: servo angle is out of range (0-180), got: %lld\n", angle_value);
                continue;
              }
            } else {
              printf("Error: servo angle acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'angle_value' not obtained, please check.\n");
            continue;
          }
          if (const auto it = iot_message_event->parameters.find("index"); it != iot_message_event->parameters.end()) {
            if (std::get_if<int64_t>(&it->second)) {
              index = std::get<int64_t>(it->second);

              if (index < 0 || index > sizeof(kServoPins) / sizeof(kServoPins[0])) {
                printf("Error: Servo number is out of range (1-%d), got: %lld\n", sizeof(kServoPins) / sizeof(kServoPins[0]), index);
                continue;
              }
            } else {
              printf("Error: Servo number acquisition failed, please check.\n");
              continue;
            }
          } else {
            printf("Error: Parameter 'index' not obtained, please check.\n");
            continue;
          }

          printf("Set servo %lld to angle: %lld\n", index, angle_value);
          SetServoAngle(index - 1, angle_value);
          std::string property_name = std::to_string(index) + "号舵机";
          g_servo_iot_entity->UpdateState(std::move(property_name), angle_value);
        }
      }
    }
  }

  taskYIELD();
}
