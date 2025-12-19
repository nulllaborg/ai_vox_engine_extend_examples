#include <Arduino.h>
#include <DHT.h>
#include <WiFi.h>
#include <driver/i2c_master.h>
#include <driver/spi_common.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

#include "ai_vox_engine.h"
#include "audio_device/audio_device_es8311.h"
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
#include "servo.h"

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
 *  SC_TYPE_ESPTOUCH            protocol: ESPTouch，支持ESPTouch APP配网
 *  SC_TYPE_AIRKISS,            protocol: AirKiss，支持小程序配网
 *  SC_TYPE_ESPTOUCH_AIRKISS,   protocol: ESPTouch and AirKiss ，支持ESPTouch APP配网，支持小程序配网
 *  SC_TYPE_ESPTOUCH_V2,        protocol: ESPTouch v2, 支持ESPTouch APP配网
 */
constexpr smartconfig_type_t kSmartConfigType = SC_TYPE_ESPTOUCH_AIRKISS;  // ESPTouch and AirKiss

constexpr gpio_num_t kButtonBoot = GPIO_NUM_0;  // Button

constexpr gpio_num_t kLcdBacklightPin = GPIO_NUM_16;

// https://docs.espressif.com/projects/esp-iot-solution/zh_CN/latest/display/lcd/spi_lcd.html#line
constexpr gpio_num_t kSt7789Sda = GPIO_NUM_21;  // SPI MOSI
constexpr gpio_num_t kSt7789Scl = GPIO_NUM_17;  // SPI SCLK
constexpr gpio_num_t kSt7789Csx = GPIO_NUM_15;  // SPI CS
constexpr gpio_num_t kSt7789Dcx = GPIO_NUM_14;  // SPI DC

constexpr gpio_num_t kEs8311Mclk = GPIO_NUM_11;   // ES8311 Master clock (MCLK) -- I2S MCLK/MCK
constexpr gpio_num_t kEs8311Sclk = GPIO_NUM_10;   // ES8311 Serial data bit clock/DMIC bit clock (SCLK/DMIC_SCL) -- I2S SCLK/BCK
constexpr gpio_num_t kEs8311Lrck = GPIO_NUM_8;    // ES8311 Serial data left and right channel frame clock (LRCK) --  I2S WS
constexpr gpio_num_t kEs8311Dsdin = GPIO_NUM_7;   // ES8311 DAC serial data input (DSDIN) -- I2S DOUT
constexpr gpio_num_t kEs8311Asdout = GPIO_NUM_9;  // ES8311 ADC serial data output (ASDOUT) -- I2S DIN

constexpr gpio_num_t kI2cScl = GPIO_NUM_12;  // ES8311 CCLK
constexpr gpio_num_t kI2cSda = GPIO_NUM_13;  // ES8311 CDATA

const std::vector<gpio_num_t> kServoPins = {GPIO_NUM_42, GPIO_NUM_43};  // Servo control pins

// Define DHT11 sensor pins
constexpr gpio_num_t kDht11Pin = GPIO_NUM_3;

constexpr gpio_num_t kLedPin = GPIO_NUM_4;  // led Pin

constexpr i2c_port_t kI2CPort = I2C_NUM_1;

constexpr auto kDisplaySpiMode = 0;
constexpr uint32_t kDisplayWidth = 240;
constexpr uint32_t kDisplayHeight = 240;
constexpr bool kDisplayMirrorX = false;
constexpr bool kDisplayMirrorY = false;
constexpr bool kDisplayInvertColor = true;
constexpr bool kDisplaySwapXY = false;
constexpr auto kDisplayRgbElementOrder = LCD_RGB_ELEMENT_ORDER_RGB;

constexpr uint8_t kEs8311I2cAddress = 0x30;
constexpr uint32_t kAudioSampleRate = 16000;

i2c_master_bus_handle_t g_i2c_master_bus_handle = nullptr;
std::shared_ptr<ai_vox::AudioDeviceEs8311> g_audio_device_es8311;
std::unique_ptr<Display> g_display;
auto g_observer = std::make_shared<ai_vox::Observer>();
button_handle_t g_button_boot_handle = nullptr;

constexpr uint32_t kMinPulse = 500;
constexpr uint32_t kMaxPulse = 2500;
constexpr uint16_t kMaxServoAngle = 180;

constexpr uint8_t kLcdBacklightChannel = 7;
constexpr uint32_t kDisplayBacklightFrequency = 1000;
constexpr uint8_t kDisplayBacklightResolution = 8;

std::vector<uint16_t> g_servo_angles(kServoPins.size(), 90);

std::vector<std::unique_ptr<em::Servo>> g_servos;

uint8_t g_display_brightness = 255;

DHT g_dht11(kDht11Pin, DHT11);

void InitI2cBus() {
  const i2c_master_bus_config_t i2c_master_bus_config = {
      .i2c_port = kI2CPort,
      .sda_io_num = kI2cSda,
      .scl_io_num = kI2cScl,
      .clk_source = I2C_CLK_SRC_DEFAULT,
      .glitch_ignore_cnt = 7,
      .intr_priority = 0,
      .trans_queue_depth = 0,
      .flags =
          {
              .enable_internal_pullup = 1,
              .allow_pd = 0,
          },
  };
  ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_master_bus_config, &g_i2c_master_bus_handle));
  printf("g_i2c_master_bus: %p\n", g_i2c_master_bus_handle);
}

void InitEs8311() {
  g_audio_device_es8311 = std::make_shared<ai_vox::AudioDeviceEs8311>(g_i2c_master_bus_handle,
                                                                      kEs8311I2cAddress,  // ES8311 I2C address
                                                                      kI2CPort,           // I2C port
                                                                      kAudioSampleRate,   // sample rate
                                                                      kEs8311Mclk,        // ES8311 Master clock (MCLK)
                                                                      kEs8311Sclk,    // ES8311 Serial data bit clock/DMIC bit clock (SCLK/DMIC_SCL)
                                                                      kEs8311Lrck,    // ES8311 Serial data left and right channel frame clock (LRCK)
                                                                      kEs8311Asdout,  // ES8311 ADC serial data output (ASDOUT)
                                                                      kEs8311Dsdin    // ES8311 DAC serial data input (DSDIN)
  );
}

void InitDisplay() {
  printf("init display\n");

  if (!ledcAttachChannel(kLcdBacklightPin, kDisplayBacklightFrequency, kDisplayBacklightResolution, kLcdBacklightChannel)) {
    printf("Error: Failed to attach LCD backlight LEDC channel.\n");
    return;
  }

  if (!ledcWriteChannel(kLcdBacklightChannel, g_display_brightness)) {
    printf("Error: Failed to set LCD backlight brightness.\n");
    return;
  }

  //  https://docs.espressif.com/projects/esp-iot-solution/zh_CN/latest/display/lcd/spi_lcd.html#id4
  spi_bus_config_t buscfg{
      .mosi_io_num = kSt7789Sda,
      .miso_io_num = GPIO_NUM_NC,
      .sclk_io_num = kSt7789Scl,
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

  // https://docs.espressif.com/projects/esp-iot-solution/zh_CN/latest/display/lcd/spi_lcd.html#id8
  esp_lcd_panel_io_spi_config_t io_config = {};
  io_config.cs_gpio_num = kSt7789Csx;  // SPI CS
  io_config.dc_gpio_num = kSt7789Dcx;  // SPI DC
  io_config.spi_mode = kDisplaySpiMode;
  io_config.pclk_hz = 40 * 1000 * 1000;
  io_config.trans_queue_depth = 10;
  io_config.lcd_cmd_bits = 8;
  io_config.lcd_param_bits = 8;
  ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi(SPI3_HOST, &io_config, &panel_io));

  // 初始化液晶屏驱动芯片
  ESP_LOGD(TAG, "Install LCD driver");
  esp_lcd_panel_dev_config_t panel_config = {};
  panel_config.reset_gpio_num = -1;
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

void PlayMp3(const uint8_t* data, size_t size) {
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
  g_audio_device_es8311->OpenOutput(16000);

  esp_audio_simple_dec_raw_t raw = {
      .buffer = const_cast<uint8_t*>(data),
      .len = size,
      .eos = true,
      .consumed = 0,
      .frame_recover = ESP_AUDIO_SIMPLE_DEC_RECOVERY_NONE,
  };

  uint8_t* frame_data = (uint8_t*)malloc(4096);
  esp_audio_simple_dec_out_t out_frame = {
      .buffer = frame_data,
      .len = 4096,
      .needed_size = 0,
      .decoded_size = 0,
  };

  while (raw.len > 0) {
    const auto ret = esp_audio_simple_dec_process(decoder, &raw, &out_frame);
    if (ret == ESP_AUDIO_ERR_BUFF_NOT_ENOUGH) {
      // Handle output buffer not enough case
      out_frame.buffer = reinterpret_cast<uint8_t*>(realloc(out_frame.buffer, out_frame.needed_size));
      if (out_frame.buffer == nullptr) {
        break;
      }
      out_frame.len = out_frame.needed_size;
      continue;
    }

    if (ret != ESP_AUDIO_ERR_OK) {
      break;
    }

    g_audio_device_es8311->Write(reinterpret_cast<int16_t*>(out_frame.buffer), out_frame.decoded_size >> 1);
    raw.len -= raw.consumed;
    raw.buffer += raw.consumed;
  }

  free(frame_data);

  g_audio_device_es8311->CloseOutput();
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
      [](void*, void* data) {
        printf("boot button pressed\n");
        static_cast<WifiConfigurator*>(data)->StartSmartConfig();
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
  auto& engine = ai_vox::Engine::GetInstance();
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

  engine.AddMcpTool(
      "self.dht11_sensor.get_temperature_and_humidity",                                                                // tool name
      "Get both temperature and humidity data from DHT11 sensor. Temperature in Celsius and humidity in percentage.",  // tool description
      {
          // empty
      }  // parameter schema
  );

  engine.AddMcpTool("self.led.set",                                           // tool name
                    "Set the state of the LED, true for on, false for off.",  // tool description
                    {
                        {
                            "state",
                            ai_vox::ParamSchema<bool>{
                                .default_value = std::nullopt,
                            },
                        },
                        // add more parameter schema as needed
                    }  // parameter schema
  );

  engine.AddMcpTool("self.led.get",                                           // tool name
                    "Get the state of the LED, true for on, false for off.",  // tool description
                    {
                        // empty
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

void InitServos() {
  printf("init servos\n");

  g_servos.clear();
  for (uint8_t i = 0; i < kServoPins.size(); i++) {
    auto servo = std::make_unique<em::Servo>(kServoPins[i], 0, kMaxServoAngle, kMinPulse, kMaxPulse);

    if (!servo->Init()) {
      printf("Error: Failed to init servo %" PRIu8 " on pin %" PRIu16 "\n", i, kServoPins[i]);
      continue;
    }

    servo->Write(g_servo_angles[i]);

    g_servos.push_back(std::move(servo));
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);

  InitI2cBus();
  InitDisplay();
  InitEs8311();

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

  g_dht11.begin();

  pinMode(kLedPin, OUTPUT);
  digitalWrite(kLedPin, LOW);

  auto& ai_vox_engine = ai_vox::Engine::GetInstance();
  ai_vox_engine.SetObserver(g_observer);
  ai_vox_engine.SetOtaUrl("https://api.tenclass.net/xiaozhi/ota/");
  ai_vox_engine.ConfigWebsocket("wss://api.tenclass.net/xiaozhi/v1/",
                                {
                                    {"Authorization", "Bearer test-token"},
                                });
  printf("engine starting\n");
  g_display->ShowStatus("AI引擎启动中");

  ai_vox_engine.Start(g_audio_device_es8311, g_audio_device_es8311);

  printf("engine started\n");

  ESP_ERROR_CHECK(iot_button_register_cb(
      g_button_boot_handle,
      BUTTON_PRESS_DOWN,
      nullptr,
      [](void* button_handle, void* usr_data) {
        printf("boot button pressed\n");
        ai_vox::Engine::GetInstance().Advance();
      },
      nullptr));
}

void loop() {
#ifdef PRINT_HEAP_INFO_INTERVAL
  static uint32_t s_print_heap_info_time = 0;
  if (s_print_heap_info_time == 0 || millis() - s_print_heap_info_time >= PRINT_HEAP_INFO_INTERVAL) {
    s_print_heap_info_time = millis();
    PrintMemInfo();
  }
#endif

  auto& engine = ai_vox::Engine::GetInstance();

  const auto events = g_observer->PopEvents();

  for (auto& event : events) {
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
        g_audio_device_es8311->set_volume(static_cast<uint16_t>(*volume_ptr));
        engine.SendMcpCallResponse(mcp_tool_call_event->id, true);

      } else if ("self.audio_speaker.get_volume" == mcp_tool_call_event->name) {
        const auto volume = g_audio_device_es8311->volume();
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
        g_servos[*index_ptr - 1]->Write(static_cast<uint16_t>(*angle_ptr));
        g_servo_angles[*index_ptr - 1] = *angle_ptr;

        engine.SendMcpCallResponse(mcp_tool_call_event->id, true);

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

        for (uint8_t index = *start_index_ptr - 1; index < *end_index_ptr; index++) {
          g_servos[index]->Write(static_cast<uint16_t>(*angle_ptr));
          g_servo_angles[index - 1] = *angle_ptr;
        }
        engine.SendMcpCallResponse(mcp_tool_call_event->id, true);

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

      } else if ("self.dht11_sensor.get_temperature_and_humidity" == mcp_tool_call_event->name) {
        const float temperature = g_dht11.readTemperature();
        const float humidity = g_dht11.readHumidity();

        printf("on mcp tool call: self.dht11_sensor.get_temperature_and_humidity, temperature: %.1f C, humidity: %.1f%%\n", temperature, humidity);

        const std::string response = "{\"temperature\": " + std::to_string(temperature) + ", \"humidity\": " + std::to_string(humidity) + "}";
        engine.SendMcpCallResponse(mcp_tool_call_event->id, std::move(response));

      } else if ("self.led.set" == mcp_tool_call_event->name) {
        const auto state_ptr = mcp_tool_call_event->param<bool>("state");
        if (state_ptr == nullptr) {
          engine.SendMcpCallError(mcp_tool_call_event->id, "Missing required argument: state");
          continue;
        }

        digitalWrite(kLedPin, *state_ptr ? HIGH : LOW);
        printf("on mcp tool call: self.led.set, state: %s\n", *state_ptr ? "true" : "false");
        engine.SendMcpCallResponse(mcp_tool_call_event->id, true);

      } else if ("self.led.get" == mcp_tool_call_event->name) {
        printf("on mcp tool call: self.led.get, state: %s\n", digitalRead(kLedPin) == HIGH ? "HIGH" : "LOW");
        engine.SendMcpCallResponse(mcp_tool_call_event->id, digitalRead(kLedPin) == HIGH);

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

        if (ledcWriteChannel(kLcdBacklightChannel, *brightness_ptr)) {
          g_display_brightness = static_cast<uint8_t>(*brightness_ptr);
          engine.SendMcpCallResponse(mcp_tool_call_event->id, true);
        } else {
          engine.SendMcpCallResponse(mcp_tool_call_event->id, "Failed to set LCD backlight brightness");
        }

      } else if ("self.display.get_brightness" == mcp_tool_call_event->name) {
        printf("on mcp tool call: self.display.get_brightness, brightness: %" PRIu8 "\n", g_display_brightness);
        engine.SendMcpCallResponse(mcp_tool_call_event->id, static_cast<int64_t>(g_display_brightness));
      }
    }
  }
}
