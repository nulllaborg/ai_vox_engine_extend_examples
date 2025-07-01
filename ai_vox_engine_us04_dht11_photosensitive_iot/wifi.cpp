#include "wifi.h"

#include <esp_wifi.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include <cstring>

namespace {
enum Status : EventBits_t {
  kStationStarted = 1 << 0,
  kWifiConnected = 1 << 1,
  kGotIp = 1 << 2,
};
}  // namespace

Wifi& Wifi::GetInstance() {
  static std::once_flag s_once_flag;
  static Wifi* s_instance = nullptr;
  std::call_once(s_once_flag, []() { s_instance = new Wifi(); });
  return *s_instance;
}

Wifi::Wifi() : event_group_(xEventGroupCreate()) {
  memset(&ip_info_, sizeof(ip_info_), 0);
  ESP_ERROR_CHECK(esp_netif_init());
  esp_event_loop_create_default();
  ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &WifiEventHandler, this));
  ESP_ERROR_CHECK(esp_event_handler_register(IP_EVENT, ESP_EVENT_ANY_ID, &IpEventHandler, this));
  esp_netif_create_default_wifi_sta();
  wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();

  cfg.static_tx_buf_num = 0;
  cfg.dynamic_tx_buf_num = 32;
  cfg.tx_buf_type = 1;

  cfg.static_rx_buf_num = 4;
  cfg.dynamic_rx_buf_num = 32;

  cfg.cache_tx_buf_num = 4;

  cfg.rx_mgmt_buf_type = 1;
  cfg.rx_mgmt_buf_num = 5;
  cfg.mgmt_sbuf_num = 32;

  ESP_ERROR_CHECK(esp_wifi_init(&cfg));
}

void Wifi::WifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  reinterpret_cast<Wifi*>(arg)->WifiEventHandler(event_base, event_id, event_data);
}

void Wifi::IpEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
  reinterpret_cast<Wifi*>(arg)->IpEventHandler(event_base, event_id, event_data);
}

void Wifi::WifiEventHandler(esp_event_base_t event_base, int32_t event_id, void* event_data) {
  switch (event_id) {
    case WIFI_EVENT_STA_START: {
      printf("WIFI_EVENT_STA_START\n");
      xEventGroupSetBits(event_group_, kStationStarted);
      break;
    }
    case WIFI_EVENT_STA_DISCONNECTED: {
      printf("WIFI_EVENT_STA_DISCONNECTED\n");
      memset(&ip_info_, sizeof(ip_info_), 0);
      xEventGroupClearBits(event_group_, kGotIp);
      xEventGroupClearBits(event_group_, kWifiConnected);
      ESP_ERROR_CHECK(esp_wifi_disconnect());
      ESP_ERROR_CHECK(esp_wifi_connect());
      break;
    }
    case WIFI_EVENT_STA_CONNECTED: {
      printf("WIFI_EVENT_STA_CONNECTED\n");
      xEventGroupSetBits(event_group_, kWifiConnected);
      break;
    }
    default: {
      break;
    }
  }
}

void Wifi::IpEventHandler(esp_event_base_t event_base, int32_t event_id, void* event_data) {
  switch (event_id) {
    case IP_EVENT_STA_GOT_IP: {
      printf("IP_EVENT_STA_GOT_IP\n");
      {
        std::lock_guard<std::mutex> lock(mutex_);
        ip_info_ = reinterpret_cast<ip_event_got_ip_t*>(event_data)->ip_info;
      }
      xEventGroupSetBits(event_group_, kGotIp);
      break;
    }
    default: {
      break;
    }
  }
}

void Wifi::Connect(const std::string& ssid, const std::string& password) {
  if (0 != (xEventGroupGetBits(event_group_) && kStationStarted)) {
    printf("wifi sta already started\n");
    return;
  }

  wifi_config_t wifi_config;
  memset(&wifi_config, 0, sizeof(wifi_config));
  wifi_config.sta.scan_method = WIFI_FAST_SCAN;
  wifi_config.sta.sort_method = WIFI_CONNECT_AP_BY_SIGNAL;
  wifi_config.sta.threshold.rssi = -127;
  wifi_config.sta.pmf_cfg.capable = true;

  strncpy(reinterpret_cast<char*>(wifi_config.sta.ssid), ssid.c_str(), sizeof(wifi_config.sta.ssid) - 1);
  if (!password.empty()) {
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA2_PSK;
    strncpy(reinterpret_cast<char*>(wifi_config.sta.password), password.c_str(), sizeof(wifi_config.sta.password) - 1);
  }

  ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
  ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
  ESP_ERROR_CHECK(esp_wifi_start());
  xEventGroupWaitBits(event_group_, kStationStarted, pdFALSE, pdFALSE, portMAX_DELAY);
  ESP_ERROR_CHECK(esp_wifi_connect());
}

bool Wifi::IsConnected() {
  return (xEventGroupGetBits(event_group_) & kWifiConnected) != 0;
}

bool Wifi::IsGotIp() {
  return (xEventGroupGetBits(event_group_) & kGotIp) != 0;
}