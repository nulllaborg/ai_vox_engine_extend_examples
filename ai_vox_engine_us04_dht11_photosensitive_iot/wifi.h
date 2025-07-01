
#pragma once

#ifndef _AI_VOX_WIFI_H_
#define _AI_VOX_WIFI_H_

#include <esp_wifi.h>

#include <mutex>
#include <string>

class Wifi {
 public:
  static Wifi& GetInstance();
  void Connect(const std::string& ssid, const std::string& password);
  bool IsConnected();
  bool IsGotIp();
  esp_netif_ip_info_t ip_info() const {
    std::lock_guard<std::mutex> lock(mutex_);
    return ip_info_;
  }

 private:
  Wifi();
  Wifi(const Wifi&) = delete;
  Wifi& operator=(const Wifi&) = delete;

  static void WifiEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);
  static void IpEventHandler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data);

  void WifiEventHandler(esp_event_base_t event_base, int32_t event_id, void* event_data);
  void IpEventHandler(esp_event_base_t event_base, int32_t event_id, void* event_data);

  mutable std::mutex mutex_;
  EventGroupHandle_t event_group_ = nullptr;
  esp_netif_ip_info_t ip_info_;
};

#endif