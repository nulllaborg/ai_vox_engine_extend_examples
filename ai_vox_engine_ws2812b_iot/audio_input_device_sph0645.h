#pragma once

#ifndef _AI_VOX_BOARD_AUDIO_INPUT_DEVICE_SPH0645_H_
#define _AI_VOX_BOARD_AUDIO_INPUT_DEVICE_SPH0645_H_

#include <driver/i2s_std.h>

#include "audio_input_device.h"

class AudioInputDeviceSph0645 : public ai_vox::AudioInputDevice {
 public:
  AudioInputDeviceSph0645(gpio_num_t bclk, gpio_num_t ws, gpio_num_t din)
      : gpio_cfg_({
            .mclk = I2S_GPIO_UNUSED,
            .bclk = bclk,
            .ws = ws,
            .dout = I2S_GPIO_UNUSED,
            .din = din,
            .invert_flags =
                {
                    .mclk_inv = false,
                    .bclk_inv = false,
                    .ws_inv = false,
                },
        }) {
  }

  ~AudioInputDeviceSph0645() {
    Close();
  }

 private:
  bool Open(uint32_t sample_rate) override {
    Close();

    i2s_chan_config_t rx_chan_cfg = {
        .id = I2S_NUM_0,
        .role = I2S_ROLE_MASTER,
        .dma_desc_num = 2,
        .dma_frame_num = 320,
        .auto_clear_after_cb = true,
        .auto_clear_before_cb = false,
        .allow_pd = false,
        .intr_priority = 0,
    };

    ESP_ERROR_CHECK(i2s_new_channel(&rx_chan_cfg, nullptr, &i2s_rx_handle_));

    i2s_std_config_t rx_std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(sample_rate),
        .slot_cfg = slot_cfg_,
        .gpio_cfg = gpio_cfg_,
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_rx_handle_, &rx_std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_rx_handle_));
    return true;
  }

  void Close() override {
    if (i2s_rx_handle_ == nullptr) {
      return;
    }

    i2s_channel_disable(i2s_rx_handle_);
    i2s_del_channel(i2s_rx_handle_);
    i2s_rx_handle_ = nullptr;
  }

  size_t Read(int16_t* buffer, uint32_t samples) override {
    auto raw_32bit_samples = new int32_t[samples];
    size_t bytes_read = 0;
    i2s_channel_read(i2s_rx_handle_, raw_32bit_samples, samples * sizeof(raw_32bit_samples[0]), &bytes_read, 1000);

    for (int i = 0; i < samples; i++) {
      int32_t value = raw_32bit_samples[i] >> 14;
      buffer[i] = (value > INT16_MAX) ? INT16_MAX : (value < -INT16_MAX) ? -INT16_MAX : (int16_t)value;
    }
    delete[] raw_32bit_samples;
    return samples;
  }

  i2s_chan_handle_t i2s_rx_handle_ = nullptr;
  i2s_std_slot_config_t slot_cfg_ = {
      .data_bit_width = I2S_DATA_BIT_WIDTH_32BIT,
      .slot_bit_width = I2S_SLOT_BIT_WIDTH_AUTO,
      .slot_mode = I2S_SLOT_MODE_MONO,
      .slot_mask = I2S_STD_SLOT_LEFT,
      .ws_width = I2S_DATA_BIT_WIDTH_32BIT,
      .ws_pol = false,
      .bit_shift = true,
#if SOC_I2S_HW_VERSION_1
      .msb_right = false,
#else
      .left_align = true,
      .big_endian = false,
      .bit_order_lsb = false,
#endif
  };
  i2s_std_gpio_config_t gpio_cfg_;
};

#endif