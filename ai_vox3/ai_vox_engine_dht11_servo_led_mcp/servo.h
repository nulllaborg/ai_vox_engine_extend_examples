#pragma once

#ifndef _EM_SERVO_H_
#define _EM_SERVO_H_

/**
 * @file servo.h
 */

#include <stdint.h>

namespace em {

/**
 * @~Chinese
 * @class Servo
 * @brief EspServo类用于控制舵机相关操作。该类仅支持ESP32平台。
 */
/**
 * @~English
 * @class Servo This class is used to control a servo motor and is supported only on the ESP32 platform.
 * @brief The Servo class is used for servo control related operations.
 */
class Servo {
 public:
  /**
   * @~Chinese
   * @brief 构造函数，初始化伺服电机的引脚和角度范围。
   * @param pin 伺服电机连接的GPIO引脚。
   * @param min_angle 最小角度，默认为0度。
   * @param max_angle 最大角度，默认为180度。
   * @param min_pulse_width_us 最小脉冲宽度（微秒），默认为500微秒。
   * @param max_pulse_width_us 最大脉冲宽度（微秒），默认为2500微秒。
   */
  /**
   * @~English
   * @brief Constructor, initializes the servo motor's pin and angle range.
   * @param pin The GPIO pin connected to the servo motor.
   * @param min_angle The minimum angle, default is 0 degrees.
   * @param max_angle The maximum angle, default is 180 degrees.
   * @param min_pulse_width_us The minimum pulse width (microseconds), default is 500 microseconds.
   * @param max_pulse_width_us The maximum pulse width (microseconds), default is 2500 microseconds.
   */
  explicit Servo(const uint8_t pin,
                 const uint16_t min_angle = 0,
                 const uint16_t max_angle = 180,
                 const uint32_t min_pulse_width_us = 500,
                 const uint32_t max_pulse_width_us = 2500);

  /**
   * @~Chinese
   * @brief 初始化。
   * @return 如果初始化成功返回true，否则返回false。
   */
  /**
   * @~English
   * @brief Initializes
   * @return Returns true if initialization is successful, otherwise returns false.
   */
  bool Init();

  /**
   * @~Chinese
   * @brief 将舵机旋转到指定的角度。
   * @param angle 要旋转到的角度。
   */
  /**
   * @~English
   * @brief Rotates the servo motor to the specified angle.
   * @param angle The angle to rotate to.
   */
  void Write(const uint16_t angle);

  /**
   * @~Chinese
   * @brief 以微秒为单位设置舵机的脉冲宽度。
   * @param us 脉冲宽度（微秒）。
   */
  /**
   * @~English
   * @brief Sets the pulse width of the servo in microseconds.
   * @param us Pulse width (microseconds).
   */
  void WriteMicroseconds(const double us);

 private:
  Servo(const Servo&) = delete;
  Servo& operator=(const Servo&) = delete;

  const uint8_t pin_;
  const uint16_t min_angle_;
  const uint16_t max_angle_;
  const uint32_t min_pulse_width_us_;
  const uint32_t max_pulse_width_us_;
};
}  // namespace em

#endif