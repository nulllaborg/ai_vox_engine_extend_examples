/**
 * @file servo.cpp
 */

#include "servo.h"

#include <Arduino.h>
#include <stdio.h>

#include <cmath>

namespace em {
namespace {
constexpr uint8_t kPwmResolution = 12;
constexpr uint8_t kPwmFrequency = 50;
constexpr uint16_t kMaxAngle = 360;
constexpr int16_t kMaxPwmDuty = pow(2, kPwmResolution) - 1;
constexpr double kPwmFactor = 1 / static_cast<double>(kPwmFrequency) * 1000 * 1000;

double Map(const double x, const double min_in, const double max_in, const double min_out, const double max_out) {
  if (x <= min_in) {
    return min_out;
  } else if (x >= max_in) {
    return max_out;
  }

  return (x - min_in) / (max_in - min_in) * (max_out - min_out) + min_out;
}

}  // namespace
Servo::Servo(
    const uint8_t pin, const uint16_t min_angle, const uint16_t max_angle, const uint32_t min_pulse_width_us, const uint32_t max_pulse_width_us)
    : pin_(pin),
      min_angle_(min_angle),
      max_angle_(std::min(max_angle, kMaxAngle)),
      min_pulse_width_us_(min_pulse_width_us),
      max_pulse_width_us_(max_pulse_width_us) {
}

bool Servo::Init() {
  return ledcAttach(pin_, kPwmFrequency, kPwmResolution);
}

void Servo::Write(const uint16_t angle) {
  WriteMicroseconds(Map(angle, min_angle_, max_angle_, min_pulse_width_us_, max_pulse_width_us_));
}

void Servo::WriteMicroseconds(const double us) {
  ledcWrite(pin_, us * kMaxPwmDuty / kPwmFactor);
}
}  // namespace em