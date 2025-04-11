#pragma once

#include <Arduino.h>
#include <TimerOne.h>
#include <TimerThree.h>

#include "robotconfig.h"

class Motor
{
 public:

  bool enable;
  int16_t pwm;

  int dir_pin;
  int pwm_pin;

 public:

  void init(const int pin_dir, const int pin_pwm);
  void setPWM(int16_t new_pwm);
};
