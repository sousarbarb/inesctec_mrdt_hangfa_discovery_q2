#include "Motor.h"

void Motor::init(const int pin_dir, const int pin_pwm) {
  dir_pin = pin_dir;
  pwm_pin = pin_pwm;

  pinMode(dir_pin, OUTPUT);
  pinMode(pwm_pin, OUTPUT);

  enable = true;
  pwm = 0;
  setPWM(0);

  if ((pwm_pin == TIMER3_A_PIN) || (pwm_pin == TIMER3_B_PIN) ||
      (pwm_pin == TIMER3_C_PIN)) {
    Timer3.pwm(pwm_pin, 0);
  } else if ((pwm_pin == TIMER1_A_PIN) || (pwm_pin == TIMER1_B_PIN)) {
    Timer1.pwm(pwm_pin, 0);
  }
}

void Motor::setPWM(int16_t new_pwm) {
  // Saturation
  if (new_pwm > kMotPWMmax) {
    new_pwm = kMotPWMmax;
  } else if (new_pwm < -kMotPWMmax) {
    new_pwm = -kMotPWMmax;
  }

  // Reset if disabled
  if (!enable) {
    new_pwm = 0;
  }

  // Limit PWM change
  if (kMotPWMDeltaMaxEnabled) {
    if (new_pwm - pwm > kMotPWMDeltaMax) {
      new_pwm = pwm + kMotPWMDeltaMax;
    } else if (new_pwm - pwm < -kMotPWMDeltaMax) {
      new_pwm = pwm - kMotPWMDeltaMax;
    }
  }

  // Set pwm
  if (enable) {
    if (new_pwm >= 0) {
      digitalWrite(dir_pin, 0);

      if ((pwm_pin == TIMER3_A_PIN) || (pwm_pin == TIMER3_B_PIN) ||
          (pwm_pin == TIMER3_C_PIN)) {
        Timer3.setPwmDuty(pwm_pin, new_pwm);
      } else if ((pwm_pin == TIMER1_A_PIN) || (pwm_pin == TIMER1_B_PIN)) {
        Timer1.setPwmDuty(pwm_pin, new_pwm);
      }
    } else {
      digitalWrite(dir_pin, 1);
      
      if ((pwm_pin == TIMER3_A_PIN) || (pwm_pin == TIMER3_B_PIN) ||
          (pwm_pin == TIMER3_C_PIN)) {
        Timer3.setPwmDuty(pwm_pin, -new_pwm);
      } else if ((pwm_pin == TIMER1_A_PIN) || (pwm_pin == TIMER1_B_PIN)) {
        Timer1.setPwmDuty(pwm_pin, -new_pwm);
      }
    }

  } else {
    if ((pwm_pin == TIMER3_A_PIN) || (pwm_pin == TIMER3_B_PIN) ||
        (pwm_pin == TIMER3_C_PIN)) {
      Timer3.setPwmDuty(pwm_pin, new_pwm);
    } else if ((pwm_pin == TIMER1_A_PIN) || (pwm_pin == TIMER1_B_PIN)) {
      Timer1.setPwmDuty(pwm_pin, new_pwm);
    }
  }

  // Save pwm value
  pwm = new_pwm;
}
