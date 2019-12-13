#include "../include/pigpiod.hpp"
#include "../include/rotary_inc.hpp"
#include <pigpiod_if2.h>

using namespace arrc_raspi;

RotaryInc::RotaryInc(int user_A, int user_B, int multiplier) {
  pin_A_ = user_A;
  pin_B_ = user_B;

  gpio_handle_ = Pigpiod::gpio().checkHandle();

  Pigpiod::gpio().set(pin_A_, IN, PULL_UP);
  Pigpiod::gpio().set(pin_B_, IN, PULL_UP);

  set_watchdog(gpio_handle_, pin_A_, 0);
  set_watchdog(gpio_handle_, pin_B_, 0);

  switch (multiplier) {
  case 2:
    id_A_ = callback_ex(gpio_handle_, pin_A_, EITHER_EDGE, rotaryTwo, this);
    id_B_ = callback_ex(gpio_handle_, pin_B_, EITHER_EDGE, rotaryTwo, this);
    break;
  case 4:
    id_A_ = callback_ex(gpio_handle_, pin_A_, EITHER_EDGE, rotaryFour, this);
    id_B_ = callback_ex(gpio_handle_, pin_B_, EITHER_EDGE, rotaryFour, this);
    break;
  default:
    id_A_ = callback_ex(gpio_handle_, pin_A_, EITHER_EDGE, rotary, this);
    id_B_ = callback_ex(gpio_handle_, pin_B_, EITHER_EDGE, rotary, this);
    break;
  }
}

RotaryInc::RotaryInc(int user_A, int user_B, int multiplier, int user_Z) {
  RotaryInc(user_A, user_B, multiplier);
  pin_Z_ = user_Z;

  Pigpiod::gpio().set(pin_Z_, IN, PULL_UP);
  set_watchdog(gpio_handle_, pin_Z_, 0);

  id_Z_ = callback_ex(gpio_handle_, pin_Z_, RISING_EDGE, rotaryZ, this);
}

int RotaryInc::get() { return pulse_; }

int RotaryInc::angle() { return position_; }

int RotaryInc::diff() {
  diff_pulse_ = pulse_ - prev_pulse_;
  prev_pulse_ = pulse_;
  return diff_pulse_;
}

void RotaryInc::rotary(int pi, unsigned int gpio, unsigned int edge,
                       uint32_t tick, void *userdata) {
  RotaryInc *regist = (RotaryInc *)userdata;

  if (gpio == regist->pin_A_) {
    regist->now_A_ = edge;
    if (edge) {
      regist->now_B_ ? ++(regist->pulse_) : --(regist->pulse_);
      regist->now_B_ ? ++(regist->position_) : --(regist->position_);
    }
  } else if (gpio == regist->pin_B_) {
    regist->now_B_ = edge;
  }
}

void RotaryInc::rotaryTwo(int pi, unsigned int gpio, unsigned int edge,
                          uint32_t tick, void *userdata) {
  RotaryInc *regist = (RotaryInc *)userdata;

  if (gpio == regist->pin_A_) {
    regist->now_A_ = edge;
    if (edge) {
      regist->now_B_ ? ++(regist->pulse_) : --(regist->pulse_);
    } else {
      regist->now_B_ ? --(regist->pulse_) : ++(regist->pulse_);
    }
  } else {
    regist->now_B_ = edge;
  }
}

void RotaryInc::rotaryFour(int pi, unsigned int gpio, unsigned int edge,
                           uint32_t tick, void *userdata) {
  RotaryInc *regist = (RotaryInc *)userdata;
  if (gpio == regist->pin_A_) {
    regist->now_A_ = edge;
    if (edge) {
      regist->now_B_ ? ++(regist->pulse_) : --(regist->pulse_);
    } else {
      regist->now_B_ ? --(regist->pulse_) : ++(regist->pulse_);
    }
  } else {
    regist->now_B_ = edge;
    if (edge) {
      regist->now_B_ ? ++(regist->pulse_) : --(regist->pulse_);
    } else {
      regist->now_B_ ? --(regist->pulse_) : ++(regist->pulse_);
    }
  }
}

void RotaryInc::rotaryZ(int pi, unsigned int gpio, unsigned int edge,
                        uint32_t tick, void *userdata) {
  RotaryInc *regist = (RotaryInc *)userdata;
  if (gpio == regist->pin_Z_) {
    regist->position_ = 0;
  }
}

RotaryInc::~RotaryInc() {
  callback_cancel(id_A_);
  callback_cancel(id_B_);
  callback_cancel(id_Z_);
}
