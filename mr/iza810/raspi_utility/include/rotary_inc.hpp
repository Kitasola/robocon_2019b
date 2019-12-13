#ifndef ARRC_RASPI_ROTARY_INC_HPP
#define ARRC_RASPI_ROTARY_INC_HPP
#include "pigpiod.hpp"

namespace arrc_raspi {
class RotaryInc {
public:
  RotaryInc(int user_A, int user_B, int multiplier);
  RotaryInc(int user_A, int user_B, int multiplier, int user_Z);
  ~RotaryInc();
  int get();
  int angle();
  int diff();

private:
  int gpio_handle_;
  int pulse_ = 0;
  int prev_pulse_ = 0;
  int diff_pulse_ = 0;
  int position_ = 0;
  unsigned int pin_A_, pin_B_, pin_Z_;
  unsigned int id_A_, id_B_, id_Z_;
  bool now_A_, now_B_;
  static void rotary(int pi, unsigned int gpio, unsigned int edge,
                     uint32_t tick, void *userdata);
  static void rotaryTwo(int pi, unsigned int gpio, unsigned int edge,
                        uint32_t tick, void *userdata);
  static void rotaryFour(int pi, unsigned int gpio, unsigned int edge,
                         uint32_t tick, void *userdata);
  static void rotaryZ(int pi, unsigned int gpio, unsigned int edge,
                      uint32_t tick, void *userdata);
};
}; // namespace ros
#endif
