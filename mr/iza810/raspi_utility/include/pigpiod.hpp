#ifndef ARRC_RASPI_PIGPIOD_HPP
#define ARRC_RASPI_PIGPIOD_HPP
#include <pigpiod_if2.h>

namespace arrc_raspi {
extern const char *PIGPIOD_HOST;
extern const char *PIGPIOD_PORT;

constexpr int LOW = 0;
constexpr int HIGH = 1;
constexpr int IN = 0;
constexpr int OUT = 1;
constexpr int PULL_UP = 2;
constexpr int PULL_DOWN = 3;
constexpr int PULL_OFF = 4;

class Pigpiod {
public:
  static Pigpiod &gpio() {
    static Pigpiod gpio;
    return gpio;
  }
  void set(int pin, int mode, int init);
  void write(int pin, int level);
  int read(int pin);
  int checkHandle();
  bool checkInit();
  void delay(double micro_sec);

private:
  int gpio_handle_;
  Pigpiod();
  ~Pigpiod();
};
}; // namespace arrc_raspi
#endif
