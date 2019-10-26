#ifndef ARRC_RASPI_I2C_HPP
#define ARRC_RASPI_I2C_HPP
#include "pigpiod.hpp"
#include <pigpiod_if2.h>

namespace arrc_raspi {
class I2c {
public:
  I2c();
  bool init(unsigned int dev_id);
  void write(unsigned int reg, unsigned int data);
  int read(unsigned int reg);
  ~I2c();

private:
  int gpio_handle_;
  int i2c_handle_;
};
};
#endif
