#include "../include/i2c.hpp"
#include "../include/pigpiod.hpp"
#include <pigpiod_if2.h>

using namespace arrc_raspi;

I2c::I2c() { gpio_handle_ = Pigpiod::gpio().checkHandle(); }

bool I2c::init(unsigned int dev_id) {
  gpio_handle_ = Pigpiod::gpio().checkHandle();
  unsigned int dummy_flag = 0;
  i2c_handle_ = i2c_open(gpio_handle_, 1, dev_id, dummy_flag);
  return i2c_handle_ < 0 ? 0 : 1;
}

void I2c::write(unsigned int reg, unsigned int data) {
  i2c_write_byte_data(gpio_handle_, i2c_handle_, reg, data);
}

int I2c::read(unsigned int reg) {
  return i2c_read_byte_data(gpio_handle_, i2c_handle_, reg);
}

I2c::~I2c() { i2c_close(gpio_handle_, i2c_handle_); }
