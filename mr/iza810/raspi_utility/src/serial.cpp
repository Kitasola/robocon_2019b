#include "../include/pigpiod.hpp"
#include "../include/serial.hpp"
#include <pigpiod_if2.h>

using namespace arrc_raspi;

Serial::Serial() { gpio_handle_ = Pigpiod::gpio().checkHandle(); }

bool Serial::init(const char *dev_file, int baudrate) {
  unsigned char dummy_flag = 0;
  serial_handle_ = serial_open(gpio_handle_, const_cast<char *>(dev_file),
                               baudrate, dummy_flag);
  return serial_handle_ < 0 ? false : true;
}

int Serial::write(unsigned char tx_data) {
  return serial_write_byte(gpio_handle_, serial_handle_, tx_data);
}

int Serial::writes(char *tx_data, unsigned int tx_len) {
  return serial_write(gpio_handle_, serial_handle_, tx_data, tx_len);
}

int Serial::read() { return serial_read_byte(gpio_handle_, serial_handle_); }

int Serial::reads(char *rx_data, unsigned int rx_len) {
  return serial_read(gpio_handle_, serial_handle_, rx_data, rx_len);
}

int Serial::available() {
  return serial_data_available(gpio_handle_, serial_handle_);
}

Serial::~Serial() { serial_close(gpio_handle_, serial_handle_); }
