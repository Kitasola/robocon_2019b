#ifndef ARRC_RASPI_MOTOR_SERIAL_HPP
#define ARRC_RASPI_MOTOR_SERIAL_HPP
#include "pigpiod.hpp"
#include "serial.hpp"
#include <fstream>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

namespace arrc_raspi {
struct SendDataFormat {
  unsigned char id;
  unsigned char cmd;
  short argData;
};

class MotorSerial {
public:
  MotorSerial(const char *dev_file = "/dev/ttyAMA0", int baudrate = 115200,
              int rede = 4, int timeout = 10);
  int send();
  void setTimeOut(int timeout);
  short sending(unsigned char id, unsigned char cmd, short data);
  short send(unsigned char id, unsigned char cmd, short data,
             bool async_flag = false);
  short send(const SendDataFormat &send_data, bool async_flag);
  virtual ~MotorSerial();
  bool sum_check_success_;
  short recent_receive_data_;

private:
  Serial serial_;
  void sendingLoop();
  bool thread_loop_flag_;
  int timeout_;
  int rede_pin_;
  std::thread send_thread_;
  std::queue<SendDataFormat> send_data_queue_;
  std::mutex mtx_;
};
}; // namespace arrc_raspi
#endif
