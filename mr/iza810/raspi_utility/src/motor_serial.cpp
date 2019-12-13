#include "../include/motor_serial.hpp"
#include "../include/pigpiod.hpp"
#include "../include/serial.hpp"
#include <chrono>
#include <cstring>
#include <iostream>
#include <mutex>
#include <pthread.h>
#include <queue>
#include <string>
#include <thread>
#include <unistd.h>

using namespace arrc_raspi;
using namespace std;

constexpr int STX = 0x41;
constexpr int SEND_DATA_NUM = 7;

MotorSerial::MotorSerial(const char *dev_file, int baudrate, int rede,
                         int timeout) {
  if (serial_.init(dev_file, baudrate)) {
    sum_check_success_ = false;
    recent_receive_data_ = 0;
    thread_loop_flag_ = false;
    timeout_ = timeout;
    rede_pin_ = rede;
    cout << "MotorSerial Initialize Success" << endl;
  } else {
    cout << "Serial Initialize Failed" << endl;
  }

  Pigpiod::gpio().set(rede_pin_, OUT, 0);
}

void MotorSerial::setTimeOut(int timeout) { timeout_ = timeout; }

short MotorSerial::sending(unsigned char id, unsigned char cmd, short data) {
  unsigned short u_data = (unsigned short)data;
  unsigned char send_array[SEND_DATA_NUM] = {
      0xFF,
      STX,
      id,
      cmd,
      (unsigned char)(u_data & 0xFF),
      (unsigned char)(u_data >> 8),
      (unsigned char)((id + cmd + (u_data & 0xFF) + (u_data >> 8)) & 0xFF)};
  lock_guard<mutex> lock(mtx_);

  Pigpiod::gpio().write(rede_pin_, 1);
  for (int i = 0; i < SEND_DATA_NUM; ++i) {
    serial_.write(send_array[i]);
    Pigpiod::gpio().delay(90);
  }
  Pigpiod::gpio().write(rede_pin_, 0);

  bool stx_flag = false;
  char receive_array[5] = {};
  int i = 0;

  auto start_time = std::chrono::system_clock::now();
  sum_check_success_ = false;
  while (std::chrono::time_point<std::chrono::system_clock>(
             start_time + std::chrono::milliseconds(timeout_)) >=
             std::chrono::system_clock::now() &&
         !sum_check_success_) {
    while (serial_.available() > 0) {
      char got_data = serial_.read();
      if (got_data == STX && !stx_flag) {
        stx_flag = true;
        continue;
      }
      if (stx_flag) {
        receive_array[i++] = got_data;
      }
      if (i > 4) {
        unsigned char sum = 0;
        for (int j = 0; j < 4; ++j)
          sum += receive_array[j];
        if (sum == receive_array[4]) {
          sum_check_success_ = true;
          break;
        }
      }
    }
  }
  if (serial_.available() < 0) {
    cout << "Serial Com Error" << endl;
  }
  return (recent_receive_data_ = receive_array[2] + (receive_array[3] << 8));
}

void MotorSerial::sendingLoop(void) {
  while (!send_data_queue_.empty()) {
    thread_loop_flag_ = true;
    SendDataFormat sendData = send_data_queue_.front();
    send_data_queue_.pop();
    sending(sendData.id, sendData.cmd, sendData.argData);
  }
  thread_loop_flag_ = false;
}

short MotorSerial::send(unsigned char id, unsigned char cmd, short data,
                        bool async_flag) {
  if (async_flag) {
    SendDataFormat sendData = {id, cmd, data};
    send_data_queue_.push(sendData);
    if (!thread_loop_flag_) {
      if (send_thread_.joinable())
        send_thread_.join();
      send_thread_ = thread([&] { sendingLoop(); });
      sched_param sch_params;
      sch_params.sched_priority = 1;
      if (pthread_setschedparam(send_thread_.native_handle(), SCHED_RR,
                                &sch_params)) {
        cout << "Failed to set Thread scheduling" << endl;
      }
    }
    return 0;
  }
  return sending(id, cmd, data);
}

short MotorSerial::send(const SendDataFormat &send_data, bool async_flag) {
  return send(send_data.id, send_data.cmd, send_data.argData, async_flag);
}

MotorSerial::~MotorSerial() {
  if (send_thread_.joinable()) {
    send_thread_.join();
  }
}
