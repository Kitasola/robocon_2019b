#include "../../include/motor_serial.hpp"
#include <iostream>
#include <ros/console.h>

using ros::MotorSerial;
using std::stoi;

int main(int argc, char *argv[]) {
  MotorSerial ms;

  unsigned char id, cmd;
  unsigned short data;
  if (argc < 4) {
    id = 1;
    cmd = 2;
    data = 0;
  } else {
    id = (unsigned char)stoi(argv[1]);
    cmd = (unsigned char)stoi(argv[2]);
    data = (short)stoi(argv[3]);
  }
  ROS_INFO_STREAM((int)id << " " << (int)cmd << " " << (int)data);
  ROS_INFO_STREAM(ms.send(id, cmd, data));
  ROS_INFO_STREAM(
      (ms.sum_check_success_ ? "Receive Success" : "Receive Failed"));
  return 0;
}
