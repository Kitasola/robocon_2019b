#include "../../include/motor_serial.hpp"
#include <iostream>

const char *arrc_raspi::PIGPIOD_HOST = "localhost";
const char *arrc_raspi::PIGPIOD_PORT = "8888";

using namespace arrc_raspi;
using namespace std;

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
  cout <<(int)id << " " << (int)cmd << " " << (int)data << endl;
  cout <<ms.send(id, cmd, data) << endl;
  cout <<
      (ms.sum_check_success_ ? "Receive Success" : "Receive Failed") << endl;
  return 0;
}
