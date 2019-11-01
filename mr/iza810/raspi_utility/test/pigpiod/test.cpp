#include "../../include/pigpiod.hpp"
#include <iostream>
#include <pigpiod_if2.h>
#include <string>

using namespace arrc_raspi;
using namespace std;

const char *arrc_raspi::PIGPIOD_HOST = "localhost";
const char *arrc_raspi::PIGPIOD_PORT = "8888";
int main(int argc, char **argv) {
  if (argc < 4) {
    cout << "Too few arguments.\nex)\n./test 3 OUT (0 | 1)\n./test 4 IN (UP | "
            "DOWN | OFF)"
         << endl;
    return -1;
  }
  if (!Pigpiod::gpio().checkInit()) {
    cout << "Pigpiod Initialize Failed" << endl;
    return -1;
  }
  if (argv[2] == string("OUT")) {
    Pigpiod::gpio().set(stoi(argv[1]), OUT, stoi(argv[3]));
  } else if (argv[2] == string("IN")) {
    if (argv[3] == string("UP")) {
      Pigpiod::gpio().set(stoi(argv[1]), IN, PULL_UP);
    } else if (argv[3] == string("DOWN")) {
      Pigpiod::gpio().set(stoi(argv[1]), IN, PULL_DOWN);
    } else if (argv[3] == string("OFF")) {
      Pigpiod::gpio().set(stoi(argv[1]), IN, PULL_OFF);
    } else {
      cout << "No Pull Mode. Choose (UP | DOWN | OFF)." << endl;
      return -1;
    }
    cout << Pigpiod::gpio().read(stoi(argv[1])) << endl;
  } else {
    cout << "No I/O Mode. Choose (OUT | IN)." << endl;
    return -1;
  }
  return 0;
}
