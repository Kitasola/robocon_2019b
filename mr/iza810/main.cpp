//立ルンです, Ma/nLoop from 294
#include "raspi_utility/include/dualshock3.hpp"
#include "raspi_utility/include/gy521.hpp"
#include "raspi_utility/include/motor_serial.hpp"
#include "raspi_utility/include/pigpiod.hpp"
#include "raspi_utility/include/time.hpp"
#include <cmath>
#include <iostream>
#include <pigpio.h>
#include <string>
#include <vector>

using namespace std;
using namespace arrc_raspi;

inline bool yaw_check(int goal, int now);
constexpr double ErrorYaw = 3;

int main() {
  // Serial
  MotorSerial ms;

  // DualShock3
  DualShock3 controller;

  // Check LED
  constexpr int RUN_LED = 13;
  pigpio.set(RUN_LED, OUT, 0);

  // Wait Calibration
  UPDATELOOP(controller,
             !(controller.button(RIGHT) && controller.button(SQUARE))) {}
  Gy521 gyro(0x68, 2, 1000, 1.01);

  // three omuni wheel
  constexpr int NUM_WHEEL = 3, WHEEL_MDD_ID[NUM_WHEEL] = {1, 2, 3},
                WHEEL_CMD[NUM_WHEEL] = {2, 2, 2};
  constexpr double WHEEL_ANGLE[NUM_WHEEL] = {};
  cout << "Main Start" << endl;
  // MainLoop
  UPDATELOOP(controller,
             !(controller.button(START) && controller.button(CROSS))) {}
finish:
  cout << "Main Finish" << endl;
  ms.send(255, 255, 0);
}

inline bool yaw_check(int goal, int now) {
  int small = goal + 360 - ErrorYaw;
  int large = goal + 360 + ErrorYaw;
  now = (now + 360) % 360 + 360;
  if (now > small && now < large) {
    return 1;
  }
  return 0;
}
