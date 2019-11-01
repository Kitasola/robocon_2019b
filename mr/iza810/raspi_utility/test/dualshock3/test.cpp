#include "../../include/dualshock3.hpp"
#include "../../include/time.hpp"
#include <iostream>

using namespace std;
using namespace arrc_raspi;

int main() {
  DualShock3 controller;
  Timer loop;
  UPDATELOOP(controller, !controller.button(START)) {
    for (int i = 0; i < NumButtons; ++i) {
      cout << "Push Button: ";
      if (controller.button(ButtonsNum(i))) {
        cout << ButtonsNum(i) << " ";
      }
      cout << endl;
    }
    for (int i = 0; i < NumSticks; ++i, cout << ", ") {
      cout << SticksNum(i) << ":" << controller.stick(SticksNum(i));
    }
    for (int i = 0; i < NumAxis; ++i, cout << ", ") {
      cout << AxisNum(i) << ":" << controller.acceleration(AxisNum(i));
    }
    cout << endl;
    loop.sleep(0.001);
  }
}
