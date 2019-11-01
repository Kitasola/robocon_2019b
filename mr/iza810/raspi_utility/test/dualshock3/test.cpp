#include "../../include/dualshock3.hpp"
#include "../../include/time.hpp"
#include <iostream>

using namespace std;
using namespace arrc_raspi;

int main() {
  DualShock3 controller;
  Timer loop;

  const char *BUTTON[NumButtons] = {"SELECT",   "LEFT_STICK", "RIGHT_STICK",
                                    "START",    "UP",         "RIGHT",
                                    "DOWN",     "LEFT",       "L2",
                                    "R2",       "L1",         "R1",
                                    "TRIANGLE", "CIRCLE",     "CROSS",
                                    "SQUARE"},
             *STICK[NumSticks] = {"LEFT_X",  "LEFT_Y", "RIGHT_X",
                                  "RIGHT_Y", "LEFT_T", "RIGHT_T"};
  /* *AXIS[NumAxis] = {"X_AXIS", "Y_AXIS", "Z_AXIS"}; */

  UPDATELOOP(controller, !controller.button(START)) {
    cout << "Push Button: ";
    for (int i = 0; i < NumButtons; ++i) {
      if (controller.button(ButtonsNum(i))) {
        cout << BUTTON[i] << " ";
      }
    }
    cout << endl;
    for (int i = 0; i < NumSticks; ++i, cout << ", ") {
      cout << STICK[i] << ":" << controller.stick(SticksNum(i));
    }
    /* for (int i = 0; i < NumAxis; ++i, cout << ", ") { */
    /*   cout << AXIS[i] << ":" << controller.acceleration(AxisNum(i)); */
    /* } */
    cout << endl;
    loop.sleep(0.01);
  }
}
