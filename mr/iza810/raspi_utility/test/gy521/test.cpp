#include "../../include/GY521.hpp"

using namespace arrc_raspi;

int main(int argc, char *argv[]) {
  GY521 gyro;
  gyro.start();

  while (1) {
    gyro.update();
  }

  return 0;
}
