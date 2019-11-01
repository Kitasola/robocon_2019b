#ifndef ARRC_RASPI_Gy521_HPP
#define ARRC_RASPI_Gy521_HPP
#include "i2c.hpp"
#include <time.h>

namespace arrc_raspi {
constexpr double GY521_LSB_MAP[4] = {131, 65.5, 32.8, 16.4};
enum Gy521RegisterMap {
  WHO_AM_I = 0x75,
  PWR_MGMT_1 = 0x6B,
  FS_SEL = 0x1B,
  GYRO_ZOUT_H = 0x47,
  GYRO_ZOUT_L = 0x48,
  AFS_SEL = 0x1C,
  ACCEL_XOUT_H = 0x3B,
  ACCEL_XOUT_L = 0x3C,
  ACCEL_YOUT_H = 0x3D,
  ACCEL_YOUT_L = 0x3E,
  ACCEL_ZOUT_H = 0x3F,
  ACCEL_ZOUT_L = 0x40,
};

struct Gy521Param {
  double LSB;
  double aver;
  double pose;
};

class Gy521 {
public:
  Gy521(unsigned int dev_id = 0x68, int bit = 2, double user_reg = 1.0);
  void calibration(int calibration = 1000);
  void update();
  double yaw = 0;
  double diff_yaw = 0;
  double checkStatus();
  Gy521Param checkParam();

private:
  I2c i2c_;
  int gyroRead2(enum Gy521RegisterMap register_h,
                enum Gy521RegisterMap register_l) {
    return (i2c_.read(register_h) << 8) + i2c_.read(register_l);
  }
  int bit_ = 2;
  double user_reg_ = 1.0;
  bool finished_calibration = false;
  double gyro_z_now_ = 0;
  double gyro_z_prev_ = 0;
  timespec now_, prev_;
  Gy521Param param_;
};
}; // namespace arrc_raspi
#endif
