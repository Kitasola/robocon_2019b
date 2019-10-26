#ifndef ARRC_RASPI_GY521_HPP
#define ARRC_RASPI_GY521_HPP
#include "i2c.hpp"
#include <time.h>

namespace arrc_raspi {
constexpr double GY521_LSB_MAP[4] = {131, 65.5, 32.8, 16.4};
enum GY521RegisterMap {
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

struct GY521Param {
  double LSB;
  double aver;
  double pose;
};

class GY521 {
public:
  GY521(unsigned int dev_id = 0x68, int bit = 2, int calibration = 1000,
        double user_reg = 1.0);
  double yaw_;
  double diff_yaw_;
  void update();
  void start(double start = 0) {
    clock_gettime(CLOCK_REALTIME, &now_);
    yaw_ = start;
  }
  double checkStatus();
  GY521Param checkParam();

private:
  I2c i2c_;
  int gyroRead2(enum GY521RegisterMap register_h,
                enum GY521RegisterMap register_l) {
    return (i2c_.read(register_h) << 8) + i2c_.read(register_l);
  }
  double gyro_z_now_ = 0;
  double gyro_z_prev_ = 0;
  timespec now_, prev_;
  GY521Param param_;
};
}; // namespace arrc_raspi
#endif
