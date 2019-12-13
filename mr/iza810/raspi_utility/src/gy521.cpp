#include "../include/gy521.hpp"
#include "../include/i2c.hpp"
#include <cmath>
#include <iostream>
#include <time.h>

using namespace arrc_raspi;
using namespace std;

Gy521::Gy521(unsigned int dev_id, int bit, double user_reg) {
  if (i2c_.init(dev_id)) {
    if ((unsigned int)i2c_.read(Gy521RegisterMap(WHO_AM_I)) == dev_id) {
      if (i2c_.read(Gy521RegisterMap(PWR_MGMT_1)) == 0x40) {
        i2c_.write(Gy521RegisterMap(PWR_MGMT_1), 0x00);
        cout << "UnLock Sleep Mode" << endl;
      }
      cout << "Gy521 Initialize Success" << endl;
    } else {
      cout << "Not Found Devaice: " << dev_id << endl;
    }
  } else {
    cout << "I2C Initialize Failed" << endl;
  }
  // AccelSensar, Max:2[g], LSB:16384[LSB/g]
  i2c_.write(Gy521RegisterMap(AFS_SEL), 0x00);
  bit_ = bit;
  user_reg_ = user_reg;
  i2c_.write(Gy521RegisterMap(FS_SEL), bit_ << 3);
}

void Gy521::calibration(int calibration) {
  // Skew Detection
  short accel_x = 0, accel_y = 0, accel_z = 0;
  double accel_x_aver = 0, accel_y_aver = 0, accel_z_aver = 0;

  cout << "Calibration Phase Accel Start." << endl;
  for (int i = 0; i < calibration; ++i) {
    accel_x = gyroRead2(ACCEL_XOUT_H, ACCEL_XOUT_L);
    accel_y = gyroRead2(ACCEL_YOUT_H, ACCEL_YOUT_L);
    accel_z = gyroRead2(ACCEL_ZOUT_H, ACCEL_ZOUT_L);
    accel_x_aver += accel_x;
    accel_y_aver += accel_y;
    accel_z_aver += accel_z;
  }
  accel_x_aver /= calibration;
  accel_y_aver /= calibration;
  accel_z_aver /= calibration;
  param_.pose =
      accel_z_aver / hypot(hypot(accel_x_aver, accel_y_aver), accel_z_aver);

  cout << "Calibration Phase Gyro Start." << endl;
  // Gyro init
  param_.LSB = GY521_LSB_MAP[bit_] * param_.pose / user_reg_;

  // Calibration gyroZAver(deg/s)
  short gyro_z;
  param_.aver = 0;
  for (int i = 0; i < calibration; ++i) {
    gyro_z = gyroRead2(GYRO_ZOUT_H, GYRO_ZOUT_L);
    param_.aver += gyro_z;
  }
  param_.aver /= calibration;

  clock_gettime(CLOCK_REALTIME, &now_);
  cout << param_.LSB << ", " << param_.aver << ", " << param_.pose << endl;
}
void Gy521::update() {
  if (finished_calibration) {
    return;
  }
  short gyro_z = gyroRead2(GYRO_ZOUT_H, GYRO_ZOUT_L);
  prev_ = now_;
  clock_gettime(CLOCK_REALTIME, &now_);
  gyro_z_prev_ = gyro_z_now_;
  gyro_z_now_ = ((double)gyro_z - param_.aver) / param_.LSB;
  diff_yaw = (gyro_z_now_ + gyro_z_prev_) / 2 *
             (now_.tv_sec - prev_.tv_sec +
              (long double)(now_.tv_nsec - prev_.tv_nsec) / 1.0e+9);
  yaw += diff_yaw;
  if (yaw > 180) {
    yaw -= 360;
  } else if (yaw <= -180) {
    yaw += 360;
  }
}

Gy521Param Gy521::checkParam() { return param_; }
