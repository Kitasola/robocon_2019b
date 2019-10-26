#include "../include/gy521.hpp"
#include "../include/i2c.hpp"
#include <cmath>
#include <ros/console.h>
#include <time.h>

using namespace arrc_raspi;

GY521::GY521(unsigned int dev_id, int bit, int calibration, double user_reg) {
  if (i2c_.init(dev_id)) {
    if ((unsigned int)i2c_.read(ros::GY521RegisterMap(WHO_AM_I)) == dev_id) {
      if (i2c_.read(ros::GY521RegisterMap(PWR_MGMT_1)) == 0x40) {
        i2c_.write(ros::GY521RegisterMap(PWR_MGMT_1), 0x00);
        ROS_DEBUG_STREAM("UnLock Sleep Mode");
      }
      ROS_DEBUG_STREAM("GY521 Initialize Success");
    } else {
      ROS_ERROR_STREAM("Not Found Devaice: " << dev_id);
    }
  } else {
    ROS_ERROR_STREAM("I2C Initialize Failed");
  }

  // Skew Detection
  short accel_x = 0, accel_y = 0, accel_z = 0;
  double accel_x_aver = 0, accel_y_aver = 0, accel_z_aver = 0;
  // AccelSensar, Max:2[g], LSB:16384[LSB/g]
  i2c_.write(ros::GY521RegisterMap(AFS_SEL), 0x00);

  ROS_INFO_STREAM("Calibration Phase Accel Start.");
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

  ROS_INFO_STREAM("Calibration Phase Gyro Start.");
  // Gyro init
  i2c_.write(ros::GY521RegisterMap(FS_SEL), bit << 3);
  param_.LSB = ros::GY521_LSB_MAP[bit] * param_.pose / user_reg;

  // Calibration gyroZAver(deg/s)
  short gyro_z;
  param_.aver = 0;
  for (int i = 0; i < calibration; ++i) {
    gyro_z = gyroRead2(GYRO_ZOUT_H, GYRO_ZOUT_L);
    param_.aver += gyro_z;
  }
  param_.aver /= calibration;

  yaw_ = diff_yaw_ = 0;
  ROS_INFO_STREAM(param_.LSB << ", " << param_.aver << ", " << param_.pose);
}

void GY521::update() {
  short gyro_z = gyroRead2(GYRO_ZOUT_H, GYRO_ZOUT_L);
  prev_ = now_;
  clock_gettime(CLOCK_REALTIME, &now_);
  gyro_z_prev_ = gyro_z_now_;
  gyro_z_now_ = ((double)gyro_z - param_.aver) / param_.LSB;
  ROS_INFO_STREAM(gyro_z_now_);
  diff_yaw_ = (gyro_z_now_ + gyro_z_prev_) / 2 *
              (now_.tv_sec - prev_.tv_sec +
               (long double)(now_.tv_nsec - prev_.tv_nsec) / 1.0e+9);
  yaw_ += diff_yaw_;
  if (yaw_ > 180) {
    yaw_ -= 360;
  } else if (yaw_ <= -180) {
    yaw_ += 360;
  }
}

ros::GY521Param GY521::checkParam() {
  return param_;
}
