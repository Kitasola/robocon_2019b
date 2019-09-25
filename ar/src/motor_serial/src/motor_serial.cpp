#include "motor_serial.hpp"
#include "motor_serial/motor_serial.h"
#include <ros/ros.h>

using ros::MotorSerial;
MotorSerial ms;

bool motorSerialSend(motor_serial::motor_serial::Request &tx,
                     motor_serial::motor_serial::Response &rx) {
  rx.data = ms.send(tx.id, tx.cmd, tx.data);
  return true;
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "motor_serial");
  ros::NodeHandle n;

  ros::ServiceServer service =
      n.advertiseService("motor_speed", motorSerialSend);
  ROS_INFO_STREAM("Start MotorSerial");

  ros::spin();
  ms.send(255, 255, 0);
  return 0;
}
