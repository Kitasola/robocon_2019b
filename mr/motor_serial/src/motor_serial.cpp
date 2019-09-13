#include "/home/ubuntu/arrc_utility/raspi/include/motor_serial.hpp"
#include "motor_serial/motor_serial.h"
#include<ros/ros.h>

using ros::MotorSerial;
MotorSerial ms;

bool motorSerialSend(motor_serial::motor_serial::Request &tx,motor_serial::motor_serial::Response &rx){
	rx.data = ms.send(tx.id, tx.cmd, tx.data);
	return true;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "motor_serial");
	ros::NodeHandle n;

	ros::ServiceServer service_wheel = n.advertiseService("motor_info", motorSerialSend);
	ros::ServiceServer service_arm = n.advertiseService("robot_arm", motorSerialSend);
	ros::ServiceServer service_calibration = n.advertiseService("arm_calibration", motorSerialSend);
	ROS_INFO_STREAM("Start MotorSerial");

	ros::spin();
	ms.send(255, 255, 0);
	return 0;
}


