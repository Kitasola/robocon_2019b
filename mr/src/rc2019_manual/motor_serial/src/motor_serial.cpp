#include "motor_serial.hpp"
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
	ros::ServiceServer service_expansion = n.advertiseService("robot_expansion", motorSerialSend);
	ros::ServiceServer service_laundry = n.advertiseService("laundry_basket", motorSerialSend);
	ros::ServiceServer service_shotting = n.advertiseService("shotting_cloths", motorSerialSend);
	ros::ServiceServer service_hand = n.advertiseService("hand_info", motorSerialSend);
	ros::ServiceServer service_led = n.advertiseService("tape_led", motorSerialSend);
	ros::ServiceServer service_head = n.advertiseService("head", motorSerialSend);
	ROS_INFO_STREAM("Start MotorSerial");

	ros::spin();
	ms.send(255, 255, 0);
	return 0;
}


