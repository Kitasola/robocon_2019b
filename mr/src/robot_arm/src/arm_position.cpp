#include<ros/ros.h>
#include<iostream>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Bool.h>
#include<three_omuni/button.h>
#include<pigpiod.hpp>
#include<motor_serial.hpp>
#include"motor_serial/motor_serial.h"
#include<std_msgs/String.h>
using ros::Pigpiod;
motor_serial::motor_serial srv;
std_msgs::Float64MultiArray angle_data;
std_msgs::String send_answer;
void controllerCallback(const three_omuni::button &button);
void arm_pose(const int angle_now);
void calibrationCallback(const std_msgs::String &file_send);

constexpr int motion_sum = 3;
bool flag_calibration = false;
int arm_count = 0;
int count_prev = 0;
double arm_angle_1 = 0, arm_angle_2 = 0;
ros::ServiceClient calibration;

int main(int argc, char **argv){
	ros::init(argc, argv, "arm_position");
	ros::NodeHandle n;
	ros::Subscriber position_sub = n.subscribe("controller_info", 10, controllerCallback);
	ros::Subscriber calibration_sub = n.subscribe("calibration", 30, calibrationCallback);
	ros::Publisher position_pub = n.advertise<std_msgs::Float64MultiArray>("angle_info", 10);
	calibration = n.serviceClient<motor_serial::motor_serial>("arm_calibration");

		
	angle_data.data.resize(2);
	ros::Rate loop_rate(100);

	while(ros::ok()){
		if(flag_calibration){
			if(arm_count > motion_sum) arm_count = 0;
			position_pub.publish(angle_data);
		}
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}

void controllerCallback(const three_omuni::button &button){
	if(button.arm_data != count_prev){
		if(button.arm_data) ++arm_count;
	}
	count_prev = button.arm_data;
	arm_pose(arm_count);
}

void arm_pose(const int angle_goal){
	switch(angle_goal){
		case 1:
			angle_data.data[0] = 200;
			angle_data.data[1] = 250;
			break;
		case 2:
			angle_data.data[0] = 200;
			angle_data.data[1] = 250;
			break;
		case 3:
			angle_data.data[0] = 200;
			angle_data.data[1] = 250;
			break;
		default:
			angle_data.data[0] = 200;
			angle_data.data[1] = 250;
			break;
	}
}
void calibrationCallback(const std_msgs::String &file_send){
	if(file_send.data == "arm"){
		constexpr int pin_Z_1 = 1;
		constexpr int pin_Z_2 = 2;
		while(!(Pigpiod::gpio().read(pin_Z_1))){
			srv.request.id = 1;
			srv.request.cmd = 2;
			srv.request.data = 50;
			calibration.call(srv);
		}
		while(!(Pigpiod::gpio().read(pin_Z_2))){
			srv.request.id = 1;
			srv.request.cmd = 3;
			srv.request.data = 50;
			calibration.call(srv);
		}
		flag_calibration = true;
		send_answer.data = "complete_arm";
	}
}
