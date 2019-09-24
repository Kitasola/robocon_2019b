#include<ros/ros.h>
#include<iostream>
#include<std_msgs/Float64MultiArray.h>
#include<std_msgs/Bool.h>
#include<three_omuni/button.h>
#include<pigpiod.hpp>
#include<motor_serial.hpp>
#include"motor_serial/motor_serial.h"
#include<std_msgs/String.h>
#include<iostream>

using std::cout;
using std::endl;
using ros::Pigpiod;

motor_serial::motor_serial srv;
std_msgs::Float64MultiArray angle_data;
std_msgs::String send_answer;
void controllerCallback(const three_omuni::button &button);
void arm_pose(const int angle_now);
void calibrationCallback(const std_msgs::String &file_send);
static void calibrationFlag_1(int pi, unsigned int gpio, unsigned int edge, uint32_t tick);
static void calibrationFlag_2(int pi, unsigned int gpio, unsigned int edge, uint32_t tick);

constexpr int motion_sum = 3;
constexpr unsigned int pin_Z_1 = 5;
constexpr unsigned int pin_Z_2 = 17;
bool flag_calibration = false;
bool flag_z_1 = false;
bool flag_z_2 = false; 
int arm_count = 0;
int count_prev = 0;
int gpio_handle_ = 0;
double arm_angle_1 = 0, arm_angle_2 = 0;
ros::ServiceClient calibration;

int main(int argc, char **argv){
	ros::init(argc, argv, "arm_position");
	ros::NodeHandle n;
	ros::Subscriber position_sub = n.subscribe("controller_info", 10, controllerCallback);
	ros::Subscriber calibration_sub = n.subscribe("calibration", 30, calibrationCallback);
	ros::Publisher position_pub = n.advertise<std_msgs::Float64MultiArray>("angle_info", 10);
	//ros::Publisher calibration_pub = n.advertise<std_msgs::Bool>("z_calibration", 10);
	calibration = n.serviceClient<motor_serial::motor_serial>("arm_calibration");
	
	gpio_handle_ = Pigpiod::gpio().checkHandle();
	Pigpiod::gpio().set(pin_Z_1, ros::IN, ros::PULL_UP);		 
        Pigpiod::gpio().set(pin_Z_2, ros::IN, ros::PULL_UP);	
	unsigned int z_1 = callback(gpio_handle_, pin_Z_1, RISING_EDGE, calibrationFlag_1);
	unsigned int z_2 = callback(gpio_handle_, pin_Z_2, RISING_EDGE, calibrationFlag_2);
	angle_data.data.resize(2);
	ros::Rate loop_rate(1000);

	while(ros::ok()){
		//std_msgs::Bool calibration;
		if(flag_calibration){
			if(arm_count > motion_sum) arm_count = 0;
			position_pub.publish(angle_data);
			ROS_INFO("%d", (int)angle_data.data[0]);
			//calibration.data = true;
		}else{
			//calibration.data = false;
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
			angle_data.data[0] = 300;
			angle_data.data[1] = 200;
			break;
		case 2:
			angle_data.data[0] = 310;
			angle_data.data[1] = 210;
			break;
		case 3:
			angle_data.data[0] = 320;
			angle_data.data[1] = 320;
			break;
		default:
			angle_data.data[0] = 330;
			angle_data.data[1] = 330;
			break;
	}
}
void calibrationCallback(const std_msgs::String &file_send){
	if(file_send.data == "arm"){
		if(flag_z_1 == false){
			srv.request.id = 4;
			srv.request.cmd = 3;
			srv.request.data = 30;
			calibration.call(srv);
		}
		if(flag_z_1 == true){
			srv.request.id = 4;
			srv.request.cmd = 3;
			srv.request.data = 0;
			calibration.call(srv);
		}

		if(flag_z_2 == false){
			srv.request.id = 4;
			srv.request.cmd = 4;
			srv.request.data = 30;
			calibration.call(srv);
		}

		if(flag_z_2 == true){
			srv.request.id = 4;
			srv.request.cmd = 4;
			srv.request.data = 0;
			calibration.call(srv);
		}

		if(flag_z_1 && flag_z_2){
			flag_calibration = true;
			send_answer.data = "complete_arm";
		}
	}
}

void calibrationFlag_1(int pi, unsigned int gpio, unsigned int edge, uint32_t tick){
	flag_z_1 = true;
	cout << "OK" << endl;
}

void calibrationFlag_2(int pi, unsigned int gpio, unsigned int edge, uint32_t tick){
	flag_z_2 = true;
	cout << "OK" << endl;
}
