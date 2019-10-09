#include<ros/ros.h>
#include"motor_serial/motor_serial.h"
#include<pigpiod_if2.h>
#include<pigpiod.hpp>

using ros:Pigpiod;

bool flag_nomal = false; //5
bool flag_warning = false; //yellow 1
bool flag_calibration = false; //green 2
bool flag_red_coat = false; //red 3
bool flag_blue_coat = false; //blue 4
constexpr int WARNING_PIN = 1;

Pigpiod::gpio().set(WARNING_PIN, ros::IN, ros::PULL_UP);	

void calibrationCallback(const std_msgs::String &msg){
	if(msg.data == "gyro")flag_calibration = true;
	if(msg.data == "complete")flag_calibration = false;
	
}

bool warning(){
	bool current_data;
	if(ros::Pigpiod::gpio().read(WARNING_PIN) == 1){
		current_data = true;
	}else{
		current_data = false;
	}
}

int main(int argc, char **argv){
	ros::init(argc, argv, "led");
	ros::NodeHandle n;
	ros::ServiceClient tape_led = n.serviceClient<motor_serial::motor_serial>("tape_led");
	ros::Subscriber calibration_sub = n.subscribe("calibration", 10, calibrationCallback);
	motor_serial::motor_serial srv;
	int led_data = 0;
	ros::Rate loop_rate(100);

	while(ros::ok()){
		flag_warning = warning();	
		if(flag_warning){
			led_data = 1;
		}else if(flag_calibration){
			led_data = 2;
		}else{
			led_data = 0;
		}

		srv.request.id = 6;
		srv.request.cmd = 1;
		srv.request.data = led_data;
			
		ros::spinOnce();
		loop_rate.sleep();
	}
}

