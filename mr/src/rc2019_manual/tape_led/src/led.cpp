#include<ros/ros.h>
#include "motor_serial/motor_serial.h"
#include<pigpiod_if2.h>
#include<pigpiod.hpp>
#include<std_msgs/String.h>

using ros::Pigpiod;
std_msgs::String msg;

bool flag_nomal = false; //5
bool flag_warning = false; //yellow 1
bool flag_calibration = false; //green 2
bool flag_red_coat = false; //red 3
bool flag_blue_coat = false; //blue 4
constexpr int WARNING_PIN = 16;
int gpio_handle;

void calibrationCallback(const std_msgs::String &msg){
	if(msg.data == "gyro")flag_calibration = true;
	if(msg.data == "complete")flag_calibration = false;
	
}

bool warning(){
	bool current_data;
	if(ros::Pigpiod::gpio().read(WARNING_PIN) == 1){
		flag_warning = true;
	}else{
		flag_warning = false;
	}
	return flag_warning;
}

int main(int argc, char **argv){
	ros::init(argc, argv, "led");
	ros::NodeHandle n;
	ros::ServiceClient tape_led = n.serviceClient<motor_serial::motor_serial>("tape_led");
	ros::Subscriber calibration_sub = n.subscribe("calibration", 10, calibrationCallback);
	ros::Publisher led_pub = n.advertise<std_msgs::String>("led_info", 10);
	motor_serial::motor_serial srv;
	int led_data = 0;
	gpio_handle = Pigpiod::gpio().checkHandle();
	Pigpiod::gpio().set(WARNING_PIN, ros::IN, ros::PULL_UP);	
	ros::Rate loop_rate(100);

	while(ros::ok()){
		flag_warning = warning();	
		if(flag_warning){
			led_data = 1;
			msg.data = "RED";
		}else if(flag_calibration){
			led_data = 2;
			msg.data = "GREEN";
		}else{
			led_data = 0;
			msg.data = "NOMAL";
		}

		srv.request.id = 6;
		srv.request.cmd = 100;
		srv.request.data = led_data;
		tape_led.call(srv);
		led_pub.publish(msg);
			
		ros::spinOnce();
		loop_rate.sleep();
	}
}

