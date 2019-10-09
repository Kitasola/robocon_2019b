#include<ros/ros.h>
#include<three_omuni/button.h>
#include"motor_serial/motor_serial.h"
#include<iostream>
#include<mutex>

ros::ServiceClient shot;
motor_serial::motor_serial srv;

void speedInit(){
	srv.request.id = 3;
	srv.request.cmd = 31;
	srv.request.data = 10;
	shot.call(srv);
}
bool shooting_prev = false;
bool loading_prev = false;
void controllerCallback(const three_omuni::button &button){
	static bool loading_flag_prev = true;
	static bool hand_flag_prev = false;
	static int times = 0;
	static std::once_flag flag;
	std::call_once(flag, speedInit);
	if(button.loading){
		if(loading_prev == false){
			if(loading_flag_prev == false){
				srv.request.id = 3;
				srv.request.cmd = 34;
				srv.request.data = 1;
				shot.call(srv);
				loading_flag_prev = true;
				ROS_INFO("loading_false");
			}else{
				srv.request.id = 3;
				srv.request.cmd = 34;
				srv.request.data = -1;
				loading_flag_prev = false;
				shot.call(srv);
				srv.request.id = 4;
				srv.request.cmd = 20;
				srv.request.data = times;
				shot.call(srv);
				++times;
				ROS_INFO("loading true");
			}
			loading_prev = true;
		}
	}else{
		loading_prev = false;
	}
	if(button.shooting){
		if(shooting_prev == false){
			srv.request.id = 3;
			srv.request.cmd = 30;
			srv.request.data = 20;
			shot.call(srv);
			ROS_INFO("OK");
			shooting_prev = true;
		}
	}else{
		shooting_prev = false;
	}
	/*if(button.hand){
	  if(hand_flag_prev == false){
	  srv.request.id = 3;
	  srv.request.cmd = 40;
	  srv.request.data = 90;
	  hand_flag_prev = true;
	  }else if(hand_flag_prev == true){
	  srv.request.id = 3;
	  srv.request.cmd = 40;
	  srv.request.data = 180;
	  hand_flag_prev = false;
	  }
	  }*/
	if(times >= 8) times = 0;
}
int main(int argc, char **argv){
	ros::init(argc, argv, "shotting_cloths");
	ros::NodeHandle n;
	ros::Subscriber shot_sub = n.subscribe("controller_info", 10, controllerCallback);
	shot = n.serviceClient<motor_serial::motor_serial>("shotting_cloths");
	ros::spin();
	return 0;
}
