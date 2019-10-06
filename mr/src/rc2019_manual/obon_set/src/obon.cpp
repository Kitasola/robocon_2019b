#include<ros/ros.h>
#include<three_omuni/button.h>
#include"motor_serial/motor_serial.h"

bool flag_hand = false;
void controllerCallback(const three_omuni::button &msg){
	msg.hand == true ? flag_hand = true : flag_hand = false;
}
int main(int argc, char **argv){
	ros::init(argc, argv, "robot_hand");
	ros::NodeHandle n;
	ros::Subscriber controller_sub = n.subscribe("controller_info", 10, controllerCallback);
	ros::ServiceClient robot_hand = n.serviceClient<motor_serial::motor_serial>("hand_info");
	motor_serial::motor_serial srv;

	int data;
	flag_hand == true ? data = 180 : data = 0;
	srv.request.id = 3;
	srv.request.cmd = 40;
	srv.request.data = data;
	robot_hand.call(srv);
	ros::spin();
}

