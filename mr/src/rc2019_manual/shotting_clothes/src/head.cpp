#include<ros/ros.h>
#include"motor_serial/motor_serial.h"
#include<three_omuni/button.h>
ros::ServiceClient head;
motor_serial::motor_serial srv;
void controllerCallback(const three_omuni::button &button){
	if(button.speed_half && button.hand){
		srv.request.id = 6;
		srv.request.cmd = 250;
		srv.request.data = 1;
		head.call(srv);
	}
}
int main(int argc, char **argv){
	ros::init(argc, argv, "head");
	ros::NodeHandle n;
	ros::Subscriber head_sub = n.subscribe("controller_info", 10, controllerCallback);
	head = n.serviceClient<motor_serial::motor_serial>("head");
	ros::spin();
	return 0;
}
