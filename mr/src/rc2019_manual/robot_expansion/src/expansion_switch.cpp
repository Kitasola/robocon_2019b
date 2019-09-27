#include<ros/ros.h>
#include<std_msgs/String.h>
#include<pigpio.hpp>
#include"three_omuni/button.h"

std_msgs::String change;

void controller_callback(const three_omuni::button &button){
	switch(&button.data[10]){
		case 1:
			change.data = fase1;
			break;
		case 2:
			change.data = fase2;
			break;
		default:
			change.data = fase1;
			break;
	}
}		

int main(int argc, char **argv){
	ros::init(argc, argv, "expansion_switch");
	ros::NodeHandle n;
	ros::Publisher switch_pub = n.advertise<std_msgs::String>("expansion_switch_info", 10);
	ros::Subscriber switch_sub = n.subscribe("controller_info", 10, controller_callback);
	ros::Rate loop_rate(100);
	while(ros::ok()){
		switch_pub.publish(change);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
 
