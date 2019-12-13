#include<ros/ros.h>
#include<std_msgs/String.h>
#include"three_omuni/button.h"

std_msgs::String change;

constexpr int count_max = 4;

void controller_callback(const three_omuni::button &button){
	int count_dummy = 0;
	static int count_now = 0;
	static int count_prev_up = 0;
	static int count_prev_down = 0;
	if(button.hand && button.expansion_up){
		button.expansion_up != count_prev_up ? ++count_now : count_dummy = 0;
		count_prev_up >= count_max ? count_now = count_max : count_dummy = 0;
		count_prev_up = button.expansion_up;
	}

	if(button.hand && button.expansion_down){
		button.expansion_down != count_prev_down ? --count_now : count_dummy = 0;
		count_prev_down = 0 ? count_now = 0 : count_dummy = 0;
		count_prev_down = button.expansion_down;
	}
	
	switch(count_now){
		case 1:
			change.data = "fase1";
			break;
		case 2:
			change.data = "fase2";
			break;
		default:
			change.data = "fase3";
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

