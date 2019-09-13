#include<ros/ros.h>
#include"/home/ubuntu/arrc_utility/raspi/include/GY521.hpp"
#include<std_msgs/Float64.h>
#include<std_msgs/String.h>

using ros::GY521;
std_msgs::Float64 msg;

bool calibration_switch = false;

void calibration_callback(const std_msgs::String &file_send){
	if(file_send.data == "gyro"){
		calibration_switch = true;
	}
}			

int main(int argc, char **argv){
	ros::init(argc, argv, "gyro_info");
	ros::NodeHandle n;
	ros::Publisher gyro_pub = n.advertise<std_msgs::Float64>("gyro_info", 10);
	ros::Subscriber calibration_sub = n.subscribe("calibration", 30, calibration_callback);
	ros::Publisher calibration_pub = n.advertise<std_msgs::String>("calibration_answer", 30);
	std_msgs::String file_answer;
	while(calibration_switch == false){
		ros::spinOnce();
	}
	GY521 gyro;
	file_answer.data = "complete_gyro";
	calibration_pub.publish(file_answer);
	gyro.start();
	ros::Rate loop_rate(1000);
	while(ros::ok()){
		gyro.update();
		msg.data = gyro.yaw_;
		gyro_pub.publish(msg);
		ros::spinOnce();
		loop_rate.sleep();
	}
	return 0;
}
