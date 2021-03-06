#include<iostream>
#include<cmath>
#include<ros/ros.h>
#include<std_msgs/Float64MultiArray.h>
#include"motor_serial/motor_serial.h"

using std::cout;
using std::endl;

std_msgs::Float64MultiArray angle;
std_msgs::Float64MultiArray angle_check;

constexpr int A = 408;
constexpr int B = 286;
constexpr double calibration_angle_1 = M_PI / 2;
constexpr double calibration_angle_2 = M_PI - (M_PI / 3);

double x, y;

void positionCallback(const std_msgs::Float64MultiArray &position);
void angle_calcurate(double *angle_1, double *angle_2);

int main(int argc, char **argv){
	ros::init(argc, argv, "robot_arm");
	ros::NodeHandle n;
	ros::Publisher robot_arm_pub = n.advertise<std_msgs::Float64MultiArray>("arm_angle", 10);
	ros::Publisher check_pub = n.advertise<std_msgs::Float64MultiArray>("check", 10);
	ros::Subscriber robot_arm_sub = n.subscribe("angle_info", 10, positionCallback);
	ros::ServiceClient arm_client = n.serviceClient<motor_serial::motor_serial>("robot_arm");

	motor_serial::motor_serial srv;

	double angle_1 = 0, angle_2 = 0;
	int ARM_ID[2] = {5, 5};
	int ARM_CMD[2] = {61, 62};
	angle_check.data.resize(2);
	angle.data.resize(2);
	ros::Rate loop_rate(100);

	while(ros::ok()){
		angle_calcurate(&angle_1, &angle_2);
		//angle.data[0] = (M_PI / 2) - angle_1;
		//angle.data[1] = angle_1 - angle_2;
		//angle_check.data[0] = angle_1 * (180 / M_PI);
		//angle_check.data[1] = angle_2 * (180 / M_PI);
		if(angle_1 >= M_PI) angle_1 = M_PI;
		if(angle_2 >= M_PI) angle_2 = M_PI;
		ROS_INFO("angle_1 = %f", angle_1);
		ROS_INFO("angle_2 = %f", angle_2);
		angle.data[0] = (angle_1 - calibration_angle_1) * (180 / M_PI);
		angle.data[1] = (angle_2 - calibration_angle_2) * (180 / M_PI);
		ROS_INFO("angle_1 = %d", (int)angle.data[0]);
		ROS_INFO("angle_2 = %d", (int)angle.data[1]);
		angle_check.data[0] = angle.data[0];
		angle_check.data[1] = angle.data[1];
		for(int i = 0; i < 2; ++i){
			srv.request.id = (unsigned int)ARM_ID[i];
			srv.request.cmd = (unsigned int)ARM_CMD[i];
			srv.request.data = (int)angle.data[i];
			arm_client.call(srv);
		}
		robot_arm_pub.publish(angle);
		check_pub.publish(angle_check);
		ros::spinOnce();
		loop_rate.sleep();
	}
}

void positionCallback(const std_msgs::Float64MultiArray &position){
	x = (double)position.data[0];
	y = (double)position.data[1];
}

void angle_calcurate(double *angle_1, double *angle_2){
	double r = sqrt(pow(x, 2) + pow(y, 2));
	*angle_2 = (double)acos(-1 * (pow(r, 2) - pow(A, 2) - pow(B, 2)) / (2 * A * B));
	double theta_1 = atan2(y, x);
	double theta_2 = acos(-1 * (pow(B, 2) - pow(A, 2) - pow(r, 2)) / (2 * A * r));
	*angle_1 = (double)theta_1 + theta_2;
	angle_check.data[0] = *angle_1;
	if(*angle_1 >= -M_PI && *angle_1 <= M_PI){
	}else{
		*angle_1 = M_PI;
	}
	if(*angle_2 >= -M_PI && *angle_2 <= M_PI){
	}else{
		*angle_2 = M_PI;
	}
}
